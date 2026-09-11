// ============================================================================
// CloudSceneBuilder.h
// 装配层：把分块数据 + 位姿纹理 + 位姿着色器组装成可直接挂进场景图的子树
// （设计文档 §3.4 双层共享几何体 / §4.1 场景图结构 / Phase 2b 接线层）
//
// 分工：
//   CloudRenderData   —— 数据（帧局部顶点、帧号属性、LOD 索引、动态世界 AABB）
//   CloudPoseBuffer   —— 位姿表 → 每槽一张 samplerBuffer 纹理
//   CoreShaders       —— 位姿点云着色器 + Turbo LUT
//   CloudGeometry     —— osg::Geometry 子类（屏蔽原生相交器、动态世界包围盒）
//   CloudPickProxy    —— 常驻拾取代理（拾取与驻留解耦）
//   **本文件**        —— 把上面这些按 §4.1 的结构组装起来，并提供运行时更新入口
//
// 场景图结构（**两层共享同一批 Geometry**，OSG 场景图是 DAG，允许一个 Drawable
// 有多个父节点 —— 这是"双层 = 1× 显存"的全部来源）：
//
//   cloudRoot (Group)
//     ├── optimizedLayer (Group)  StateSet: 位姿纹理=优化槽, uColorMode=0
//     │     └── cloudGeode (Geode) ── CloudGeometry × chunk   ←┐ 同一批对象
//     ├── originalLayer  (Group)  StateSet: 位姿纹理=原始槽, uColorMode=1  ←┘
//     │     └── （同一个 cloudGeode，ref_ptr 共享，不是拷贝）
//     └── pickProxyGeode (Geode → CloudPickProxy)   ← 常驻，不渲染
//
// StateSet 组织（唯一容易踩坑的地方）：
//   · **层级的量挂父 Group**，靠状态继承向下传播：位姿纹理、uColorMode、
//     uConstantColor、uColorZRange、uOpacity、uPointSize、Z 裁剪。
//     两层各自把**自己槽位**的纹理绑到同一个采样器单元 —— 这就是"双层共享顶点、
//     只换位姿纹理"的落地方式，着色器因此不需要 uPoseOffset 之类的分支。
//   · **每 chunk 的量挂 Geometry**：只有 uFrameIdBias 一个。因为 Geometry 是两层
//     共享的，这个值对两层必须一致（帧号基准与层无关），所以挂在这里既正确又省。
//     注意绝**不能**把它挂层级 StateSet：那样浅拷贝给 chunk 时会共享同一个
//     Uniform 对象，给某个 chunk 设值会污染其余 chunk（症状：除第一个 chunk 外
//     整体错位）。
//
// ⚠️ 前提（设计文档 §4.1 不变量 I1）：本子树**不得**挂在非恒等变换下。着色器算出的
// 世界坐标与 osg_ModelViewProjectionMatrix 里的模型矩阵只有恒等时才不冲突；
// CloudGeometry::computeBoundingBox 返回世界 AABB、CloudRayIntersector 用世界射线，
// 也都基于同一前提。
// ============================================================================

#pragma once

#include "visualizers/CloudGeometry.h"
#include "visualizers/CloudPickSource.h"
#include "visualizers/CloudPoseBuffer.h"
#include "visualizers/CloudRayIntersector.h"
#include "visualizers/CoreShaders.h"

#include <osg/BoundingBox>
#include <osg/CopyOp>
#include <osg/Geode>
#include <osg/Group>
#include <osg/Node>
#include <osg/StateSet>
#include <osg/Uniform>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace hdl_graph_slam {

/**
 * @brief 点云场景子树装配器（两层共享同一批几何体，各绑一张位姿纹理）
 *
 * 用法：
 * @code
 *   CloudPoseBuffer poseBuf;
 *   poseBuf.rebuild(poses);
 *   CloudSceneBuilder scene;
 *   scene.build(res.chunks, opt.chunkFrames, poseBuf, 是否要原始层);
 *   cloudRoot->addChild(scene.root());
 *   // 位姿优化后（不重建顶点、不重传显存）：
 *   poses.updateOptimized(graph);
 *   poseBuf.updateOptimizedSlot(poses);
 *   scene.refreshWorldBounds(res.chunks, poses, CloudPoseTable::kSlotOptimized);
 * @endcode
 */
class CloudSceneBuilder {
public:
    /** @brief 原始层参照底图的常量色（与 PointCloudBuilder::odomColor 同值） */
    static constexpr float kOdomR = 0.98f;
    static constexpr float kOdomG = 0.72f;
    static constexpr float kOdomB = 0.45f;
    /** @brief 原始层 alpha 系数（再乘全局 uOpacity） */
    static constexpr float kOdomAlphaScale = 0.35f;

    CloudSceneBuilder() = default;

    // ------------------------------------------------------------------
    // 装配
    // ------------------------------------------------------------------

    /**
     * @brief 从分块结果装配场景子树（会替换已有内容）
     *
     * @param chunks        分块数据（buildChunks 的产物）
     * @param chunkFrames   每 chunk 帧数（与 buildChunks 时一致；决定帧号基准）
     * @param poseBuffer    位姿纹理（须已 rebuild）
     * @param withOriginal  是否装配原始层（槽 1）
     * @param pointSize     初始点大小（像素）
     * @return 装配出的 chunk 几何体总数（共享一份，两层不重复计数）
     */
    size_t build(const std::vector<CloudChunk>& chunks, uint32_t chunkFrames,
                 CloudPoseBuffer& poseBuffer, bool withOriginal, float pointSize = 3.0f) {
        m_root      = new osg::Group;
        m_root->setName("CloudRoot");
        m_geode     = new osg::Geode;
        m_geode->setName("CloudGeode");
        m_geoms.clear();
        m_chunkFrames = chunkFrames ? chunkFrames : 1u;

        // ---- 1) 只建**一份**几何体（顶点/帧号/LOD 索引数组都在这里） ----
        buildChunkGeometries(chunks);

        // ---- 2) 两层：各自的 StateSet（位姿纹理 + 颜色语义），共享同一个 Geode ----
        m_optimizedLayer = new osg::Group;
        m_optimizedLayer->setName("CloudOptimizedLayer");
        m_optimized = makeLayerState(m_optimizedLayer.get(), CloudPoseTable::kSlotOptimized,
                                     /*colorMode=*/0, osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f),
                                     poseBuffer.texture(CloudPoseTable::kSlotOptimized),
                                     pointSize);
        // DAG 多父：同一个 Geode 同时挂到两层 —— 顶点只存一份、两层各自渲染一次
        m_optimizedLayer->addChild(m_geode.get());
        m_root->addChild(m_optimizedLayer.get());

        if (withOriginal) {
            m_originalLayer = new osg::Group;
            m_originalLayer->setName("CloudOriginalLayer");
            m_original = makeLayerState(m_originalLayer.get(), CloudPoseTable::kSlotOriginal,
                                        /*colorMode=*/1,
                                        osg::Vec4(kOdomR, kOdomG, kOdomB, kOdomAlphaScale),
                                        poseBuffer.texture(CloudPoseTable::kSlotOriginal),
                                        pointSize);
            m_originalLayer->addChild(m_geode.get());   // 同一个 Geode，ref_ptr 共享
            m_originalLayer->setNodeMask(0);            // 原始层默认隐藏，由开关控制
            m_root->addChild(m_originalLayer.get());
        }

        // ---- 3) 常驻拾取代理（不渲染，只保证拾取不依赖渲染状态） ----
        m_pickProxy = new CloudPickProxy;
        m_pickProxyGeode = new osg::Geode;
        m_pickProxyGeode->setName("CloudPickProxy");
        m_pickProxyGeode->addDrawable(m_pickProxy.get());
        m_root->addChild(m_pickProxyGeode.get());

        // 同步一次世界包围盒（buildChunks 已算好每 chunk 的世界 AABB）
        syncWorldBounds(chunks);
        return m_geoms.size();
    }

    // ------------------------------------------------------------------
    // 运行时更新
    // ------------------------------------------------------------------

    /**
     * @brief 位姿变化后刷新世界包围盒（Phase 3 的落地入口）
     *
     * 先让数据层按新位姿重算每 chunk 的世界 AABB（O(帧数)，实测 27 chunk /
     * 6836 帧约 0.3 ms），再把结果推给各 CloudGeometry 并 dirtyBound()，
     * 使 OSG 原生视锥剔除重新正确。**不动顶点、不重传显存**。
     *
     * @return 数据层重算耗时（毫秒）
     */
    double refreshWorldBounds(std::vector<CloudChunk>& chunks,
                              const CloudPoseTable& poses, uint32_t slot) {
        const double ms = refreshAllWorldBounds(chunks, poses, slot);
        syncWorldBounds(chunks);
        return ms;
    }

    /** @brief 切换 LOD 级别（替换各 chunk 的 primitive set，无顶点重传） */
    void setActiveLodLevel(int level) {
        for (auto& g : m_geoms) if (g.valid()) g->setActiveLodLevel(level);
        m_activeLod = level;
    }
    int activeLodLevel() const { return m_activeLod; }

    /** @brief 点大小（像素） */
    void setPointSize(float px) { setBothUniform("uPointSize", px); }
    /** @brief 全局透明度（O(1)，不必重着色） */
    void setOpacity(float a)    { setBothUniform("uOpacity", a); }

    /** @brief 颜色映射的世界 Z 范围（对应构建时的 colorZMin/Max） */
    void setColorZRange(float zMin, float zMax) {
        if (!(zMax > zMin)) return;
        setBothUniform("uZRange", osg::Vec2(zMin, zMax));
    }

    /** @brief Z 轴裁剪（沿用旧 shader 的同名 uniform 语义） */
    void setZClipping(bool on, float zMin, float zMax) {
        setBothUniform("z_clipping", on ? 1 : 0);
        setBothUniform("z_range", osg::Vec2(zMin, zMax));
    }

    /** @brief 某层可见性（两层独立控制；原始层不存在时是空操作） */
    void setLayerVisible(uint32_t slot, bool visible) {
        osg::Group* g = layerGroup(slot);
        if (g) g->setNodeMask(visible ? ~0u : 0u);
    }
    bool layerVisible(uint32_t slot) const {
        const osg::Group* g = layerGroup(slot);
        return g && g->getNodeMask() != 0u;
    }

    // ------------------------------------------------------------------
    // 访问（接线与自检用）
    // ------------------------------------------------------------------

    osg::Group* root() const { return m_root.get(); }
    osg::Group* layerGroup(uint32_t slot) const {
        return slot == CloudPoseTable::kSlotOriginal ? m_originalLayer.get()
                                                     : m_optimizedLayer.get();
    }
    osg::StateSet* layerStateSet(uint32_t slot) const {
        return slot == CloudPoseTable::kSlotOriginal ? m_original.stateSet.get()
                                                     : m_optimized.stateSet.get();
    }
    /** @brief **共享**的那一个 Geode（两层的子节点是同一个对象） */
    osg::Geode* cloudGeode() const { return m_geode.get(); }
    const std::vector<osg::ref_ptr<CloudGeometry>>& geometries() const { return m_geoms; }
    CloudPickProxy* pickProxy() const { return m_pickProxy.get(); }

    /** @brief 整图世界 AABB（resetCamera 应当用它，而不是 scene->getBound()） */
    const osg::BoundingBox& mapBounds() const { return m_mapBounds; }

    /** @brief 每 chunk 帧数（拾取侧 ChunkPickSource 需要同一个值） */
    uint32_t chunkFrames() const { return m_chunkFrames; }

    /** @brief 顶点总数（两层共享同一份，所以这就是显存里的那一份） */
    size_t vertexCount() const {
        size_t n = 0;
        for (const auto& g : m_geoms) {
            if (g.valid() && g->getVertexArray()) n += g->getVertexArray()->getNumElements();
        }
        return n;
    }

    /** @brief 供拾取使用的帧级数据源（优化层或原始层） */
    std::shared_ptr<const CloudPickSource> makePickSource(
        const std::vector<CloudChunk>* chunks, const CloudPoseTable* poses,
        uint32_t slot) const {
        return std::make_shared<ChunkPickSource>(
            chunks, m_chunkFrames, poses, slot,
            slot == CloudPoseTable::kSlotOriginal ? 1 : 0);
    }

private:
    /** @brief 一层的 StateSet 与其全局 uniform 句柄（几何体是两层共享的） */
    struct LayerState {
        osg::ref_ptr<osg::StateSet> stateSet;
        PoseCloudUniforms uniforms;
    };

    static LayerState makeLayerState(osg::Group* layer, uint32_t slot, int colorMode,
                                     const osg::Vec4& constantColor,
                                     osg::TextureBuffer* poseTex, float pointSize) {
        LayerState ls;
        ls.stateSet = new osg::StateSet;
        ls.stateSet->setName(slot == CloudPoseTable::kSlotOriginal ? "CloudOriginalSS"
                                                                  : "CloudOptimizedSS");
        ls.uniforms = applyPosePointCloudShader(ls.stateSet.get(), pointSize);
        ls.uniforms.colorMode->set(colorMode);
        ls.uniforms.constantColor->set(constantColor);
        // 本层槽位的位姿纹理绑到约定的采样器单元（两层用同一个单元号）
        bindPoseTableTexture(ls.stateSet.get(), poseTex);
        layer->setStateSet(ls.stateSet.get());
        return ls;
    }

    void buildChunkGeometries(const std::vector<CloudChunk>& chunks) {
        m_geoms.reserve(chunks.size());
        for (const CloudChunk& c : chunks) {
            if (c.positions.empty()) continue;   // 空 chunk 不建几何体

            osg::ref_ptr<osg::Vec3Array>   pos = new osg::Vec3Array(c.pointCount);
            osg::ref_ptr<osg::UShortArray> fid = new osg::UShortArray(c.pointCount);
            for (uint32_t i = 0; i < c.pointCount; ++i) {
                (*pos)[i].set(c.positions[3 * i], c.positions[3 * i + 1],
                              c.positions[3 * i + 2]);
                (*fid)[i] = (i < c.frameLocalId.size())
                                ? static_cast<unsigned short>(c.frameLocalId[i]) : 0;
            }

            // LOD 索引：c.lodIndices[0] 恒为空（L0 = 全量直绘），[L>=1] 对应 level L
            std::vector<osg::ref_ptr<osg::DrawElementsUInt>> lod;
            if (c.lodIndices.size() > 1) lod.reserve(c.lodIndices.size() - 1);
            for (size_t L = 1; L < c.lodIndices.size(); ++L) {
                if (c.lodIndices[L].empty()) continue;
                lod.push_back(new osg::DrawElementsUInt(GL_POINTS,
                                                        c.lodIndices[L].begin(),
                                                        c.lodIndices[L].end()));
            }

            osg::ref_ptr<CloudGeometry> g = new CloudGeometry;
            g->setCloudData(pos.get(), fid.get(), lod);

            // 每 chunk 的 StateSet：**只**放 uFrameIdBias（几何体是两层共享的，
            // 帧号基准与层无关）。位姿程序/纹理/其余 uniform 全部由父 Group 继承。
            osg::ref_ptr<osg::StateSet> ss = new osg::StateSet;
            ss->setName("CloudChunkSS");
            setPoseCloudFrameBias(ss.get(), static_cast<int>(c.firstFrameIndex));
            g->setStateSet(ss.get());

            g->setWorldBounds(boundsOf(c));

            m_geode->addDrawable(g.get());
            m_geoms.push_back(g);
        }
    }

    static osg::BoundingBox boundsOf(const CloudChunk& c) {
        if (!c.worldBoundsValid) return osg::BoundingBox();   // 无效盒 = 不剔除
        return osg::BoundingBox(c.worldMin[0], c.worldMin[1], c.worldMin[2],
                                c.worldMax[0], c.worldMax[1], c.worldMax[2]);
    }

    /** @brief 把数据层算好的世界 AABB 推给各几何体并 dirtyBound() */
    void syncWorldBounds(const std::vector<CloudChunk>& chunks) {
        osg::BoundingBox all;
        size_t gi = 0;
        for (const auto& c : chunks) {
            if (c.positions.empty()) continue;
            const osg::BoundingBox bb = boundsOf(c);
            if (gi < m_geoms.size() && m_geoms[gi].valid()) m_geoms[gi]->setWorldBounds(bb);
            if (bb.valid()) all.expandBy(bb);
            ++gi;
        }
        m_mapBounds = all;
        if (m_pickProxy.valid()) m_pickProxy->setWorldBounds(all);
    }

    /** @brief 给两层的同名 uniform 写同一个值（全局量；两层各持有一份 Uniform 对象） */
    template <typename T>
    void setBothUniform(const char* name, const T& value) {
        osg::Uniform* a = m_optimized.stateSet.valid()
                              ? m_optimized.stateSet->getUniform(name) : nullptr;
        osg::Uniform* b = m_original.stateSet.valid()
                              ? m_original.stateSet->getUniform(name) : nullptr;
        if (a) a->set(value);
        if (b) b->set(value);
    }

    osg::ref_ptr<osg::Group>  m_root;
    osg::ref_ptr<osg::Group>  m_optimizedLayer;   ///< 优化层父 Group（StateSet 在此）
    osg::ref_ptr<osg::Group>  m_originalLayer;    ///< 原始层父 Group（可选）
    osg::ref_ptr<osg::Geode>  m_geode;            ///< **两层共享**的 Geode
    osg::ref_ptr<osg::Geode>  m_pickProxyGeode;
    LayerState                m_optimized;
    LayerState                m_original;
    osg::ref_ptr<CloudPickProxy> m_pickProxy;
    std::vector<osg::ref_ptr<CloudGeometry>> m_geoms;
    uint32_t                  m_chunkFrames = 256;
    int                       m_activeLod = 0;
    osg::BoundingBox          m_mapBounds;
};

}  // namespace hdl_graph_slam
