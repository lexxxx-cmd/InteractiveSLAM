// ============================================================================
// KeyframePointCloudVisualizer.h
// 关键帧点云可视化器
//
// 功能：渲染世界坐标系中合并的点云几何体。点云数据（全量世界点、降采样
//       渲染索引/范围、顶点数组、Z 统计）由 PointCloudBuilder 在后台线程
//       构建（见 PointCloudBuilder.h），本类通过 commitBuild() 以 swap
//       方式换入场景——主线程 O(1) 交换，旧几何体持续渲染到新数据就绪，
//       避免上千万点重建阻塞 UI 线程。
//
//       记录每个关键帧的渲染顶点范围，使得选中关键帧时可以仅重新着色
//       该帧的点云，而无需重建整个 VBO。
//
// 分块上传：每个 LOD 级别的顶点/颜色数组按固定块大小拆分（builder 后台
//       生成），每块一个独立 Geometry 挂到同一 geode，共享同一 StateSet。
//       commit 后逐帧显示一块（ViewportWidget::updateScene 驱动
//       advanceChunkUpload()），OSG 首次绘制某块时才上传该块 VBO——
//       把一次超大上传（如 560MB）摊成多次小块上传（每块 ~18MB），
//       避免"换回主线程那一帧"的 GPU 卡顿。
//
// 着色方案：默认使用 Turbo 颜色映射根据高程（Z 值）着色（颜色由 builder
//           在后台生成，主线程 commit 零遍历）。
//           选中关键帧的点云高亮为白色。
// ============================================================================

#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/Uniform>
#include <osg/BlendFunc>
#include <osg/BlendColor>

#include <limits>
#include <set>
#include <utility>
#include <vector>

#include "visualizers/PointCloudBuilder.h"
#include "visualizers/TurboColormap.h"
#include "visualizers/CoreShaders.h"

/**
 * @brief 关键帧点云可视化器
 *
 * 持有渲染所需的状态与交互逻辑（着色、高亮、裁剪、透明度），
 * 数据本身由 PointCloudBuilder 后台构建后经 commitBuild() 换入。
 *
 * 核心功能：
 *   1. commitBuild() —— 将构建结果 swap 进场景（主线程调用）
 *   2. recolorHighlight() —— 高亮选中的关键帧点云（白色）
 *   3. Z 轴裁剪（着色器级别，无需重建 VBO）
 *   4. 高程颜色范围动态调整（CPU 重新着色，无需重建顶点）
 */
class KeyframePointCloudVisualizer {
public:
    /** @brief 构造函数：初始化点云几何体和着色器 */
    KeyframePointCloudVisualizer() {
        // 共享 StateSet：所有分块几何体共用，uniform 一次设置全局生效
        m_cloudStateSet = new osg::StateSet;
        m_pointSizeUniform = applyPointCloudShader(m_cloudStateSet, m_pointSize);
        m_zClipUniform  = m_cloudStateSet->getUniform("z_clipping");
        m_zRangeUniform = m_cloudStateSet->getUniform("z_range");
        m_cloudStateSet->setAttributeAndModes(
            new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON);

        m_geode = new osg::Geode;

        // 高亮几何体：选中帧的全量点云（纯白），与 LOD 主几何体叠加。
        // 它始终使用全量数据渲染，不受 LOD 级别切换影响。
        m_highlightGeom = new osg::Geometry;
        m_highlightGeom->setUseDisplayList(false);
        m_highlightGeom->setUseVertexBufferObjects(true);
        m_highlightGeom->setUseVertexArrayObject(true);
        m_highlightGeom->setDataVariance(osg::Object::DYNAMIC);

        m_highlightVertices = new osg::Vec3Array;
        m_highlightColors   = new osg::Vec4Array;

        m_highlightGeom->setVertexArray(m_highlightVertices);
        m_highlightGeom->setColorArray(m_highlightColors, osg::Array::BIND_PER_VERTEX);
        m_highlightGeom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, 0));

        // 高亮点云着色器（支持点大小；Z 裁剪与主几何体同步）
        auto* hss = m_highlightGeom->getOrCreateStateSet();
        m_highlightPointSizeUniform = applyPointCloudShader(hss, m_pointSize);
        m_highlightZClipUniform  = hss->getUniform("z_clipping");
        m_highlightZRangeUniform = hss->getUniform("z_range");
        hss->setAttributeAndModes(
            new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON);

        // 默认隐藏，选中时才显示
        m_highlightGeom->setNodeMask(0);
        m_geode->addDrawable(m_highlightGeom);
    }

    /** @brief 析构：清空场景图（geode 持有 ref_ptr，自动释放） */
    ~KeyframePointCloudVisualizer() = default;

    /**
     * @brief 换入后台构建的点云数据（主线程调用）
     *
     * 将 PointCloudBuilder 构建结果换入本可视化器：
     *   - 全量世界点 / 全量范围 / Z 值范围与包围盒统计
     *   - 主级别渲染数据 + 多级 LOD（全量模式生成）
     * 顶点/颜色数组已由 builder 分块并着色，主线程零遍历；
     * 提交后逐帧显示各块（渐进上传，见 advanceChunkUpload()）。
     *
     * @param result 后台构建结果（右值，内容被交换移入）
     */
    void commitBuild(hdl_graph_slam::PointCloudBuildResult&& result) {
        m_allWorldPoints.swap(result.allWorldPoints);
        m_cloudRanges.swap(result.cloudRanges);
        m_zMin = result.zMin;
        m_zMax = result.zMax;
        m_bMin = result.bMin;
        m_bMax = result.bMax;

        // 首次构建时从数据初始化裁剪范围
        if (!m_clipRangeInitialized && m_zMax > m_zMin) {
            m_zClipMin = m_zMin;
            m_zClipMax = m_zMax;
            m_clipRangeInitialized = true;
        }
        if (m_zRangeUniform)
            m_zRangeUniform->set(osg::Vec2(m_zClipMin, m_zClipMax));

        // 记录构建时所用的颜色参数（协调层据此判断是否需要重新着色）
        m_colorZMin = result.colorZMinUsed;
        m_colorZMax = result.colorZMaxUsed;
        m_useAutoColorRange = result.colorUsedAuto;
        m_opacity   = result.opacityUsed;
        m_colorParamsValid = true;  // 各级颜色与构建时参数一致，无需重着色

        // 从 geode 移除旧的分块几何体（ref_ptr 自动释放）
        for (auto& chunk : m_allChunkGeoms) {
            m_geode->removeDrawable(chunk);
        }
        m_allChunkGeoms.clear();
        m_lodLevels.clear();
        m_chunkGlobalIndex = 0;  // 池化数组按块全局索引复用

        // 构建 LOD 级别列表：level 0 = 主级别，其后为 builder 生成的远级别
        m_lodLevels.reserve(1 + result.lodLevels.size());

        // level 0（主级别）
        m_lodLevels.emplace_back();
        LodLevelGeoms& l0 = m_lodLevels.back();
        l0.renderIndices.swap(result.renderIndices);
        l0.renderRanges.swap(result.renderRanges);
        l0.renderFullRes = result.renderFullRes;
        l0.totalPoints = 0;
        for (size_t c = 0; c < result.vertexChunks.size(); ++c) {
            osg::ref_ptr<osg::Vec3Array> v = result.vertexChunks[c];
            if (!v.valid()) v = new osg::Vec3Array;
            osg::ref_ptr<osg::Vec4Array> col;
            if (c < result.colorChunks.size()) col = result.colorChunks[c];
            if (!col.valid()) col = new osg::Vec4Array;
            addChunk(l0, v, col);
        }

        // LOD 级别 1..n
        for (auto& lod : result.lodLevels) {
            m_lodLevels.emplace_back();
            LodLevelGeoms& lg = m_lodLevels.back();
            lg.renderIndices.swap(lod.renderIndices);
            lg.renderRanges.swap(lod.renderRanges);
            lg.renderFullRes = lod.renderFullRes;
            lg.totalPoints = 0;
            for (size_t c = 0; c < lod.vertexChunks.size(); ++c) {
                osg::ref_ptr<osg::Vec3Array> v = lod.vertexChunks[c];
                if (!v.valid()) v = new osg::Vec3Array;
                osg::ref_ptr<osg::Vec4Array> col;
                if (c < lod.colorChunks.size()) col = lod.colorChunks[c];
                if (!col.valid()) col = new osg::Vec4Array;
                addChunk(lg, v, col);
            }
        }

        m_activeLodLevel = 0;
        m_levelUploaded.assign(m_lodLevels.size(), false);

        // 隐藏所有块，仅从 level 0 第一块开始渐进上传
        hideAllChunks();
        if (!m_lodLevels.empty() && !m_lodLevels[0].chunks.empty()) {
            m_uploadLevel = 0;
            m_uploadChunk = 0;
            m_lodLevels[0].chunks[0].geom->setNodeMask(~0u);
            m_uploadChunk = 1;
        } else {
            m_uploadLevel = -1;
        }

        // 全量数据已更新：若存在高亮，重建高亮几何体（全量白色）
        if (!m_highlightIds.empty()) {
            rebuildHighlightGeometry();
        }
    }

    /**
     * @brief 渐进上传推进（每帧由 ViewportWidget::updateScene 调用）
     *
     * 每次显示当前上传级别（m_uploadLevel）的下一块。OSG 首次绘制某块时
     * 才创建并上传该块 VBO，从而把一次超大上传摊成多次小块上传。
     * 全部块显示完毕后结束渐进流程。
     */
    void advanceChunkUpload() {
        if (m_uploadLevel < 0) return;
        LodLevelGeoms& lg = m_lodLevels[m_uploadLevel];
        if (m_uploadChunk < lg.chunks.size()) {
            lg.chunks[m_uploadChunk].geom->setNodeMask(~0u);
            m_uploadChunk++;
        }
        if (m_uploadChunk >= lg.chunks.size()) {
            m_levelUploaded[m_uploadLevel] = true;
            m_uploadLevel = -1;
            m_uploadChunk = 0;
        }
    }

    /**
     * @brief 是否仍有分块在渐进上传中
     *
     * true = 点云尚未全部显示（还有块待上传）；
     * false = 已全部显示（或未开始/无分块）。
     * 供 ViewportWidget 判断"点云渲染是否完成"。
     */
    bool chunkUploadPending() const { return m_uploadLevel >= 0; }

    /**
     * @brief 按相机距离切换到指定 LOD 级别（主线程调用）
     *
     * 目标级别的块若已上传过则全部直接显示；否则从第一块开始渐进上传。
     * 级别 0 = 主级别（全量），级别越远点数越少。
     * 高亮几何体使用全量数据，不受级别切换影响，无需重建。
     *
     * @param level 目标级别（0 .. lodLevelCount()-1）
     */
    void setLodLevel(int level) {
        if (level == m_activeLodLevel) return;
        if (level < 0 || (size_t)level >= m_lodLevels.size()) return;

        hideAllChunks();
        m_activeLodLevel = level;

        if (m_levelUploaded[level]) {
            // 该级别之前已完整上传：直接全部显示
            for (auto& chunk : m_lodLevels[level].chunks)
                chunk.geom->setNodeMask(~0u);
        } else if (!m_lodLevels[level].chunks.empty()) {
            // 首次进入该级别：渐进上传
            m_uploadLevel = level;
            m_uploadChunk = 0;
            m_lodLevels[level].chunks[0].geom->setNodeMask(~0u);
            m_uploadChunk = 1;
        }

        // 构建后用户改过颜色范围/透明度时，切换级别需用当前参数重新着色；
        // 未改动时颜色已由 builder 生成，直接使用（零遍历）
        if (!m_colorParamsValid) recolorAll();
        if (!m_highlightIds.empty()) applyHighlight();
    }

    /** @brief 当前激活的 LOD 级别 */
    int currentLodLevel() const { return m_activeLodLevel; }

    /** @brief LOD 级别总数（≥1，仅主级别时为 1） */
    int lodLevelCount() const { return static_cast<int>(m_lodLevels.size()); }

    /** @brief 点云世界包围盒中心（供相机距离计算） */
    Eigen::Vector3d boundsCenter() const { return (m_bMin + m_bMax) * 0.5; }

    /** @brief 点云世界包围球半径（供相机距离计算） */
    double boundsRadius() const { return (m_bMax - m_bMin).norm() * 0.5; }

    /** @brief 裁剪范围是否已初始化（首次构建后为 true） */
    bool isClipRangeInitialized() const { return m_clipRangeInitialized; }

    /**
     * @brief 清除高亮状态并恢复默认着色
     *
     * 用于无选中/无播放高亮时恢复所有点云为 Turbo 高程着色 + 当前透明度，
     * 并隐藏/清空高亮几何体。
     */
    void clearHighlight() {
        m_highlightIds.clear();
        recolorAll();
        m_highlightVertices->clear();
        m_highlightColors->clear();
        m_highlightGeom->setNodeMask(0);
    }

    /**
     * @brief 对指定关键帧集合的点云高亮
     *
     * 选中/高亮帧的点云以**全量数据**单独渲染为白色（独立高亮几何体，
     * 不受 LOD 级别影响），主几何体中的选中帧淡化、非选中帧半透明，
     * 形成"全量白色点云突出"的视觉层次。
     *
     * @param highlightIds 需要高亮的顶点 ID 集合（空集合 = 取消高亮）
     */
    void recolorHighlight(const std::set<long>& highlightIds) {
        m_highlightIds = highlightIds;
        applyHighlight();          // 主几何体：选中帧淡化，非选中帧半透明
        rebuildHighlightGeometry(); // 高亮几何体：选中帧全量白色点云
    }

    /**
     * @brief 清除所有数据（用于完全重建）
     */
    void clear() {
        m_allWorldPoints.clear();
        m_cloudRanges.clear();
        m_highlightIds.clear();
        m_lodLevels.clear();
        for (auto& chunk : m_allChunkGeoms) {
            m_geode->removeDrawable(chunk);
        }
        m_allChunkGeoms.clear();
        m_chunkGlobalIndex = 0;
        m_chunkVertexPool.clear();
        m_chunkColorPool.clear();
        m_uploadLevel = -1;
        m_uploadChunk = 0;
        m_levelUploaded.clear();
        m_activeLodLevel = 0;
        m_highlightVertices->clear();
        m_highlightColors->clear();
        m_highlightGeom->setNodeMask(0);
        m_zMin =  std::numeric_limits<float>::max();
        m_zMax = -std::numeric_limits<float>::max();
        m_bMin = Eigen::Vector3d( std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max());
        m_bMax = Eigen::Vector3d(-std::numeric_limits<double>::max(),
                                 -std::numeric_limits<double>::max(),
                                 -std::numeric_limits<double>::max());
        m_useAutoColorRange = true;
        m_colorParamsValid = true;
        m_clipRangeInitialized = false;
    }

    /** @brief 设置点的大小（像素单位，主几何体与高亮几何体同步） */
    void setPointSize(float size) {
        m_pointSize = size;
        if (m_pointSizeUniform)
            m_pointSizeUniform->set(size);
        if (m_highlightPointSizeUniform)
            m_highlightPointSizeUniform->set(size);
    }

    /** @brief 设置点云透明度 */
    void setOpacity(float opacity) {
        if (opacity == m_opacity) return;  // 无变化，避免无谓的全量重着色
        m_opacity = opacity;
        m_colorParamsValid = false;  // 颜色与构建时参数不一致，切换级别时需重着色
        if (!m_highlightIds.empty()) {
            // 高亮存在时同步更新主几何体着色与高亮几何体的 alpha
            applyHighlight();
            rebuildHighlightGeometry();
        } else {
            recolorAll();
        }
    }

    // ========================================================================
    // Z 轴裁剪控制（着色器级别，无需重建 VBO；高亮几何体同步）
    // ========================================================================

    /** @brief 启用/禁用 Z 轴裁剪 */
    void setZClipping(bool enabled) {
        m_zClipping = enabled;
        if (m_zClipUniform)
            m_zClipUniform->set(enabled ? 1 : 0);
        if (m_highlightZClipUniform)
            m_highlightZClipUniform->set(enabled ? 1 : 0);
    }

    /** @brief 设置 Z 轴裁剪范围 */
    void setZClipRange(float minZ, float maxZ) {
        m_zClipMin = minZ;
        m_zClipMax = maxZ;
        if (m_zRangeUniform)
            m_zRangeUniform->set(osg::Vec2(minZ, maxZ));
        if (m_highlightZRangeUniform)
            m_highlightZRangeUniform->set(osg::Vec2(minZ, maxZ));
    }

    /** @brief 查询 Z 裁剪状态 */
    bool isZClipping() const { return m_zClipping; }
    float getZClipMin() const { return m_zClipMin; }
    float getZClipMax() const { return m_zClipMax; }

    // ========================================================================
    // 高程颜色范围控制（CPU 重新着色，无需重建顶点）
    // ========================================================================

    /** @brief 手动设置颜色 Z 值范围（将关闭自动范围） */
    void setColorZRange(float minZ, float maxZ) {
        if (!m_useAutoColorRange && minZ == m_colorZMin && maxZ == m_colorZMax) return;
        m_colorZMin = minZ;
        m_colorZMax = maxZ;
        m_useAutoColorRange = false;
        m_colorParamsValid = false;  // 颜色与构建时参数不一致，切换级别时需重着色
        recolorAll();
    }

    /** @brief 设置是否自动计算颜色范围 */
    void setAutoColorRange(bool autoRange) {
        if (autoRange == m_useAutoColorRange) return;
        m_useAutoColorRange = autoRange;
        if (autoRange) {
            m_colorZMin = m_zMin;
            m_colorZMax = m_zMax;
        }
        m_colorParamsValid = false;  // 颜色与构建时参数不一致，切换级别时需重着色
        recolorAll();
    }

    /** @brief 查询是否使用自动颜色范围 */
    bool isAutoColorRange() const { return m_useAutoColorRange; }

    // ========================================================================
    // 数据范围访问（用于 UI 初始化）
    // ========================================================================

    float getDataZMin() const { return m_zMin; }     ///< 数据 Z 最小值
    float getDataZMax() const { return m_zMax; }     ///< 数据 Z 最大值
    float getColorZMin() const { return m_colorZMin; } ///< 颜色映射 Z 最小值
    float getColorZMax() const { return m_colorZMax; } ///< 颜色映射 Z 最大值

    /** @brief 获取 OSG 节点 */
    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

    /** @brief 返回点云中的渲染点数（当前 LOD 级别） */
    int pointCount() const {
        if (m_activeLodLevel < 0 || (size_t)m_activeLodLevel >= m_lodLevels.size())
            return 0;
        return static_cast<int>(m_lodLevels[m_activeLodLevel].totalPoints);
    }

private:
    /** @brief 单个分块几何体 */
    struct CloudChunk {
        osg::ref_ptr<osg::Geometry>  geom;       ///< 该块几何体
        osg::ref_ptr<osg::Vec3Array> vertices;   ///< 该块顶点数组
        osg::ref_ptr<osg::Vec4Array> colors;     ///< 该块颜色数组
        size_t renderStart = 0;                  ///< 该块在级别渲染数组中的起始逻辑索引
        size_t renderCount = 0;                  ///< 该块渲染点数
    };

    /** @brief 单个 LOD 级别的分块几何体集合 */
    struct LodLevelGeoms {
        std::vector<CloudChunk> chunks;          ///< 该级别所有块
        std::vector<size_t>     renderIndices;   ///< 渲染索引 -> 全量索引（该级别）
        std::vector<hdl_graph_slam::CloudRange> renderRanges; ///< 该级别逐帧渲染范围
        bool renderFullRes = true;               ///< true = 渲染索引与全量索引一致
        size_t totalPoints = 0;                  ///< 该级别总渲染点数
    };

    /**
     * @brief 为某级别添加一个分块几何体
     *
     * 每块独立的 Geometry（共享 m_cloudStateSet），挂到 geode。
     * renderStart 为该块在级别渲染数组中的起始逻辑索引（块间连续）。
     *
     * 数组对象复用（方案 C）：跨构建时按块全局索引复用同一
     * osg::Vec3Array/Vec4Array 对象（swap 数据而非换新对象），使 OSG 的
     * BufferObject 跨构建保持；块大小不变时上传走 glBufferSubData
     * 增量更新，避免每次重建都重新分配 GPU buffer。
     */
    void addChunk(LodLevelGeoms& lg, osg::ref_ptr<osg::Vec3Array> v,
                  osg::ref_ptr<osg::Vec4Array> col) {
        // 复用池化数组对象：交换数据后 geometry 引用池化对象
        if (m_chunkGlobalIndex < m_chunkVertexPool.size() &&
            m_chunkVertexPool[m_chunkGlobalIndex].valid() &&
            m_chunkColorPool[m_chunkGlobalIndex].valid()) {
            m_chunkVertexPool[m_chunkGlobalIndex]->swap(*v);
            m_chunkColorPool[m_chunkGlobalIndex]->swap(*col);
            v   = m_chunkVertexPool[m_chunkGlobalIndex];
            col = m_chunkColorPool[m_chunkGlobalIndex];
        } else {
            m_chunkVertexPool.push_back(v);
            m_chunkColorPool.push_back(col);
        }

        CloudChunk chunk;
        chunk.vertices = v;
        chunk.colors = col;
        chunk.renderStart = lg.totalPoints;
        chunk.renderCount = v->size();
        lg.totalPoints += v->size();

        auto* geom = new osg::Geometry;
        geom->setUseDisplayList(false);
        geom->setUseVertexBufferObjects(true);
        geom->setUseVertexArrayObject(true);
        geom->setDataVariance(osg::Object::DYNAMIC);
        geom->setVertexArray(v);
        geom->setColorArray(col, osg::Array::BIND_PER_VERTEX);
        geom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, v->size()));
        geom->setStateSet(m_cloudStateSet);   // 共享着色器状态
        geom->setNodeMask(0);                 // 初始隐藏，渐进上传时逐块显示

        chunk.geom = geom;
        lg.chunks.push_back(chunk);
        m_allChunkGeoms.push_back(geom);
        m_geode->addDrawable(geom);
        m_chunkGlobalIndex++;
    }

    /** @brief 隐藏所有分块几何体 */
    void hideAllChunks() {
        for (auto& chunk : m_allChunkGeoms) {
            chunk->setNodeMask(0);
        }
    }

    /**
     * @brief 应用当前高亮集合到主几何体
     *
     * 选中帧在主几何体中淡化（白色低 alpha，作为高亮几何体的底色），
     * 其余帧半透明。遍历渲染范围而非全量范围，索引经 m_renderIndices
     * 映射回全量点以获取正确的 Z 值。
     */
    void applyHighlight() {
        if (m_lodLevels.empty()) return;
        LodLevelGeoms& lg = m_lodLevels[m_activeLodLevel];
        if (lg.renderRanges.empty()) return;

        const osg::Vec4 white(1.0f, 1.0f, 1.0f, 1.0f);
        // 遍历每个关键帧的渲染顶点范围
        for (const auto& range : lg.renderRanges) {
            bool highlighted = m_highlightIds.count(range.vertexId) > 0;
            // 选中帧：淡化（全量白色由高亮几何体承担）；非选中帧：半透明
            float alpha = highlighted ? m_opacity * 0.35f : m_opacity * 0.5f;
            for (size_t i = range.startVertex;
                 i < range.startVertex + range.vertexCount; ++i) {
                // 定位到所属分块（块间逻辑索引连续）
                const CloudChunk* chunk = chunkForIndex(lg, i);
                if (!chunk) continue;
                size_t local = i - chunk->renderStart;
                size_t fi = lg.renderFullRes ? i : lg.renderIndices[i];
                float wz = static_cast<float>(m_allWorldPoints[fi].z());
                if (highlighted) {
                    (*chunk->colors)[local] = white;
                    (*chunk->colors)[local].a() = alpha;
                } else {
                    (*chunk->colors)[local] = turboColor(wz, m_colorZMin, m_colorZMax);
                    (*chunk->colors)[local].a() = alpha;
                }
            }
        }
        dirtyAllChunkColors(lg);
    }

    /**
     * @brief 重建高亮几何体：选中帧的全量点云（纯白色，独立 VBO）
     *
     * 从全量数据（m_allWorldPoints + m_cloudRanges）提取高亮帧的全部点，
     * 不经过任何 LOD 降采样，因此与主几何体的当前级别无关——LOD 切换时
     * 高亮帧始终以全量白色渲染。空高亮集合时隐藏几何体。
     */
    void rebuildHighlightGeometry() {
        m_highlightVertices->clear();
        m_highlightColors->clear();

        if (m_highlightIds.empty() || m_allWorldPoints.empty()) {
            m_highlightGeom->setNodeMask(0);
            return;
        }

        // 统计选中帧的全量点数并预留
        size_t total = 0;
        for (const auto& range : m_cloudRanges) {
            if (m_highlightIds.count(range.vertexId)) total += range.vertexCount;
        }
        m_highlightVertices->reserve(total);
        m_highlightColors->reserve(total);

        // 提取选中帧全量点，颜色纯白 + 当前透明度
        for (const auto& range : m_cloudRanges) {
            if (!m_highlightIds.count(range.vertexId)) continue;
            for (size_t j = range.startVertex;
                 j < range.startVertex + range.vertexCount; ++j) {
                const Eigen::Vector3d& p = m_allWorldPoints[j];
                m_highlightVertices->push_back(osg::Vec3(
                    static_cast<float>(p.x()),
                    static_cast<float>(p.y()),
                    static_cast<float>(p.z())));
                m_highlightColors->push_back(osg::Vec4(1.0f, 1.0f, 1.0f, m_opacity));
            }
        }

        m_highlightVertices->dirty();
        m_highlightColors->dirty();
        auto* prim = static_cast<osg::DrawArrays*>(
            m_highlightGeom->getPrimitiveSet(0));
        if (prim) prim->setCount(m_highlightVertices->size());
        m_highlightGeom->dirtyBound();
        m_highlightGeom->setNodeMask(~0u);
    }

    /**
     * @brief 根据当前颜色 Z 值范围重新计算所有渲染顶点颜色
     *
     * 不重建顶点数据，仅更新颜色数组（遍历当前级别的所有分块）。
     */
    void recolorAll() {
        if (m_lodLevels.empty()) return;
        LodLevelGeoms& lg = m_lodLevels[m_activeLodLevel];
        if (m_colorZMax - m_colorZMin < 0.001f) return;

        for (auto& chunk : lg.chunks) {
            if (!chunk.vertices || chunk.vertices->empty()) continue;
            for (size_t j = 0; j < chunk.renderCount; ++j) {
                size_t i = chunk.renderStart + j;
                size_t fi = lg.renderFullRes ? i : lg.renderIndices[i];
                float wz = static_cast<float>(m_allWorldPoints[fi].z());
                (*chunk.colors)[j] = turboColor(wz, m_colorZMin, m_colorZMax);
                (*chunk.colors)[j].a() = m_opacity;
            }
            chunk.colors->dirty();
        }
    }

    /** @brief 将某级别的所有块颜色数组标记为上传 */
    void dirtyAllChunkColors(LodLevelGeoms& lg) {
        for (auto& chunk : lg.chunks) {
            if (chunk.colors.valid()) chunk.colors->dirty();
        }
    }

    /**
     * @brief 定位逻辑渲染索引所属的分块（块间连续，线性查找即可）
     */
    static const CloudChunk* chunkForIndex(const LodLevelGeoms& lg, size_t index) {
        for (const auto& chunk : lg.chunks) {
            if (index >= chunk.renderStart &&
                index < chunk.renderStart + chunk.renderCount) {
                return &chunk;
            }
        }
        return nullptr;
    }

    // —— OSG 对象 ——
    osg::ref_ptr<osg::Geode>     m_geode;          ///< 叶节点
    osg::ref_ptr<osg::StateSet>  m_cloudStateSet;  ///< 所有分块共享的着色器状态
    std::vector<osg::ref_ptr<osg::Geometry>> m_allChunkGeoms; ///< 所有分块几何体（供隐藏/移除）
    // —— 分块数组对象池（方案 C：跨构建复用，走 glBufferSubData 增量上传） ——
    std::vector<osg::ref_ptr<osg::Vec3Array>> m_chunkVertexPool;  ///< 池化顶点数组
    std::vector<osg::ref_ptr<osg::Vec4Array>> m_chunkColorPool;   ///< 池化颜色数组
    size_t m_chunkGlobalIndex = 0;  ///< 当前构建的块全局索引（对应池下标）
    // BlendColor 已移除，透明度通过 per-vertex alpha 控制
    osg::ref_ptr<osg::Uniform>    m_pointSizeUniform; ///< 点大小 uniform
    osg::ref_ptr<osg::Uniform>    m_zClipUniform;     ///< Z 轴裁剪开关 uniform
    osg::ref_ptr<osg::Uniform>    m_zRangeUniform;    ///< Z 轴裁剪范围 uniform

    // —— 高亮几何体（选中帧全量白色，不受 LOD 影响） ——
    osg::ref_ptr<osg::Geometry>  m_highlightGeom;         ///< 高亮点云几何体
    osg::ref_ptr<osg::Vec3Array> m_highlightVertices;     ///< 高亮顶点数组（全量）
    osg::ref_ptr<osg::Vec4Array> m_highlightColors;       ///< 高亮颜色数组（纯白）
    osg::ref_ptr<osg::Uniform>   m_highlightPointSizeUniform; ///< 高亮点大小 uniform
    osg::ref_ptr<osg::Uniform>   m_highlightZClipUniform;     ///< 高亮 Z 裁剪开关 uniform
    osg::ref_ptr<osg::Uniform>   m_highlightZRangeUniform;    ///< 高亮 Z 裁剪范围 uniform

    // —— 世界坐标系点数据（全量保留，供着色、高亮、统计使用） ——
    std::vector<Eigen::Vector3d,
                Eigen::aligned_allocator<Eigen::Vector3d>> m_allWorldPoints; ///< 所有世界坐标点

    // —— 每个关键帧的全量云范围 ——
    std::vector<hdl_graph_slam::CloudRange> m_cloudRanges;

    // —— 高亮状态 ——
    std::set<long> m_highlightIds;         ///< 当前高亮的顶点 ID 集合（重建后重新应用）

    // —— LOD 级别数据（分块） ——
    std::vector<LodLevelGeoms> m_lodLevels;  ///< 全部级别（level0 = 主级别）
    int m_activeLodLevel = 0;                ///< 当前激活的 LOD 级别索引
    std::vector<bool> m_levelUploaded;       ///< 各级别是否已完整上传（切回时直接显示）
    int  m_uploadLevel = -1;                 ///< 渐进上传中的级别（-1 = 无）
    size_t m_uploadChunk = 0;                ///< 渐进上传中下一个要显示的块

    // —— 数据范围 ——
    float m_zMin   =  std::numeric_limits<float>::max(); ///< 数据 Z 最小值
    float m_zMax   = -std::numeric_limits<float>::max(); ///< 数据 Z 最大值
    Eigen::Vector3d m_bMin;  ///< 世界包围盒最小值
    Eigen::Vector3d m_bMax;  ///< 世界包围盒最大值
    float m_pointSize = 3.0f;  ///< 点大小（像素）
    float m_opacity   = 1.0f;  ///< 透明度（1.0 不透明）

    // —— Z 轴裁剪状态（着色器级别，变化时无需重建 VBO） ——
    bool  m_zClipping = false;           ///< Z 裁剪开关
    float m_zClipMin  = -10.0f;          ///< Z 裁剪最小值
    float m_zClipMax  = 10.0f;           ///< Z 裁剪最大值
    bool  m_clipRangeInitialized = false; ///< 是否已初始化裁剪范围

    // —— 高程颜色范围（CPU 重新着色） ——
    float m_colorZMin = 0.0f;   ///< 颜色映射 Z 最小值
    float m_colorZMax = 1.0f;   ///< 颜色映射 Z 最大值
    bool  m_useAutoColorRange = true; ///< 是否使用自动颜色范围
    bool  m_colorParamsValid = true;  ///< 各级颜色是否仍与构建时参数一致（false 时切换级别需重着色）
};
