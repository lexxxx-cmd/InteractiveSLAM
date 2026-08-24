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
// 着色方案：默认使用 Turbo 颜色映射根据高程（Z 值）着色。
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
        m_geom = new osg::Geometry;
        m_geom->setUseDisplayList(false);
        m_geom->setUseVertexBufferObjects(true);
        m_geom->setUseVertexArrayObject(true);
        m_geom->setDataVariance(osg::Object::DYNAMIC);

        // 顶点和颜色数组
        m_vertices = new osg::Vec3Array;
        m_colors   = new osg::Vec4Array;

        m_geom->setVertexArray(m_vertices);
        m_geom->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);
        m_geom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, 0));

        // 设置点云着色器
        auto* ss = m_geom->getOrCreateStateSet();
        m_pointSizeUniform = applyPointCloudShader(ss, m_pointSize);

        // 查找 Z 轴裁剪 uniform（由 applyPointCloudShader 添加）
        m_zClipUniform  = ss->getUniform("z_clipping");
        m_zRangeUniform = ss->getUniform("z_range");

        // 透明度控制（使用 per-vertex alpha，支持逐帧差异化透明度）
        ss->setAttributeAndModes(
            new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON);

        m_geode = new osg::Geode;
        m_geode->addDrawable(m_geom);

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

    /**
     * @brief 换入后台构建的点云数据（主线程调用）
     *
     * 将 PointCloudBuilder 构建结果换入本可视化器：
     *   - 全量世界点 / 全量范围 / Z 值范围与包围盒统计
     *   - 主级别渲染数据 + 多级 LOD（全量模式生成）
     * 当前渲染数组（m_vertices/m_colors）换绑到 level 0，
     * 颜色由 recolorAll 按顶点数生成。之后可经 setLodLevel() 换绑到
     * 其他级别（相机距离驱动）。
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

        // 同步颜色范围（自动模式下用数据范围）
        if (m_useAutoColorRange) {
            m_colorZMin = m_zMin;
            m_colorZMax = m_zMax;
        }

        // 构建 LOD 级别列表：level 0 = 主级别，其后为 builder 生成的远级别
        m_lodLevels.clear();
        m_lodLevels.reserve(1 + result.lodLevels.size());
        {
            hdl_graph_slam::LodLevel l0;
            l0.vertices = result.vertices;
            if (!l0.vertices.valid()) {
                l0.vertices = new osg::Vec3Array;  // 防御：空结果也提供空数组
            }
            l0.colors = new osg::Vec4Array;
            l0.renderIndices.swap(result.renderIndices);
            l0.renderRanges.swap(result.renderRanges);
            l0.renderFullRes = result.renderFullRes;
            m_lodLevels.push_back(std::move(l0));
        }
        for (auto& lod : result.lodLevels) {
            lod.colors = new osg::Vec4Array;
            m_lodLevels.push_back(std::move(lod));
        }
        m_activeLodLevel = 0;

        // 换绑当前渲染数组到 level 0（ref_ptr 指向同一数组，零拷贝）
        bindLevel(0);

        // 全量数据已更新：若存在高亮，重建高亮几何体（全量白色）并重着色主几何体
        if (!m_highlightIds.empty()) {
            applyHighlight();
            rebuildHighlightGeometry();
        }
    }

    /**
     * @brief 按相机距离切换到指定 LOD 级别（主线程调用）
     *
     * 将渲染顶点/颜色数组换绑到目标级别（ref_ptr 指向级别数据，零拷贝），
     * 拷贝该级渲染索引/范围，然后重新着色并标记上传。
     * 级别 0 = 主级别（全量），级别越远点数越少。
     * 高亮几何体使用全量数据，不受级别切换影响，无需重建。
     *
     * @param level 目标级别（0 .. lodLevelCount()-1）
     */
    void setLodLevel(int level) {
        if (level == m_activeLodLevel) return;
        if (level < 0 || (size_t)level >= m_lodLevels.size()) return;
        bindLevel(level);
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
        m_renderIndices.clear();
        m_renderRanges.clear();
        m_renderFullRes = true;
        m_highlightIds.clear();
        m_lodLevels.clear();
        m_activeLodLevel = 0;
        m_vertices->clear();
        m_colors->clear();
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
        m_opacity = opacity;
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
        m_colorZMin = minZ;
        m_colorZMax = maxZ;
        m_useAutoColorRange = false;
        recolorAll();
    }

    /** @brief 设置是否自动计算颜色范围 */
    void setAutoColorRange(bool autoRange) {
        m_useAutoColorRange = autoRange;
        if (autoRange) {
            m_colorZMin = m_zMin;
            m_colorZMax = m_zMax;
        }
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
        auto* prim = static_cast<osg::DrawArrays*>(m_geom->getPrimitiveSet(0));
        return prim ? prim->getCount() : 0;
    }

private:
    /**
     * @brief 将当前渲染数组换绑到指定级别（ref_ptr 零拷贝）
     *
     * 目标级别的顶点/颜色数组直接成为当前渲染数组（geometry 重新绑定），
     * 拷贝该级渲染索引/范围，重设颜色数组大小，重新着色并标记上传。
     *
     * @param level 目标级别索引（调用方需保证合法）
     */
    void bindLevel(int level) {
        const hdl_graph_slam::LodLevel& lod = m_lodLevels[level];
        m_vertices = lod.vertices;
        m_colors = lod.colors;
        m_renderIndices = lod.renderIndices;
        m_renderRanges = lod.renderRanges;
        m_renderFullRes = lod.renderFullRes;
        m_activeLodLevel = level;

        m_geom->setVertexArray(m_vertices);
        m_geom->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);

        // 颜色数组与顶点数对齐，重新着色
        m_colors->resize(m_vertices->size());
        m_vertices->dirty();
        m_colors->dirty();
        auto* prim = static_cast<osg::DrawArrays*>(m_geom->getPrimitiveSet(0));
        if (prim) prim->setCount(m_vertices->size());
        m_geom->dirtyBound();

        recolorAll();
    }

    /**
     * @brief 应用当前高亮集合到主几何体
     *
     * 选中帧在主几何体中淡化（白色低 alpha，作为高亮几何体的底色），
     * 其余帧半透明。遍历渲染范围而非全量范围，索引经 m_renderIndices
     * 映射回全量点以获取正确的 Z 值。
     */
    void applyHighlight() {
        if (!m_colors || m_renderRanges.empty()) return;

        const osg::Vec4 white(1.0f, 1.0f, 1.0f, 1.0f);
        // 遍历每个关键帧的渲染顶点范围
        for (const auto& range : m_renderRanges) {
            bool highlighted = m_highlightIds.count(range.vertexId) > 0;
            // 选中帧：淡化（全量白色由高亮几何体承担）；非选中帧：半透明
            float alpha = highlighted ? m_opacity * 0.35f : m_opacity * 0.5f;
            for (size_t i = range.startVertex;
                 i < range.startVertex + range.vertexCount; ++i) {
                size_t fi = m_renderFullRes ? i : m_renderIndices[i];
                float wz = static_cast<float>(m_allWorldPoints[fi].z());
                if (highlighted) {
                    (*m_colors)[i] = white;
                    (*m_colors)[i].a() = alpha;
                } else {
                    (*m_colors)[i] = turboColor(wz, m_colorZMin, m_colorZMax);
                    (*m_colors)[i].a() = alpha;
                }
            }
        }
        m_colors->dirty();
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
     * 不重建顶点数据，仅更新颜色数组。
     */
    void recolorAll() {
        if (!m_colors || m_vertices->empty()) return;
        if (m_colorZMax - m_colorZMin < 0.001f) return;

        for (size_t i = 0; i < m_vertices->size(); ++i) {
            size_t fi = m_renderFullRes ? i : m_renderIndices[i];
            float wz = static_cast<float>(m_allWorldPoints[fi].z());
            (*m_colors)[i] = turboColor(wz, m_colorZMin, m_colorZMax);
            (*m_colors)[i].a() = m_opacity;
        }
        m_colors->dirty();
    }

    // —— OSG 对象 ——
    osg::ref_ptr<osg::Geode>     m_geode;         ///< 叶节点
    osg::ref_ptr<osg::Geometry>  m_geom;          ///< 点云几何体（LOD 主几何体）
    osg::ref_ptr<osg::Vec3Array> m_vertices;      ///< 渲染顶点数组（当前 LOD 级别）
    osg::ref_ptr<osg::Vec4Array> m_colors;        ///< 渲染颜色数组（当前 LOD 级别）
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

    // —— 降采样渲染状态 ——
    std::vector<size_t> m_renderIndices;   ///< 渲染索引 -> 全量索引（当前激活级别）
    std::vector<hdl_graph_slam::CloudRange> m_renderRanges; ///< 每个关键帧在渲染数组中的范围
    bool m_renderFullRes = true;           ///< true = 渲染索引与全量索引一致（当前激活级别）
    std::set<long> m_highlightIds;         ///< 当前高亮的顶点 ID 集合（重建后重新应用）

    // —— LOD 级别数据 ——
    std::vector<hdl_graph_slam::LodLevel> m_lodLevels; ///< 全部级别（level0 = 主级别）
    int m_activeLodLevel = 0;              ///< 当前激活的 LOD 级别索引

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
};
