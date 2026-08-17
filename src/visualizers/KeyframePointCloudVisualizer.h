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
    }

    /**
     * @brief 换入后台构建的点云数据（主线程调用）
     *
     * 将 PointCloudBuilder 构建结果通过 swap 换入本可视化器：
     *   - 全量世界点 / 全量范围 / 渲染索引 / 渲染范围 / 顶点数组
     *   - Z 值范围与包围盒统计
     * 完成后顶点数组标记 dirty，OSG 在下一帧自动重新上传 GPU。
     *
     * @param result 后台构建结果（右值，内容被交换移入）
     */
    void commitBuild(hdl_graph_slam::PointCloudBuildResult&& result) {
        m_allWorldPoints.swap(result.allWorldPoints);
        m_cloudRanges.swap(result.cloudRanges);
        m_renderIndices.swap(result.renderIndices);
        m_renderRanges.swap(result.renderRanges);
        m_renderFullRes = result.renderFullRes;
        m_zMin = result.zMin;
        m_zMax = result.zMax;
        m_bMin = result.bMin;
        m_bMax = result.bMax;

        // 换入渲染顶点数组；颜色数组由 recolorAll 按顶点数生成，
        // 因此这里只需将颜色数组大小与顶点数对齐（内容在下方 recolorAll 填充）
        if (m_vertices && result.vertices) {
            m_vertices->swap(*result.vertices);
            m_colors->resize(m_vertices->size());
        } else {
            // 防御：结果为空时清空旧数据，避免与新全量点状态不一致
            m_vertices->clear();
            m_colors->clear();
        }

        // 首次构建时从数据初始化裁剪范围
        if (!m_clipRangeInitialized && m_zMax > m_zMin) {
            m_zClipMin = m_zMin;
            m_zClipMax = m_zMax;
            m_clipRangeInitialized = true;
        }
        if (m_zRangeUniform)
            m_zRangeUniform->set(osg::Vec2(m_zClipMin, m_zClipMax));

        // 标记数据为脏，使 OSG 重新上传到 GPU
        m_vertices->dirty();
        m_colors->dirty();

        // 更新图元计数
        auto* prim = static_cast<osg::DrawArrays*>(m_geom->getPrimitiveSet(0));
        if (prim) prim->setCount(m_vertices->size());
        m_geom->dirtyBound();

        // 同步颜色范围（自动模式下用数据范围）并生成默认着色
        if (m_useAutoColorRange) {
            m_colorZMin = m_zMin;
            m_colorZMax = m_zMax;
        }
        recolorAll();
    }

    /** @brief 裁剪范围是否已初始化（首次构建后为 true） */
    bool isClipRangeInitialized() const { return m_clipRangeInitialized; }

    /**
     * @brief 清除高亮状态并恢复默认着色
     *
     * 用于无选中/无播放高亮时恢复所有点云为 Turbo 高程着色 + 当前透明度。
     */
    void clearHighlight() {
        m_highlightIds.clear();
        recolorAll();
    }

    /**
     * @brief 对指定关键帧集合的点云差异化透明度
     *
     * 在 commitBuild() 之后调用。选中/高亮帧保持当前透明度 m_opacity，
     * 非选中帧透明度变为 m_opacity × 0.5，形成视觉层次。
     * 传入空集合时所有帧降为半透明（"暗淡"效果，恢复完整透明度
     * 请调用 setOpacity）。
     *
     * @param highlightIds 需要保持完全可见的顶点 ID 集合。
     */
    void recolorHighlight(const std::set<long>& highlightIds) {
        m_highlightIds = highlightIds;
        applyHighlight();
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
        m_vertices->clear();
        m_colors->clear();
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

    /** @brief 设置点的大小（像素单位） */
    void setPointSize(float size) {
        m_pointSize = size;
        if (m_pointSizeUniform)
            m_pointSizeUniform->set(size);
    }

    /** @brief 设置点云透明度 */
    void setOpacity(float opacity) {
        m_opacity = opacity;
        // 透明度变化需更新所有顶点的 alpha 通道
        recolorAll();
    }

    // ========================================================================
    // Z 轴裁剪控制（着色器级别，无需重建 VBO）
    // ========================================================================

    /** @brief 启用/禁用 Z 轴裁剪 */
    void setZClipping(bool enabled) {
        m_zClipping = enabled;
        if (m_zClipUniform)
            m_zClipUniform->set(enabled ? 1 : 0);
    }

    /** @brief 设置 Z 轴裁剪范围 */
    void setZClipRange(float minZ, float maxZ) {
        m_zClipMin = minZ;
        m_zClipMax = maxZ;
        if (m_zRangeUniform)
            m_zRangeUniform->set(osg::Vec2(minZ, maxZ));
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

    /** @brief 当前实际渲染的点数（降采样后） */
    int renderPointCount() const {
        return static_cast<int>(m_vertices->size());
    }

    /** @brief 全量点云总点数（降采样前） */
    size_t totalPointCount() const { return m_allWorldPoints.size(); }

    /** @brief 返回点云中的渲染点数（降采样后） */
    int pointCount() const {
        auto* prim = static_cast<osg::DrawArrays*>(m_geom->getPrimitiveSet(0));
        return prim ? prim->getCount() : 0;
    }

private:
    /**
     * @brief 应用当前高亮集合（选中帧白色，其余帧半透明）
     *
     * 遍历渲染范围而非全量范围，索引经 m_renderIndices 映射回全量点
     * 以获取正确的 Z 值。降采样后逐帧范围仍保持完整。
     */
    void applyHighlight() {
        if (!m_colors || m_renderRanges.empty()) return;

        const osg::Vec4 white(1.0f, 1.0f, 1.0f, 1.0f);
        // 遍历每个关键帧的渲染顶点范围
        for (const auto& range : m_renderRanges) {
            float alpha = m_highlightIds.count(range.vertexId)
                              ? m_opacity          // 选中帧：保持当前透明度
                              : m_opacity * 0.5f;  // 非选中帧：半透明度
            for (size_t i = range.startVertex;
                 i < range.startVertex + range.vertexCount; ++i) {
                size_t fi = m_renderFullRes ? i : m_renderIndices[i];
                float wz = static_cast<float>(m_allWorldPoints[fi].z());
                if (alpha == m_opacity) (*m_colors)[i] = white;
                else {
                    (*m_colors)[i] = turboColor(wz, m_colorZMin, m_colorZMax);
                    (*m_colors)[i].a() = alpha;
                }
            }
        }
        m_colors->dirty();
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
    osg::ref_ptr<osg::Geometry>  m_geom;          ///< 点云几何体
    osg::ref_ptr<osg::Vec3Array> m_vertices;      ///< 渲染顶点数组（降采样后）
    osg::ref_ptr<osg::Vec4Array> m_colors;        ///< 渲染颜色数组（降采样后）
    // BlendColor 已移除，透明度通过 per-vertex alpha 控制
    osg::ref_ptr<osg::Uniform>    m_pointSizeUniform; ///< 点大小 uniform
    osg::ref_ptr<osg::Uniform>    m_zClipUniform;     ///< Z 轴裁剪开关 uniform
    osg::ref_ptr<osg::Uniform>    m_zRangeUniform;    ///< Z 轴裁剪范围 uniform

    // —— 世界坐标系点数据（全量保留，供着色、高亮、统计使用） ——
    std::vector<Eigen::Vector3d,
                Eigen::aligned_allocator<Eigen::Vector3d>> m_allWorldPoints; ///< 所有世界坐标点

    // —— 每个关键帧的全量云范围 ——
    std::vector<hdl_graph_slam::CloudRange> m_cloudRanges;

    // —— 降采样渲染状态 ——
    std::vector<size_t> m_renderIndices;   ///< 渲染索引 -> 全量索引（仅降采样时使用）
    std::vector<hdl_graph_slam::CloudRange> m_renderRanges; ///< 每个关键帧在渲染数组中的范围
    bool m_renderFullRes = true;           ///< true = 渲染索引与全量索引一致（不降采样）
    std::set<long> m_highlightIds;         ///< 当前高亮的顶点 ID 集合（重建后重新应用）

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
