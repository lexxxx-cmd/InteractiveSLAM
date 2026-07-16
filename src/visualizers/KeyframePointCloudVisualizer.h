// ============================================================================
// KeyframePointCloudVisualizer.h
// 关键帧点云可视化器
//
// 功能：在世界坐标系中构建合并的点云几何体。每个关键帧的局部激光雷达
//       点通过 g2o 位姿在 CPU 上变换到世界坐标系，然后追加到单个 VBO 中。
//       与 EdgeLineVisualizer 和 VertexSphereVisualizer 使用相同的模式。
//
//       记录每个关键帧的顶点范围，使得选中关键帧时可以仅重新着色
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

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

#include <set>
#include <vector>

#include "visualizers/TurboColormap.h"
#include "visualizers/CoreShaders.h"

/**
 * @brief 关键帧点云可视化器
 *
 * 将所有关键帧的点云合并到单个几何体中，通过 CPU 变换将每个点
 * 从关键帧局部坐标系变换到世界坐标系。
 *
 * 核心功能：
 *   1. appendCloud() —— 添加一个关键帧的点云（按位姿变换到世界坐标）
 *   2. finish() —— 完成点云构建，应用 Turbo 颜色映射并上传到 GPU
 *   3. recolorHighlight() —— 高亮选中的关键帧点云（白色）
 *   4. Z 轴裁剪（着色器级别，无需重建 VBO）
 *   5. 高程颜色范围动态调整（CPU 重新着色，无需重建顶点）
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
     * @brief 添加一个关键帧的点云（变换到世界坐标系）
     *
     * 将每个点通过给定的位姿变换到世界坐标系，并更新 Z 值范围。
     *
     * @param cloud    局部坐标系下的点云（const 引用）
     * @param pose     关键帧的 SE3 位姿（世界坐标系）
     * @param vertexId 关键帧对应的顶点 ID
     */
    void appendCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                     const Eigen::Isometry3d& pose,
                     long vertexId) {
        if (!cloud || cloud->empty()) return;

        size_t start = m_allWorldPoints.size();

        // 将每个点通过位姿变换到世界坐标系
        for (const auto& pt : cloud->points) {
            Eigen::Vector3d wp = pose * Eigen::Vector3d(pt.x, pt.y, pt.z);
            float wz = static_cast<float>(wp.z());
            // 更新 Z 值范围（用于 Turbo 颜色映射和裁剪）
            if (wz < m_zMin) m_zMin = wz;
            if (wz > m_zMax) m_zMax = wz;
            m_allWorldPoints.push_back(wp);
        }

        size_t count = m_allWorldPoints.size() - start;
        m_cloudRanges.push_back({start, count, vertexId});
    }

    /**
     * @brief 完成点云构建，上传数据并应用初始 Turbo 颜色映射
     *
     * 在添加完所有关键帧的点云后调用，执行以下操作：
     *   1. 初始化颜色范围（自动模式）
     *   2. 初始化 Z 轴裁剪范围（首次加载）
     *   3. 将顶点和颜色数据上传到 GPU
     *   4. 应用 Turbo 颜色映射（根据 Z 值着色）
     */
    void finish() {
        if (m_allWorldPoints.empty()) return;

        // 防止 Z 值范围过小导致的颜色映射异常
        if (m_zMax - m_zMin < 0.001f) {
            m_zMin -= 0.5f;
            m_zMax += 0.5f;
        }

        // 仅在自动模式下从数据初始化颜色范围
        if (m_useAutoColorRange) {
            m_colorZMin = m_zMin;
            m_colorZMax = m_zMax;
        }

        // 首次加载时从数据初始化裁剪范围
        if (!m_clipRangeInitialized) {
            m_zClipMin = m_zMin;
            m_zClipMax = m_zMax;
            m_clipRangeInitialized = true;
        }

        // 将 Z 轴裁剪范围推送到 GPU
        if (m_zRangeUniform)
            m_zRangeUniform->set(osg::Vec2(m_zClipMin, m_zClipMax));

        // 预分配顶点和颜色数组
        m_vertices->reserve(m_allWorldPoints.size());
        m_colors->reserve(m_allWorldPoints.size());

        // 构建顶点并应用 Turbo 颜色映射（含透明度）
        for (const auto& wp : m_allWorldPoints) {
            m_vertices->push_back(osg::Vec3(
                static_cast<float>(wp.x()),
                static_cast<float>(wp.y()),
                static_cast<float>(wp.z())));
            osg::Vec4 color = turboColor(
                static_cast<float>(wp.z()), m_colorZMin, m_colorZMax);
            color.a() = m_opacity;
            m_colors->push_back(color);
        }

        // 标记数据为脏，使 OSG 重新上传到 GPU
        m_vertices->dirty();
        m_colors->dirty();

        // 更新图元计数
        auto* prim = static_cast<osg::DrawArrays*>(m_geom->getPrimitiveSet(0));
        if (prim) prim->setCount(m_vertices->size());
        m_geom->dirtyBound();
    }

    /**
     * @brief 对指定关键帧集合的点云差异化透明度
     *
     * 在 finish() 之后调用。选中/高亮帧保持当前透明度 m_opacity，
     * 非选中帧透明度变为 m_opacity × 0.5，形成视觉层次。
     * 传入空集合可恢复所有点云为完整透明度。
     *
     * @param highlightIds 需要保持完全可见的顶点 ID 集合。
     *                     传入空集合可恢复所有点云为 m_opacity。
     */
    void recolorHighlight(const std::set<long>& highlightIds) {
        if (!m_colors || m_cloudRanges.empty()) return;

        const osg::Vec4 white(1.0f, 1.0f, 1.0f, 1.0f);
        // 遍历每个关键帧的顶点范围
        for (const auto& range : m_cloudRanges) {
            float alpha = highlightIds.count(range.vertexId)
                              ? m_opacity          // 选中帧：保持当前透明度
                              : m_opacity * 0.5f;  // 非选中帧：半透明度
            for (size_t i = range.startVertex;
                 i < range.startVertex + range.vertexCount; ++i) {
                float wz = static_cast<float>(m_allWorldPoints[i].z());
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
     * @brief 清除所有数据（用于完全重建）
     */
    void clear() {
        m_allWorldPoints.clear();
        m_cloudRanges.clear();
        m_vertices->clear();
        m_colors->clear();
        m_zMin =  std::numeric_limits<float>::max();
        m_zMax = -std::numeric_limits<float>::max();
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

    /** @brief 返回点云中的点数量 */
    int pointCount() const {
        auto* prim = static_cast<osg::DrawArrays*>(m_geom->getPrimitiveSet(0));
        return prim ? prim->getCount() : 0;
    }

private:
    /**
     * @brief 每个关键帧点云的范围结构
     *
     * 用于快速定位每个关键帧对应的顶点范围，避免在重新着色时遍历所有顶点。
     */
    struct CloudRange {
        size_t startVertex;   ///< 该关键帧在全局顶点数组中的起始索引
        size_t vertexCount;   ///< 该关键帧的顶点数量
        long   vertexId;      ///< 关键帧对应的顶点 ID
    };

    /**
     * @brief 根据当前颜色 Z 值范围重新计算所有顶点颜色
     *
     * 不重建顶点数据，仅更新颜色数组。
     */
    void recolorAll() {
        if (!m_colors || m_allWorldPoints.empty()) return;
        if (m_colorZMax - m_colorZMin < 0.001f) return;

        for (size_t i = 0; i < m_allWorldPoints.size(); ++i) {
            float wz = static_cast<float>(m_allWorldPoints[i].z());
            (*m_colors)[i] = turboColor(wz, m_colorZMin, m_colorZMax);
            (*m_colors)[i].a() = m_opacity;
        }
        m_colors->dirty();
    }

    // —— OSG 对象 ——
    osg::ref_ptr<osg::Geode>     m_geode;         ///< 叶节点
    osg::ref_ptr<osg::Geometry>  m_geom;          ///< 点云几何体
    osg::ref_ptr<osg::Vec3Array> m_vertices;      ///< 顶点数组
    osg::ref_ptr<osg::Vec4Array> m_colors;        ///< 颜色数组
    // BlendColor 已移除，透明度通过 per-vertex alpha 控制
    osg::ref_ptr<osg::Uniform>    m_pointSizeUniform; ///< 点大小 uniform
    osg::ref_ptr<osg::Uniform>    m_zClipUniform;     ///< Z 轴裁剪开关 uniform
    osg::ref_ptr<osg::Uniform>    m_zRangeUniform;    ///< Z 轴裁剪范围 uniform

    // —— 世界坐标系点数据（保留以备选择变化时重新着色） ——
    std::vector<Eigen::Vector3d,
                Eigen::aligned_allocator<Eigen::Vector3d>> m_allWorldPoints; ///< 所有世界坐标点

    // —— 每个关键帧的云范围（用于选择性重新着色） ——
    std::vector<CloudRange> m_cloudRanges;

    // —— 数据范围 ——
    float m_zMin   =  std::numeric_limits<float>::max(); ///< 数据 Z 最小值
    float m_zMax   = -std::numeric_limits<float>::max(); ///< 数据 Z 最大值
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
