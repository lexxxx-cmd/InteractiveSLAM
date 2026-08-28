// ============================================================================
// VertexSphereVisualizer.h
// 顶点位姿标记可视化器（保留历史类名）
//
// 功能：在世界坐标系中构建顶点位姿标记（无需 MatrixTransform）。
//       调用 appendCone() / appendArrow() 为每个顶点添加标记，
//       然后调用 finish() 完成构建。
//       与 EdgeLineVisualizer 使用相同的模式——原始世界坐标系顶点，
//       不使用场景图变换技巧。
//
// 标记形状：
//   - 锥体（appendCone）：普通顶点标记，尖端指向关键帧局部 +Z（前方）
//   - 箭头（appendArrow）：高亮顶点标记，圆柱杆 + 圆锥头，同样指向局部 +Z
//
// 方向由关键帧局部位姿（右-下-前坐标系，X右/Y下/Z前）的旋转矩阵决定，
// 生成时把局部坐标系中的偏移量旋转到世界坐标系并平移到顶点位置。
// ============================================================================

#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/Vec3>
#include <osg/Vec4>
#include <cmath>
#include <unordered_map>
#include <cstdint>

#include <Eigen/Geometry>

#include "visualizers/CoreShaders.h"

/**
 * @brief 顶点位姿标记可视化器
 *
 * 在场景中为每个 SLAM 图顶点绘制一个定向标记（锥体或箭头），
 * 位置与方向均来自关键帧的局部位姿。所有标记合并到单个几何体中
 * 以提高渲染效率。
 *
 * 使用模式：
 *   1. clear() —— 清除旧数据
 *   2. appendCone() / appendArrow() —— 逐个添加标记（指定位置、方向和颜色）
 *   3. finish() —— 完成构建并更新 GPU 缓冲区
 */
class VertexSphereVisualizer {
public:
    /**
     * @brief 构造函数
     *
     * @param radius   标记特征尺寸（默认 1.0，作为锥体底面半径 / 箭头总高的一半）
     * @param segments 旋转体圆周分段数（默认 16，控制横向细分）
     */
    explicit VertexSphereVisualizer(float radius = 1.0f,
                                    int segments = 16)
        : m_radius(radius), m_segments(segments) {

        m_geom = new osg::Geometry;
        m_geom->setUseDisplayList(false);
        m_geom->setUseVertexBufferObjects(true);
        m_geom->setUseVertexArrayObject(true);
        m_geom->setDataVariance(osg::Object::DYNAMIC);

        // 顶点、颜色和索引数组
        m_verts  = new osg::Vec3Array;
        m_colors = new osg::Vec4Array;
        m_indices = new osg::DrawElementsUInt(GL_TRIANGLES);

        m_geom->setVertexArray(m_verts);
        m_geom->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);
        m_geom->addPrimitiveSet(m_indices);

        // 应用纯色着色器
        applySimpleColorShader(m_geom->getOrCreateStateSet());

        m_geode = new osg::Geode;
        m_geode->addDrawable(m_geom);
    }

    /**
     * @brief 在世界坐标系中添加一个锥体标记
     *
     * 锥体总高 = 2 * r，底面半径 = r，尖端沿局部 +Z（前方）方向，
     * 中心（几何中点）位于 @p center。锥体在局部坐标系中生成后
     * 通过 @p rot 旋转到世界坐标系。
     *
     * 同时记录 vertexId 对应的颜色数组范围，供 updateSphereColor() 后续
     * 增量更新颜色使用（避免全量几何体重建）。
     *
     * @param center   标记中心位置（世界坐标）
     * @param rot      局部坐标系 → 世界坐标系的旋转（关键帧位姿的旋转部分）
     * @param color    颜色（RGBA，默认深红色）
     * @param vertexId 对应的顶点 ID（用于后续增量颜色更新，默认 -1 不追踪）
     * @param r        特征尺寸（< 0 时使用构造时设置的全局半径 m_radius）
     */
    void appendCone(const osg::Vec3d& center,
                    const Eigen::Matrix3f& rot,
                    const osg::Vec4& color = osg::Vec4(0.2f, 0.0f, 0.0f, 1.0f),
                    long vertexId = -1,
                    float r = -1.0f) {
        float R = (r >= 0.0f) ? r : m_radius;

        beginAppend(color, vertexId);

        float halfH = R;          // 总高 2R，尖端在 +Z 方向
        int n = m_segments;

        // 尖端
        int tipIdx = pushVertex(0.0f, 0.0f, halfH, center, rot);
        // 底面环
        int ringStart = pushRing(R, -halfH, center, rot);
        // 底面圆心
        int baseCenter = pushVertex(0.0f, 0.0f, -halfH, center, rot);

        // 侧面三角形
        for (int i = 0; i < n; ++i) {
            int a = ringStart + i;
            int b = ringStart + (i + 1) % n;
            m_indices->push_back(tipIdx);
            m_indices->push_back(b);
            m_indices->push_back(a);
        }
        // 底面扇形三角化
        for (int i = 0; i < n; ++i) {
            int a = ringStart + i;
            int b = ringStart + (i + 1) % n;
            m_indices->push_back(baseCenter);
            m_indices->push_back(a);
            m_indices->push_back(b);
        }

        endAppend(vertexId);
    }

    /**
     * @brief 在世界坐标系中添加一个箭头标记
     *
     * 箭头总高 = 2 * r，由圆柱杆（后 55%）和圆锥头（前 45%）组成，
     * 尖端沿局部 +Z（前方）方向，几何中点位于 @p center。
     * 用于高亮选中的顶点（替代 2 倍尺寸的球体）。
     *
     * @param center   标记中心位置（世界坐标）
     * @param rot      局部坐标系 → 世界坐标系的旋转（关键帧位姿的旋转部分）
     * @param color    颜色（RGBA）
     * @param vertexId 对应的顶点 ID（默认 -1 不追踪）
     * @param r        特征尺寸（< 0 时使用构造时设置的全局半径 m_radius）
     */
    void appendArrow(const osg::Vec3d& center,
                     const Eigen::Matrix3f& rot,
                     const osg::Vec4& color = osg::Vec4(1.0f, 0.8f, 0.0f, 1.0f),
                     long vertexId = -1,
                     float r = -1.0f) {
        float R = (r >= 0.0f) ? r : m_radius;

        beginAppend(color, vertexId);

        float halfH  = R;              // 总高 2R，尖端在 +Z 方向
        float headLen  = 0.9f * R;     // 圆锥头长度（45%）
        float shaftLen = 2.0f * halfH - headLen;
        float shaftR   = 0.25f * R;    // 杆半径
        int n = m_segments;

        // 杆后环 / 杆前环 / 头部底环 / 尖端
        int shaftBack  = pushRing(shaftR, -halfH, center, rot);
        int shaftFront = pushRing(shaftR, -halfH + shaftLen, center, rot);
        int headBase   = pushRing(R, -halfH + shaftLen, center, rot);
        int tipIdx     = pushVertex(0.0f, 0.0f, halfH, center, rot);
        // 端盖圆心（杆后端、杆前端环面近似用头部底面盖板、锥底）
        int backCenter  = pushVertex(0.0f, 0.0f, -halfH, center, rot);
        int frontCenter = pushVertex(0.0f, 0.0f, -halfH + shaftLen, center, rot);

        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            int sb0 = shaftBack + i,  sb1 = shaftBack + j;
            int sf0 = shaftFront + i, sf1 = shaftFront + j;
            int hb0 = headBase + i,   hb1 = headBase + j;

            // 杆侧面四边形（两个三角形）
            m_indices->push_back(sb0);
            m_indices->push_back(sb1);
            m_indices->push_back(sf0);
            m_indices->push_back(sb1);
            m_indices->push_back(hb1);
            m_indices->push_back(sf0);
            // 头部环形底面（连接杆前环与锥底环的圆环面）
            m_indices->push_back(frontCenter);
            m_indices->push_back(hb0);
            m_indices->push_back(sf0);
            m_indices->push_back(frontCenter);
            m_indices->push_back(sf0);
            m_indices->push_back(hb0);
            // 锥头侧面
            m_indices->push_back(tipIdx);
            m_indices->push_back(hb1);
            m_indices->push_back(hb0);
            // 杆后端盖
            m_indices->push_back(backCenter);
            m_indices->push_back(sb0);
            m_indices->push_back(sb1);
        }

        endAppend(vertexId);
    }

    /**
     * @brief 兼容旧接口：按无旋转（世界轴对齐）方式添加锥体标记
     */
    void appendSphere(const osg::Vec3d& center,
                      const osg::Vec4& color = osg::Vec4(0.2f, 0.0f, 0.0f, 1.0f),
                      long vertexId = -1,
                      float radius = -1.0f) {
        appendCone(center, Eigen::Matrix3f::Identity(), color, vertexId, radius);
    }

    /**
     * @brief 按顶点 ID 更新单个标记的颜色（不重建几何体）
     *
     * 从 m_sphereRanges 中查找该顶点对应的颜色数组范围，仅更新该范围的颜色值。
     * 与 clear() + append*() + finish() 的全量重建相比，此方法
     * 只需更新少量颜色值 + 一次 dirty()，O(1) 复杂度，与总关键帧数无关。
     *
     * @param vertexId 顶点 ID（需已在 append* 中添加过）
     * @param newColor 新颜色（RGBA）
     */
    void updateSphereColor(long vertexId, const osg::Vec4& newColor) {
        auto it = m_sphereRanges.find(vertexId);
        if (it == m_sphereRanges.end()) return;
        const auto& range = it->second;
        for (int i = 0; i < range.vertexCount; ++i)
            (*m_colors)[range.startIndex + i] = newColor;
        m_colors->dirty();
    }

    /** @brief 设置标记特征尺寸 */
    void setRadius(float r) { m_radius = r; }

    /** @brief 获取当前标记特征尺寸 */
    float radius() const { return m_radius; }

    /**
     * @brief 在所有标记添加完成后调用，完成构建
     *
     * 标记顶点、颜色、索引数据为脏，使 OSG 将它们上传到 GPU。
     */
    void finish() {
        m_verts->dirty();
        m_colors->dirty();
        m_indices->dirty();
        m_geom->dirtyBound();
    }

    /** @brief 获取包含标记的 OSG 节点 */
    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

    /**
     * @brief 清除所有数据（用于重建）
     */
    void clear() {
        m_verts->clear();
        m_colors->clear();
        m_indices->clear();
        m_sphereRanges.clear();
    }

private:
    /**
     * @brief 单个标记在颜色数组中的范围
     *
     * 用于 updateSphereColor() 定位需要更新的颜色值范围，
     * 避免遍历所有标记。
     */
    struct SphereRange {
        unsigned int startIndex;  ///< 在 m_colors 中的起始索引
        int vertexCount;          ///< 该标记的顶点数（随形状而变）
    };

    /** @brief append* 公共前置：记录颜色基准并注册追踪范围 */
    void beginAppend(const osg::Vec4& color, long vertexId) {
        m_pendingColor = color;
        m_colorBase = m_colors->size();
        if (vertexId >= 0) {
            m_sphereRanges[vertexId] = {static_cast<unsigned int>(m_colorBase), 0};
        }
    }

    /** @brief append* 公共后置：回填该标记的实际顶点数 */
    void endAppend(long vertexId) {
        if (vertexId >= 0) {
            m_sphereRanges[vertexId].vertexCount =
                static_cast<int>(m_colors->size()) - m_colorBase;
        }
    }

    /** @brief 推入一个局部坐标点经 rot 旋转 + center 平移后的顶点，返回其索引 */
    int pushVertex(float lx, float ly, float lz,
                   const osg::Vec3d& center, const Eigen::Matrix3f& rot) {
        Eigen::Vector3f local(lx, ly, lz);
        Eigen::Vector3f world = rot * local;
        m_verts->push_back(osg::Vec3(
            center.x() + world.x(),
            center.y() + world.y(),
            center.z() + world.z()));
        m_colors->push_back(m_pendingColor);
        return static_cast<int>(m_verts->size()) - 1;
    }

    /** @brief 推入一圈半径 radius、高度 z 的圆周顶点，返回起始索引 */
    int pushRing(float radius, float z,
                 const osg::Vec3d& center, const Eigen::Matrix3f& rot) {
        int start = static_cast<int>(m_verts->size());
        for (int i = 0; i < m_segments; ++i) {
            float theta = float(i) * 2.0f * 3.14159265f / float(m_segments);
            pushVertex(radius * std::cos(theta), radius * std::sin(theta),
                       z, center, rot);
        }
        return start;
    }

    float m_radius;    ///< 标记特征尺寸（锥体底面半径 / 箭头总高一半）
    int m_segments;    ///< 旋转体圆周分段数

    osg::ref_ptr<osg::Geode> m_geode;              ///< 叶节点
    osg::ref_ptr<osg::Geometry> m_geom;            ///< 标记几何体
    osg::ref_ptr<osg::Vec3Array> m_verts;          ///< 标记顶点数组
    osg::ref_ptr<osg::Vec4Array> m_colors;         ///< 标记颜色数组
    osg::ref_ptr<osg::DrawElementsUInt> m_indices; ///< 标记索引数组

    unsigned int m_colorBase = 0;       ///< 当前标记颜色起始索引（append 期间临时使用）
    osg::Vec4 m_pendingColor;           ///< 当前标记颜色（append 期间临时使用）

    /** @brief 顶点 ID → 颜色数组范围的映射（用于增量颜色更新）
     *         由 beginAppend()/endAppend() 在添加标记时填充 */
    std::unordered_map<long, SphereRange> m_sphereRanges;
};
