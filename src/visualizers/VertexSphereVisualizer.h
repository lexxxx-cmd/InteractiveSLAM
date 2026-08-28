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
//   - 尖锥体（appendCone）：高亮顶点标记，尖端指向关键帧局部 +Z（前方）
//   - 截锥体（appendTruncatedCone）：普通顶点标记，切掉尖顶的圆台，同样指向局部 +Z
//
// 方向由关键帧局部位姿（右-下-前坐标系，X右/Y下/Z前）的旋转矩阵决定，
// 生成时把局部坐标系中的偏移量旋转到世界坐标系并平移到顶点位置。
// ============================================================================

#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/BlendFunc>
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
 *   2. appendCone() / appendTruncatedCone() —— 逐个添加标记（指定位置、方向和颜色）
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

        // 启用 alpha 混合（顶点透明度调整）：OSG 会把开启 GL_BLEND 的
        // StateSet 归入透明渲染队列，在不透明几何体之后绘制
        auto* ss = m_geom->getOrCreateStateSet();
        ss->setMode(GL_BLEND, osg::StateAttribute::ON);
        ss->setAttributeAndModes(
            new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON);
        // 背面剔除：透明混合下若同时渲染内壁（背面）与正面，
        // 同一像素会被混合两次形成深色杂斑；凸体每个像素只有一层
        // 正面，剔除背面后混合结果干净。要求所有三角形按
        // "从外部看逆时针"（外法线）的环绕方向生成。
        ss->setMode(GL_CULL_FACE, osg::StateAttribute::ON);

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

        // 侧面三角形（外法线：从外部看逆时针）
        for (int i = 0; i < n; ++i) {
            int a = ringStart + i;
            int b = ringStart + (i + 1) % n;
            m_indices->push_back(tipIdx);
            m_indices->push_back(a);
            m_indices->push_back(b);
        }
        // 底面扇形三角化（外法线朝 -Z：从外部看逆时针）
        for (int i = 0; i < n; ++i) {
            int a = ringStart + i;
            int b = ringStart + (i + 1) % n;
            m_indices->push_back(baseCenter);
            m_indices->push_back(b);
            m_indices->push_back(a);
        }

        endAppend(vertexId);
    }

    /**
     * @brief 在世界坐标系中添加一个截锥体（圆台）标记
     *
     * 总高 = 2 * r，底面半径 = r，顶面半径 = topRatio * r（切掉尖锥的尖端），
     * 轴向沿局部 +Z（前方），几何中点位于 @p center。
     * 用于普通顶点标记（与高亮的尖锥体形成形状区分）。
     *
     * @param center   标记中心位置（世界坐标）
     * @param rot      局部坐标系 → 世界坐标系的旋转（关键帧位姿的旋转部分）
     * @param color    颜色（RGBA，默认深红色）
     * @param vertexId 对应的顶点 ID（用于后续增量颜色更新，默认 -1 不追踪）
     * @param r        特征尺寸（< 0 时使用构造时设置的全局半径 m_radius）
     * @param topRatio 顶面半径与底面半径之比（默认 0.35）
     */
    void appendTruncatedCone(const osg::Vec3d& center,
                             const Eigen::Matrix3f& rot,
                             const osg::Vec4& color = osg::Vec4(0.2f, 0.0f, 0.0f, 1.0f),
                             long vertexId = -1,
                             float r = -1.0f,
                             float topRatio = 0.35f) {
        float R = (r >= 0.0f) ? r : m_radius;

        beginAppend(color, vertexId);

        float halfH = R;          // 总高 2R，截锥轴向沿 +Z
        float topR  = topRatio * R;
        int n = m_segments;

        // 顶面环 / 底面环 / 两个端面圆心
        int topStart  = pushRing(topR, +halfH, center, rot);
        int baseStart = pushRing(R, -halfH, center, rot);
        int topCenter  = pushVertex(0.0f, 0.0f, +halfH, center, rot);
        int baseCenter = pushVertex(0.0f, 0.0f, -halfH, center, rot);

        for (int i = 0; i < n; ++i) {
            int j = (i + 1) % n;
            int t0 = topStart + i,  t1 = topStart + j;
            int b0 = baseStart + i, b1 = baseStart + j;

            // 侧面四边形（两个三角形，外法线朝外）
            m_indices->push_back(b0);
            m_indices->push_back(b1);
            m_indices->push_back(t0);
            m_indices->push_back(b1);
            m_indices->push_back(t1);
            m_indices->push_back(t0);
            // 顶面扇形（外法线朝 +Z）
            m_indices->push_back(topCenter);
            m_indices->push_back(t0);
            m_indices->push_back(t1);
            // 底面扇形（外法线朝 -Z）
            m_indices->push_back(baseCenter);
            m_indices->push_back(b1);
            m_indices->push_back(b0);
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
        osg::Vec4 color = newColor;
        color.a() = m_opacity;  // 颜色更新不破坏整体不透明度
        for (int i = 0; i < range.vertexCount; ++i)
            (*m_colors)[range.startIndex + i] = color;
        m_colors->dirty();
    }

    /** @brief 设置标记特征尺寸 */
    void setRadius(float r) { m_radius = r; }

    /** @brief 获取当前标记特征尺寸 */
    float radius() const { return m_radius; }

    /**
     * @brief 设置顶点标记的整体不透明度
     *
     * 立即重写当前颜色数组中所有顶点的 alpha 值；后续 append*()
     * 新增的标记也会使用该不透明度。
     *
     * @param opacity 不透明度（0.0 全透明 ~ 1.0 不透明）
     */
    void setOpacity(float opacity) {
        m_opacity = opacity;
        for (unsigned int i = 0; i < m_colors->size(); ++i)
            (*m_colors)[i].a() = opacity;
        m_colors->dirty();
    }

    /** @brief 获取当前不透明度 */
    float opacity() const { return m_opacity; }

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
        m_pendingColor.a() *= m_opacity;  // 应用全局不透明度
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
    float m_opacity = 1.0f;  ///< 标记整体不透明度（1.0 不透明）

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
