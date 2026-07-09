// ============================================================================
// VertexSphereVisualizer.h
// 顶点球体可视化器
//
// 功能：在世界坐标系中构建球体几何体（无需 MatrixTransform）。
//       调用 appendSphere() 为每个位置添加球体，然后调用 finish() 完成构建。
//       与 EdgeLineVisualizer 使用相同的模式——原始世界坐标系顶点，
//       不使用场景图变换技巧。
//
// 球体使用 Y 轴向上（Y-up）的经纬网格细分方式生成。
// 默认参数（rings=8, sectors=8）在视觉质量和顶点数量间取得平衡。
// ============================================================================

#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/Vec3>
#include <osg/Vec4>
#include <cmath>

#include "visualizers/CoreShaders.h"

/**
 * @brief 顶点球体可视化器
 *
 * 在场景中为每个 SLAM 图顶点绘制一个球体，位置在世界坐标系中。
 * 所有球体合并到单个几何体中以提高渲染效率。
 *
 * 使用模式：
 *   1. clear() —— 清除旧数据
 *   2. appendSphere() —— 逐个添加球体（指定位置和颜色）
 *   3. finish() —— 完成构建并更新 GPU 缓冲区
 *
 * 球体通过经纬网格（纬度环 × 经线段）细分：纬线从北极到南极（phi 从 0 到 PI），
 * 经线沿赤道一周（theta 从 0 到 2*PI）。
 */
class VertexSphereVisualizer {
public:
    /**
     * @brief 构造函数
     *
     * @param radius  球体半径（默认 1.0）
     * @param rings   纬度环数（默认 8，控制垂直细分）
     * @param sectors 经线段数（默认 8，控制水平细分）
     */
    VertexSphereVisualizer(float radius = 1.0f,
                           int rings = 8,
                           int sectors = 8)
        : m_radius(radius), m_rings(rings), m_sectors(sectors) {

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
     * @brief 在世界坐标系中添加一个球体
     *
     * 生成球体的经纬网格顶点，所有顶点偏离中心位置 m_radius 距离。
     * 纬度 phi 从 0（北极）到 PI（南极），经度 theta 从 0 到 2*PI。
     *
     * 每个球体生成 (m_rings+1) * (m_sectors+1) 个顶点和
     * m_rings * m_sectors * 6 个索引（每个四边形两个三角形）。
     *
     * @param center 球心位置（世界坐标）
     * @param color  球体颜色（RGBA，默认深红色）
     */
    void appendSphere(const osg::Vec3d& center,
                      const osg::Vec4& color = osg::Vec4(0.2f, 0.0f, 0.0f, 1.0f)) {
        const float pi = 3.14159265f;
        unsigned int base = m_verts->size();  // 当前已有点数，作为索引基准

        // 生成 Y 轴朝上的球体顶点
        // r: 纬度环索引（0 到 m_rings，每环增加 phi）
        for (int r = 0; r <= m_rings; ++r) {
            float phi    = float(r) * pi / float(m_rings);  // 纬度角 [0, PI]
            float sinPhi = std::sin(phi);
            float cosPhi = std::cos(phi);

            // s: 经线段索引（0 到 m_sectors，每段增加 theta）
            for (int s = 0; s <= m_sectors; ++s) {
                float theta = float(s) * 2.0f * pi / float(m_sectors);  // 经度角 [0, 2*PI]
                float sinT  = std::sin(theta);
                float cosT  = std::cos(theta);

                // 计算顶点位置：球心 + 半径 * 方向向量（Y-up）
                m_verts->push_back(osg::Vec3(
                    center.x() + m_radius * sinPhi * cosT,
                    center.y() + m_radius * cosPhi,
                    center.z() + m_radius * sinPhi * sinT));
                m_colors->push_back(color);
            }
        }

        // 生成索引缓冲：每个四边形划分为两个三角形
        for (int r = 0; r < m_rings; ++r) {
            for (int s = 0; s < m_sectors; ++s) {
                unsigned int a = base + r * (m_sectors + 1) + s;
                unsigned int b = a + m_sectors + 1;
                // 第一个三角形：(a, b, a+1)
                m_indices->push_back(a);
                m_indices->push_back(b);
                m_indices->push_back(a + 1);
                // 第二个三角形：(b, b+1, a+1)
                m_indices->push_back(b);
                m_indices->push_back(b + 1);
                m_indices->push_back(a + 1);
            }
        }
    }

    /** @brief 设置球体半径 */
    void setRadius(float r) { m_radius = r; }

    /** @brief 获取当前球体半径 */
    float radius() const { return m_radius; }

    /**
     * @brief 在所有球体添加完成后调用，完成构建
     *
     * 标记顶点、颜色、索引数据为脏，使 OSG 将它们上传到 GPU。
     */
    void finish() {
        m_verts->dirty();
        m_colors->dirty();
        m_indices->dirty();
        m_geom->dirtyBound();
    }

    /** @brief 获取包含球体的 OSG 节点 */
    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

    /**
     * @brief 清除所有数据（用于重建）
     */
    void clear() {
        m_verts->clear();
        m_colors->clear();
        m_indices->clear();
    }

private:
    float m_radius;  ///< 球体半径
    int m_rings;     ///< 纬度环数
    int m_sectors;   ///< 经线段数

    osg::ref_ptr<osg::Geode> m_geode;              ///< 叶节点
    osg::ref_ptr<osg::Geometry> m_geom;            ///< 几何体
    osg::ref_ptr<osg::Vec3Array> m_verts;          ///< 顶点数组
    osg::ref_ptr<osg::Vec4Array> m_colors;         ///< 颜色数组
    osg::ref_ptr<osg::DrawElementsUInt> m_indices; ///< 索引数组
};
