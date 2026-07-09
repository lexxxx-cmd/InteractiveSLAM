// ============================================================================
// CoordinateAxesVisualizer.h
// 坐标轴可视化器
//
// 功能：在世界坐标系原点绘制红绿蓝三色坐标轴。
//       - X 轴：红色 (1,0,0)
//       - Y 轴：绿色 (0,1,0)
//       - Z 轴：蓝色 (0,0,1)
//       每个轴从原点 (0,0,0) 延伸到单位长度 (1,0,0) 等。
//
// 使用 GLSL 330 Core Profile 着色器，避免 OSG 固定管线
// 在 Core Profile 上下文中产生的 OpenGL 错误。
// ============================================================================

#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include "visualizers/CoreShaders.h"

/**
 * @brief 简易 RGB 三色坐标轴可视化器
 *
 * 在场景原点绘制三条轴线，用于指示场景坐标系的方向。
 * 每条轴线由一条线段组成，从原点延伸到对应轴的单位位置。
 *
 * 使用 GLSL 330 Core Profile 着色器（而非 OSG 固定功能管线），
 * 以确保在 Core Profile OpenGL 上下文中的兼容性。
 */
class CoordinateAxesVisualizer {
public:
    /** @brief 构造函数：创建三条颜色轴线几何体 */
    CoordinateAxesVisualizer() {
        // 创建几何体对象，配置顶点缓冲对象（VBO）和顶点数组对象（VAO）
        // Core Profile 3.3 要求显式使用 VAO
        auto* geom = new osg::Geometry;
        geom->setUseDisplayList(false);          // 使用 VBO 而非显示列表
        geom->setUseVertexBufferObjects(true);   // 启用顶点缓冲对象
        geom->setUseVertexArrayObject(true);     // 启用顶点数组对象（Core Profile 3.3 必需）

        // 顶点数组和颜色数组
        auto* verts = new osg::Vec3Array;
        auto* colors = new osg::Vec4Array;

        // X 轴（红色）：从 (0,0,0) 到 (1,0,0)
        verts->push_back(osg::Vec3(0, 0, 0));
        verts->push_back(osg::Vec3(1, 0, 0));
        colors->push_back(osg::Vec4(1, 0, 0, 1));
        colors->push_back(osg::Vec4(1, 0, 0, 1));

        // Y 轴（绿色）：从 (0,0,0) 到 (0,1,0)
        verts->push_back(osg::Vec3(0, 0, 0));
        verts->push_back(osg::Vec3(0, 1, 0));
        colors->push_back(osg::Vec4(0, 1, 0, 1));
        colors->push_back(osg::Vec4(0, 1, 0, 1));

        // Z 轴（蓝色）：从 (0,0,0) 到 (0,0,1)
        verts->push_back(osg::Vec3(0, 0, 0));
        verts->push_back(osg::Vec3(0, 0, 1));
        colors->push_back(osg::Vec4(0, 0, 1, 1));
        colors->push_back(osg::Vec4(0, 0, 1, 1));

        // 设置几何体的顶点和颜色数组，添加基本图元（线段）
        geom->setVertexArray(verts);
        geom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
        geom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, verts->size()));

        // 应用自定义 GLSL 330 着色器，防止 OSG 设置已废弃的
        // Material/LightModel/AlphaFunc 等 uniform 变量
        applySimpleColorShader(geom->getOrCreateStateSet());

        // 将几何体添加到叶节点（Geode）中
        m_geode = new osg::Geode;
        m_geode->addDrawable(geom);
    }

    /** @brief 获取包含坐标轴的 OSG 节点 */
    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

private:
    osg::ref_ptr<osg::Geode> m_geode;  ///< 坐标轴几何体所在的叶节点
};
