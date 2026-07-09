// ============================================================================
// EdgeLineVisualizer.h
// 边线可视化器
//
// 功能：将 SLAM 图中所有的 SE3 边渲染为一系列线段。
//       将所有边合并为一个统一的线段几何体，以提高渲染效率。
//       根据边的来源（原始、手动回环、自动回环、锚点）使用不同颜色。
//
// 使用 GLSL 330 Core Profile 着色器以确保兼容性。
// 支持运行时动态重建（当图数据或边的隐藏状态变化时）。
// ============================================================================

#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/LineWidth>
#include <vector>
#include <string>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/hyper_graph.h>

#include <set>

#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "data/g2o/robust_kernel_io.hpp"

#include "visualizers/CoreShaders.h"

/**
 * @brief 边线段数据结构，用于鼠标拾取和右键上下文菜单信息展示
 *
 * 不仅存储线段的端点坐标，还包含边的元数据（ID、连接顶点、长度、
 * 信息矩阵迹、鲁棒核函数类型等），方便交互式操作中的信息查询。
 */
struct EdgeSegment {
    osg::Vec3d p1, p2;      ///< 线段端点（世界坐标）
    long id;                 ///< g2o 边的 ID
    long v1_id, v2_id;      ///< 连接的两个顶点 ID
    double distance;         ///< 边的长度（米）
    double info_diag;        ///< 信息矩阵的迹（置信度指标，值越大表示约束越强）
    std::string kernel;      ///< 鲁棒核函数类型（"NONE"、"Huber" 等）
};

/**
 * @brief 边线可视化器
 *
 * 将所有 SE3 边合并为单个线段几何体，每条线段的颜色取决于其来源：
 *   - 原始边（灰色半透明）：原始 SLAM 过程中的边
 *   - 手动回环边（绿色）：用户手动添加的回环约束
 *   - 自动回环边（青色）：自动检测的回环约束
 *   - 锚点边（黄色）：锚点/地面控制点约束
 *
 * 支持通过 setLineWidth() 动态调整线宽。
 * 支持隐藏指定边（通过 hiddenEdgeIds 集合，来自 UI 面板）。
 */
class EdgeLineVisualizer {
public:
    /** @brief 构造函数：初始化线段几何体，设置 GLSL 着色器和线宽属性 */
    EdgeLineVisualizer() {
        // 创建几何体并配置为动态数据（因为需要频繁重建）
        m_geom = new osg::Geometry;
        m_geom->setUseDisplayList(false);
        m_geom->setUseVertexBufferObjects(true);
        m_geom->setUseVertexArrayObject(true);  // Core Profile 3.3 必需
        m_geom->setDataVariance(osg::Object::DYNAMIC);  // 数据会动态变化

        // 顶点和颜色数组
        m_vertices = new osg::Vec3Array;
        m_colors = new osg::Vec4Array;

        m_geom->setVertexArray(m_vertices);
        m_geom->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);
        m_geom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 0));

        // 应用自定义 GLSL 330 着色器
        auto* ss = m_geom->getOrCreateStateSet();
        applySimpleColorShader(ss);

        // 线宽状态属性
        m_lineWidth = new osg::LineWidth(m_width);
        ss->setAttributeAndModes(m_lineWidth, osg::StateAttribute::ON);

        // 创建叶节点并添加几何体
        m_geode = new osg::Geode;
        m_geode->addDrawable(m_geom);
    }

    /**
     * @brief 从 InteractiveGraph 重建所有边线
     *
     * 清空当前线段数据，遍历图中的所有 SE3 边，根据边的来源着色，
     * 并收集每条边的元数据用于拾取交互。
     *
     * @param graph         交互式图数据指针
     * @param hiddenEdgeIds 用户通过 EdgeListPanel 隐藏的边 ID 集合
     */
    void rebuild(hdl_graph_slam::InteractiveGraph* graph,
                 const std::set<long>& hiddenEdgeIds = {}) {
        m_vertices->clear();
        m_colors->clear();
        m_edgeSegments.clear();

        if (!graph) return;

        // 根据边的来源定义颜色方案
        static const osg::Vec4 kColorOriginal   (0.70f, 0.75f, 0.80f, 0.55f);  // 暗淡灰色 —— 原始边
        static const osg::Vec4 kColorManualLoop (0.00f, 0.90f, 0.20f, 0.85f);  // 绿色     —— 手动回环
        static const osg::Vec4 kColorAutoLoop   (0.00f, 0.75f, 1.00f, 0.85f);  // 青色     —— 自动回环
        static const osg::Vec4 kColorAnchor     (1.00f, 0.85f, 0.00f, 0.85f);  // 黄色     —— 锚点边

        auto* g2oGraph = dynamic_cast<g2o::SparseOptimizer*>(graph->graph.get());
        if (!g2oGraph) return;

        // 遍历图中所有边
        for (auto* edge : g2oGraph->edges()) {
            // 只处理 SE3 类型的边
            auto* se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
            if (!se3) continue;

            long eid = static_cast<long>(se3->id());

            // 跳过用户隐藏的边
            if (hiddenEdgeIds.count(eid)) continue;

            // 获取边的两个端点顶点
            auto* v1 = dynamic_cast<g2o::VertexSE3*>(se3->vertices()[0]);
            auto* v2 = dynamic_cast<g2o::VertexSE3*>(se3->vertices()[1]);
            if (!v1 || !v2) continue;

            // 获取顶点位置估计值
            Eigen::Vector3d p1 = v1->estimate().translation();
            Eigen::Vector3d p2 = v2->estimate().translation();

            // 根据边的来源选择颜色
            osg::Vec4 color = kColorOriginal;
            switch (graph->edge_source(eid)) {
                case hdl_graph_slam::EdgeSource::ManualLoop: color = kColorManualLoop; break;
                case hdl_graph_slam::EdgeSource::AutoLoop:   color = kColorAutoLoop;   break;
                case hdl_graph_slam::EdgeSource::Anchor:     color = kColorAnchor;     break;
                default: break;
            }

            // 添加线段的两个端点和对应的颜色
            m_vertices->push_back(osg::Vec3(p1.x(), p1.y(), p1.z()));
            m_vertices->push_back(osg::Vec3(p2.x(), p2.y(), p2.z()));
            m_colors->push_back(color);
            m_colors->push_back(color);

            // 收集边的元数据，用于鼠标拾取和右键菜单展示
            double dist = (p1 - p2).norm();                      // 边的空间距离
            double trace = se3->information().trace();            // 信息矩阵的迹（约束强度）
            std::string kernelName = g2o::kernel_type(se3->robustKernel());
            if (kernelName.empty()) kernelName = "NONE";          // 无鲁棒核函数
            m_edgeSegments.push_back({
                osg::Vec3d(p1.x(), p1.y(), p1.z()),
                osg::Vec3d(p2.x(), p2.y(), p2.z()),
                eid, v1->id(), v2->id(),
                dist, trace, kernelName
            });
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
     * @brief 获取边线段列表（用于右键点击拾取，通过点到线段的距离判断）
     * @return 边线段数据向量
     */
    const std::vector<EdgeSegment>& edgeSegments() const {
        return m_edgeSegments;
    }

    /**
     * @brief 设置线宽
     * @param w 线宽（像素单位）
     */
    void setLineWidth(float w) {
        m_width = w;
        if (m_lineWidth) m_lineWidth->setWidth(w);
    }

    /** @brief 获取包含边线的 OSG 节点 */
    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

private:
    osg::ref_ptr<osg::Geode> m_geode;       ///< 边线所在的叶节点
    osg::ref_ptr<osg::Geometry> m_geom;     ///< 线段几何体
    osg::ref_ptr<osg::Vec3Array> m_vertices; ///< 顶点数组
    osg::ref_ptr<osg::Vec4Array> m_colors;   ///< 颜色数组
    osg::ref_ptr<osg::LineWidth> m_lineWidth; ///< 线宽状态属性
    float m_width = 2.0f;                    ///< 当前线宽（默认 2 像素）
    std::vector<EdgeSegment> m_edgeSegments; ///< 边线段元数据缓存（用于拾取）
};
