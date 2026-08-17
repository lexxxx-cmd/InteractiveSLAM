/**
 * @file DrawFlags.h
 * @brief 绘制标志枚举定义
 *
 * 定义在 3D 视口中控制各类元素渲染开关的结构体。
 * 该文件最初对应已删除的 drawable_object.hpp 中的标志位定义。
 * 用于 ViewportWidget 和 GraphSceneVisualizer 之间传递渲染状态。
 */

#pragma once

namespace hdl_graph_slam {

/**
 * @brief 绘制标志结构体
 *
 * 控制 3D 视口中哪些元素被渲染。
 * ViewportWidget 持有此结构体实例，通过 setter 方法修改标志位，
 * 并将标志传递给 GraphSceneVisualizer 以控制实际渲染内容。
 */
struct DrawFlags {
    bool draw_verticies        = true;   ///< 是否绘制所有顶点（球体）
    bool draw_edges            = true;   ///< 是否绘制所有边（连线）
    bool draw_keyframe_vertices = true;  ///< 是否绘制关键帧点云
    bool draw_se3_edges        = true;   ///< 是否绘制 SE3 约束边
    bool z_clipping            = false;  ///< 是否启用 Z 轴裁剪
    int  sample_stride         = 1;     ///< 渲染采样步长（1=全部渲染, N=每N帧渲染1个球体）
    int  point_budget          = 5000000; ///< 点云渲染点预算（≤0=全量，超过自动体素降采样）
};

}  // namespace hdl_graph_slam
