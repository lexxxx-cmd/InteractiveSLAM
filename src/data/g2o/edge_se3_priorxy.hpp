// ============================================================================
// edge_se3_priorxy.hpp
// XY 位置先验边 —— g2o 自定义一元边
//
// 功能：为一个 SE3 顶点的水平位置（x, y）添加先验约束。
//       当已知某帧的绝对水平位置（如 GPS、UWB 定位数据）时使用。
//
// 误差维度：2（x, y 位移残差）
// 测量类型：Eigen::Vector2d（目标位置的 x, y 坐标）
// 顶点类型：g2o::VertexSE3（6 自由度位姿顶点）
// ============================================================================

#pragma once

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {

/**
 * @brief 水平位置 (XY) 先验一元边
 *
 * 用于对 SE3 位姿顶点的水平平移分量 (x, y) 施加先验约束。
 * 适用于以下场景：
 *   - GPS 提供的水平位置定位
 *   - UWB 室内定位的水平坐标
 *   - 已知地面控制点的水平位置
 *
 * 此边忽略 z 轴和平移分量和旋转部分，仅约束 x 和 y。
 * 通常与 EdgeSE3PriorZ 或 EdgeSE3PriorQuat 等组合使用。
 */
class EdgeSE3PriorXY : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  ///< Eigen 内存对齐要求

    /** @brief 默认构造函数 */
    EdgeSE3PriorXY() : g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3>() {}

    /**
     * @brief 计算误差向量
     *
     * 提取顶点估计位置的前两维 (x, y)，与测量值比较。
     * 误差 = [estimate_x - measurement_x, estimate_y - measurement_y]
     */
    void computeError() override {
        const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
        Eigen::Vector2d estimate = v1->estimate().translation().head<2>();
        _error = estimate - _measurement;
    }

    /**
     * @brief 设置测量值
     * @param m 2 维测量向量 (x, y)
     */
    void setMeasurement(const Eigen::Vector2d& m) override { _measurement = m; }

    /**
     * @brief 从输入流读取边数据（用于加载 .g2o 文件）
     *
     * 格式：x y [信息矩阵上三角]
     *
     * @param is 输入流
     * @return 读取是否成功
     */
    virtual bool read(std::istream& is) override {
        Eigen::Vector2d v;
        is >> v(0) >> v(1);
        setMeasurement(v);
        // 读取信息矩阵的上三角部分，并对称填充
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j) {
                is >> information()(i, j);
                if (i != j) information()(j, i) = information()(i, j);
            }
        return true;
    }

    /**
     * @brief 将边数据写入输出流（用于保存 .g2o 文件）
     *
     * @param os 输出流
     * @return 写入是否成功
     */
    virtual bool write(std::ostream& os) const override {
        Eigen::Vector2d v = _measurement;
        os << v(0) << " " << v(1) << " ";
        // 写入信息矩阵的上三角部分
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
                os << " " << information()(i, j);
        return os.good();
    }
};
}  // namespace g2o
