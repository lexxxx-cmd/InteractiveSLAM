// ============================================================================
// edge_se3_priorxyz.hpp
// 三维位置先验边 —— g2o 自定义一元边
//
// 功能：为一个 SE3 顶点的完整三维位置 (x, y, z) 添加先验约束。
//       当已知某帧的绝对位置（如 GPS-RTK、激光雷达定位、运动捕获数据）时使用。
//
// 误差维度：3（x, y, z 位移残差）
// 测量类型：Eigen::Vector3d（目标位置的 x, y, z 坐标）
// 顶点类型：g2o::VertexSE3（6 自由度位姿顶点）
// ============================================================================

#pragma once

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {

/**
 * @brief 三维位置 (XYZ) 先验一元边
 *
 * 用于对 SE3 位姿顶点的完整平移分量 (x, y, z) 施加先验约束。
 * 这是最常用的位置先验边，适用于以下场景：
 *   - RTK-GPS 提供的高精度三维位置
 *   - 激光雷达扫描匹配后获得的地图坐标系下位置
 *   - 运动捕获系统提供的标记点位置
 *   - 人工标注的关键帧位置
 *
 * 此边不约束顶点的旋转部分，通常与 EdgeSE3PriorQuat 组合使用
 * 以实现对完整 6 自由度的先验约束。
 */
class EdgeSE3PriorXYZ : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  ///< Eigen 内存对齐要求

    /** @brief 默认构造函数 */
    EdgeSE3PriorXYZ() : g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3>() {}

    /**
     * @brief 计算误差向量
     *
     * 提取顶点估计位置的三维向量，直接与测量值比较。
     * 误差 = [x_est - x_meas, y_est - y_meas, z_est - z_meas]
     */
    void computeError() override {
        const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
        Eigen::Vector3d estimate = v1->estimate().translation();
        _error = estimate - _measurement;
    }

    /**
     * @brief 设置测量值
     * @param m 3 维测量向量 (x, y, z)
     */
    void setMeasurement(const Eigen::Vector3d& m) override { _measurement = m; }

    /**
     * @brief 从输入流读取边数据（用于加载 .g2o 文件）
     *
     * 格式：x y z [信息矩阵上三角]
     *
     * @param is 输入流
     * @return 读取是否成功
     */
    virtual bool read(std::istream& is) override {
        Eigen::Vector3d v;
        is >> v(0) >> v(1) >> v(2);
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
        Eigen::Vector3d v = _measurement;
        os << v(0) << " " << v(1) << " " << v(2) << " ";
        // 写入信息矩阵的上三角部分
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
                os << " " << information()(i, j);
        return os.good();
    }
};
}  // namespace g2o
