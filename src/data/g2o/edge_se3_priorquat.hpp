// ============================================================================
// edge_se3_priorquat.hpp
// 四元数先验边 —— g2o 自定义一元边
//
// 功能：为一个 SE3 顶点添加四元数姿态（旋转）的先验约束。
//       当已知某帧的绝对姿态（如 GPS/IMU 提供的方向）时使用，
//       将该帧的旋转约束在测量值附近。
//
// 误差维度：3（四元数的虚部向量 = [x, y, z]）
// 测量类型：Eigen::Quaterniond（单位四元数）
// 顶点类型：g2o::VertexSE3（6 自由度位姿顶点）
// ============================================================================

#pragma once

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {

/**
 * @brief 四元数姿态先验一元边
 *
 * 用于对 SE3 位姿顶点的旋转部分施加先验约束。
 * 当已知某个关键帧的绝对方向（如通过 IMU 积分或视觉定位获得）时，
 * 使用此边将其约束在测量值附近。
 *
 * 误差计算：将顶点估计的旋转矩阵转换为四元数，计算其虚部向量与
 * 测量四元数虚部向量之差。处理了四元数的符号二义性问题
 * （q 和 -q 表示相同的旋转），确保估计与测量具有相同的符号方向。
 */
class EdgeSE3PriorQuat : public g2o::BaseUnaryEdge<3, Eigen::Quaterniond, g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  ///< Eigen 内存对齐要求

    /** @brief 默认构造函数 */
    EdgeSE3PriorQuat() : g2o::BaseUnaryEdge<3, Eigen::Quaterniond, g2o::VertexSE3>() {}

    /**
     * @brief 计算误差向量
     *
     * 从顶点估计的旋转矩阵提取四元数，然后与测量值比较。
     * 使用四元数的虚部向量（.vec() = [x, y, z]）作为 3 维误差。
     * 处理了符号二义性：如果估计与测量的点积为负，翻转估计的符号。
     */
    void computeError() override {
        const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
        Eigen::Quaterniond estimate = Eigen::Quaterniond(v1->estimate().linear());
        // 解决四元数的符号二义性（q 和 -q 表示同一旋转）
        if (_measurement.coeffs().dot(estimate.coeffs()) < 0.0) {
            estimate.coeffs() = -estimate.coeffs();
        }
        _error = estimate.vec() - _measurement.vec();
    }

    /**
     * @brief 设置测量值
     *
     * 如果测量四元数的实部 w 为负，翻转整个四元数符号，
     * 确保存储的测量值始终具有正的 w 分量。
     *
     * @param m 测量四元数
     */
    void setMeasurement(const Eigen::Quaterniond& m) override {
        _measurement = m;
        if (m.w() < 0.0) {
            _measurement.coeffs() = -m.coeffs();
        }
    }

    /**
     * @brief 从输入流读取边数据（用于加载 .g2o 文件）
     *
     * 格式：w x y z [信息矩阵上三角]
     *
     * @param is 输入流
     * @return 读取是否成功
     */
    virtual bool read(std::istream& is) override {
        Eigen::Quaterniond q;
        is >> q.w() >> q.x() >> q.y() >> q.z();
        setMeasurement(q);
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
        Eigen::Quaterniond q = _measurement;
        os << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
        // 写入信息矩阵的上三角部分
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
                os << " " << information()(i, j);
        return os.good();
    }
};
}  // namespace g2o
