// ============================================================================
// edge_se3_priorvec.hpp
// 向量方向先验边 —— g2o 自定义一元边
//
// 功能：为一个 SE3 顶点添加向量方向约束。
//       约束将顶点坐标系中的某个方向向量（如重力方向、主方向）
//       与世界坐标系中的参考方向对齐。
//
// 误差维度：3（方向向量的残差）
// 测量类型：Eigen::Matrix<double, 6, 1>
//           - 前 3 维：顶点局部坐标系中的方向向量（如传感器坐标系中的重力方向）
//           - 后 3 维：世界坐标系中的参考方向向量（如世界坐标系中的重力方向 [0,0,-1]）
// 顶点类型：g2o::VertexSE3（6 自由度位姿顶点）
// ============================================================================

#pragma once

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/types/slam3d_addons/types_slam3d_addons.h>

namespace g2o {

/**
 * @brief 向量方向先验一元边
 *
 * 用于约束 SE3 位姿的旋转部分，使得顶点坐标系中的某个指定方向
 * 经过旋转后与世界坐标系中的参考方向对齐。
 *
 * 典型应用场景：
 *   - 重力方向约束：已知 IMU 测量的重力方向在传感器坐标系中的表示，
 *     约束位姿使其与世界坐标系的重力方向对齐，从而修正俯仰和横滚角。
 *   - 主方向约束：在结构化环境中，约束位姿与建筑物的主方向对齐。
 *
 * 误差计算：将局部方向向量通过顶点旋转的逆变换到世界坐标系，
 * 然后与参考方向向量比较。
 */
class EdgeSE3PriorVec : public g2o::BaseUnaryEdge<3, Eigen::Matrix<double, 6, 1>, g2o::VertexSE3> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  ///< Eigen 内存对齐要求

    /** @brief 默认构造函数 */
    EdgeSE3PriorVec() : g2o::BaseUnaryEdge<3, Eigen::Matrix<double, 6, 1>, g2o::VertexSE3>() {}

    /**
     * @brief 计算误差向量
     *
     * 将局部方向向量（前 3 维）通过顶点旋转的逆变换到世界坐标系，
     * 然后与世界坐标系中的参考方向（后 3 维）比较，计算残差。
     *
     * 误差 = R^{-1} * direction_local - direction_world
     */
    void computeError() override {
        const g2o::VertexSE3* v1 = static_cast<const g2o::VertexSE3*>(_vertices[0]);
        Eigen::Vector3d direction  = _measurement.head<3>();   // 局部坐标系中的方向向量
        Eigen::Vector3d measurement = _measurement.tail<3>();  // 世界坐标系中的参考方向
        // 将局部方向向量变换到世界坐标系并计算误差
        Eigen::Vector3d estimate = (v1->estimate().linear().inverse() * direction);
        _error = estimate - measurement;
    }

    /**
     * @brief 设置测量值
     *
     * 对局部方向向量和参考方向向量分别进行归一化处理。
     *
     * @param m 6 维测量向量 [dir_x, dir_y, dir_z, ref_x, ref_y, ref_z]
     */
    void setMeasurement(const Eigen::Matrix<double, 6, 1>& m) override {
        _measurement.head<3>() = m.head<3>().normalized();   // 局部方向向量归一化
        _measurement.tail<3>() = m.tail<3>().normalized();   // 世界参考方向向量归一化
    }

    /**
     * @brief 从输入流读取边数据（用于加载 .g2o 文件）
     *
     * 格式：dir_x dir_y dir_z ref_x ref_y ref_z [信息矩阵上三角]
     *
     * @param is 输入流
     * @return 读取是否成功
     */
    virtual bool read(std::istream& is) override {
        Eigen::Matrix<double, 6, 1> v;
        is >> v[0] >> v[1] >> v[2] >> v[3] >> v[4] >> v[5];
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
        Eigen::Matrix<double, 6, 1> v = _measurement;
        os << v[0] << " " << v[1] << " " << v[2] << " "
           << v[3] << " " << v[4] << " " << v[5];
        // 写入信息矩阵的上三角部分
        for (int i = 0; i < information().rows(); ++i)
            for (int j = i; j < information().cols(); ++j)
                os << " " << information()(i, j);
        return os.good();
    }
};
}  // namespace g2o
