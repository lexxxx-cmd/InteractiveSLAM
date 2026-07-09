/**
 * @file graph_slam.hpp
 * @brief g2o 图结构轻量封装 —— GraphSLAM 基类定义
 *
 * GraphSLAM 是对 g2o（General Graph Optimization）库的轻量级封装，
 * 提供了一个简化的接口来创建和管理 SLAM 位姿图。
 *
 * 主要功能：
 *   - 创建 SE3 位姿节点（顶点）
 *   - 创建 SE3 约束边（包括相对位姿边和各种先验边）
 *   - 支持鲁棒核函数（降低外点影响）
 *   - 执行图优化（Levenberg-Marquardt 等算法）
 *   - 序列化保存和加载图文件（.g2o 格式）
 *
 * 本版本为 SE3-only 适配，移除了平面相关的所有方法和类型注册。
 */
#pragma once

#include <memory>
#include <Eigen/Dense>

#include <g2o/core/hyper_graph.h>

namespace g2o {
class VertexSE3;        ///< SE3 位姿顶点（前向声明）
class EdgeSE3;          ///< SE3 相对位姿边（前向声明）
class EdgeSE3PriorXY;   ///< XY 坐标先验边（前向声明）
class EdgeSE3PriorXYZ;  ///< XYZ 坐标先验边（前向声明）
class EdgeSE3PriorVec;  ///< 方向向量先验边（前向声明）
class EdgeSE3PriorQuat; ///< 四元数姿态先验边（前向声明）
class RobustKernelFactory;  ///< 鲁棒核函数工厂（前向声明）
}  // namespace g2o

namespace hdl_graph_slam {

/**
 * @brief g2o 图结构的轻量级封装，提供位姿图优化的核心接口
 *
 * GraphSLAM 封装了 g2o::SparseOptimizer，提供了简化的 API 用于：
 *   - 管理图结构（添加顶点和边）
 *   - 配置优化求解器
 *   - 执行图优化
 *   - 保存/加载图文件
 *
 * 继承关系：
 *   InteractiveGraph 继承自此类，扩展了交互式操作功能。
 *
 * 注意：
 *   原始版本包含平面相关的节点和边类型，本适配版本已全部移除。
 *   仅保留 SE3 相关的节点和边类型。
 */
class GraphSLAM {
public:
    /**
     * @brief 构造函数
     * @param solver_type 优化求解器类型（默认 "lm_var"）
     *
     * 可用求解器类型：
     *   - "lm_var"：Levenberg-Marquardt + 变分求解
     *   - "lm_var_cholmod"：LM + Cholmod 分解
     *   - "lm_fix3_var"：固定 3 自由度的 LM
     *
     * 初始化流程：
     *   1. 创建 SparseOptimizer 实例
     *   2. 根据 solver_type 构造求解器
     *   3. 获取鲁棒核函数工厂实例
     */
    GraphSLAM(const std::string& solver_type = "lm_var");

    virtual ~GraphSLAM();

    /**
     * @brief 获取图中顶点的数量
     * @return 顶点个数
     */
    int num_vertices() const;

    /**
     * @brief 获取图中边的数量
     * @return 边的条数
     */
    int num_edges() const;

    /**
     * @brief 设置优化求解器类型
     * @param solver_type 求解器类型字符串
     *
     * 可在优化前动态切换求解器。
     */
    void set_solver(const std::string& solver_type);

    /**
     * @brief 添加 SE3 位姿节点（顶点）
     * @param pose 节点位姿（等距变换）
     * @return 创建的 SE3 顶点指针
     *
     * 自动分配节点 ID（从 0 开始自增）。
     */
    g2o::VertexSE3* add_se3_node(const Eigen::Isometry3d& pose);

    /**
     * @brief 添加 SE3 相对位姿约束边
     * @param v1                第一个顶点
     * @param v2                第二个顶点
     * @param relative_pose     从 v1 到 v2 的相对位姿变换
     * @param information_matrix 信息矩阵（表示约束的置信度）
     * @return 创建的 SE3 边指针
     */
    g2o::EdgeSE3* add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2,
                               const Eigen::Isometry3d& relative_pose,
                               const Eigen::MatrixXd& information_matrix);

    /**
     * @brief 添加 XY 坐标先验边（约束顶点的水平位置）
     * @param v_se3             SE3 顶点
     * @param xy                先验的 XY 坐标
     * @param information_matrix 信息矩阵
     * @return 创建的 XY 先验边指针
     */
    g2o::EdgeSE3PriorXY* add_se3_prior_xy_edge(g2o::VertexSE3* v_se3,
                                                const Eigen::Vector2d& xy,
                                                const Eigen::MatrixXd& information_matrix);

    /**
     * @brief 添加 XYZ 坐标先验边（约束顶点的三维位置）
     * @param v_se3             SE3 顶点
     * @param xyz               先验的三维坐标
     * @param information_matrix 信息矩阵
     * @return 创建的 XYZ 先验边指针
     */
    g2o::EdgeSE3PriorXYZ* add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3,
                                                  const Eigen::Vector3d& xyz,
                                                  const Eigen::MatrixXd& information_matrix);

    /**
     * @brief 添加四元数姿态先验边（约束顶点的朝向）
     * @param v_se3             SE3 顶点
     * @param quat              先验的姿态四元数
     * @param information_matrix 信息矩阵
     * @return 创建的四元数先验边指针
     */
    g2o::EdgeSE3PriorQuat* add_se3_prior_quat_edge(g2o::VertexSE3* v_se3,
                                                    const Eigen::Quaterniond& quat,
                                                    const Eigen::MatrixXd& information_matrix);

    /**
     * @brief 添加方向向量先验边（约束顶点的某个方向）
     * @param v_se3             SE3 顶点
     * @param direction         方向向量
     * @param measurement       测量值
     * @param information_matrix 信息矩阵
     * @return 创建的方向向量先验边指针
     */
    g2o::EdgeSE3PriorVec* add_se3_prior_vec_edge(g2o::VertexSE3* v_se3,
                                                  const Eigen::Vector3d& direction,
                                                  const Eigen::Vector3d& measurement,
                                                  const Eigen::MatrixXd& information_matrix);

    /**
     * @brief 为边添加鲁棒核函数
     * @param edge        目标边
     * @param kernel_type 核函数类型（"Huber"、"Cauchy"、"DCS" 等）
     * @param kernel_size 核函数 delta 参数
     *
     * 鲁棒核函数用于降低外点（错误匹配）对优化结果的影响。
     * 若 kernel_type 为 "NONE"，则不添加任何核函数。
     */
    void add_robust_kernel(g2o::HyperGraph::Edge* edge,
                           const std::string& kernel_type, double kernel_size);

    /**
     * @brief 执行图优化
     * @param num_iterations 最大迭代次数
     * @return 实际执行的迭代次数（若边数不足 10 条则返回 -1）
     *
     * 优化流程：
     *   1. 检查边数，若少于 10 条则跳过优化
     *   2. 初始化优化器
     *   3. 执行指定次数的 LM/高斯-牛顿迭代
     *   4. 输出优化前后的卡方值（chi2）和耗时
     */
    int optimize(int num_iterations);

    /**
     * @brief 保存图到文件
     * @param filename 输出文件路径（.g2o 格式）
     *
     * 同时保存鲁棒核函数信息到独立的 .kernels 文件。
     */
    void save(const std::string& filename);

    /**
     * @brief 从文件加载图
     * @param filename 输入的 .g2o 文件路径
     * @return 加载成功返回 true
     *
     * 加载 .g2o 文件后，自动加载同名的 .kernels 文件恢复鲁棒核函数设置。
     */
    bool load(const std::string& filename);

public:
    g2o::RobustKernelFactory* robust_kernel_factory;  ///< 鲁棒核函数工厂（用于创建各种鲁棒核函数）
    std::unique_ptr<g2o::HyperGraph> graph;             ///< g2o 图对象（HyperGraph 基类指针）
};

}  // namespace hdl_graph_slam
