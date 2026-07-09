/**
 * @file graph_slam.cpp
 * @brief g2o 图结构封装的实现 —— GraphSLAM
 *
 * 该文件实现了 GraphSLAM 类的所有方法，提供对 g2o 稀疏优化器的封装。
 *
 * 核心功能实现：
 *   1. 求解器构建与配置（支持多种后端求解器）
 *   2. 顶点（Node）的创建与添加
 *   3. 各种边（Edge）的创建与添加（相对位姿边、先验边）
 *   4. 鲁棒核函数的添加
 *   5. 图优化流程
 *   6. 图的序列化保存与加载
 *
 * 依赖的自定义 g2o 类型（由 vcpkg 的标准 g2o 提供）：
 *   标准 SE3 顶点和边由 g2o 的 types_slam3d 提供。
 *   SE3 先验边（PriorXY/PriorXYZ/PriorVec/PriorQuat）从项目本地
 *   data/g2o/ 目录加载并进行类型注册。
 */
#include "data/hdl_graph_slam/graph_slam.hpp"

#include <iostream>
#include <fstream>
#include <chrono>
#include <boost/format.hpp>

#include <g2o/stuff/macros.h>
#include <g2o/core/factory.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/optimization_algorithm.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/slam3d/types_slam3d.h>

// 自定义 g2o 类型（SE3 先验边，非 vcpkg 标准 g2o 提供）
#include "data/g2o/edge_se3_priorxy.hpp"
#include "data/g2o/edge_se3_priorxyz.hpp"
#include "data/g2o/edge_se3_priorvec.hpp"
#include "data/g2o/edge_se3_priorquat.hpp"

// 鲁棒核函数 I/O（项目本地拷贝）
#include "data/g2o/robust_kernel_io.hpp"

// 使用可用的优化库
G2O_USE_OPTIMIZATION_LIBRARY(pcg)      ///< PCG 共轭梯度求解器
G2O_USE_OPTIMIZATION_LIBRARY(cholmod)  ///< Cholmod 分解求解器
G2O_USE_OPTIMIZATION_LIBRARY(csparse)  ///< CSparse 分解求解器

namespace g2o {
// 注册自定义 g2o 边类型到工厂，使其可在 .g2o 文件中按名称识别
G2O_REGISTER_TYPE(EDGE_SE3_PRIORXY, EdgeSE3PriorXY)
G2O_REGISTER_TYPE(EDGE_SE3_PRIORXYZ, EdgeSE3PriorXYZ)
G2O_REGISTER_TYPE(EDGE_SE3_PRIORVEC, EdgeSE3PriorVec)
G2O_REGISTER_TYPE(EDGE_SE3_PRIORQUAT, EdgeSE3PriorQuat)
}  // namespace g2o

namespace hdl_graph_slam {

/**
 * @brief 构造函数：创建 g2o 稀疏优化器并配置求解器
 * @param solver_type 求解器类型字符串
 *
 * 初始化流程：
 *   1. 创建 SparseOptimizer 实例
 *   2. 通过 OptimizationAlgorithmFactory 根据 solver_type 构建求解器
 *   3. 将求解器绑定到优化器
 *   4. 获取鲁棒核函数工厂实例
 *
 * 如果指定求解器类型不可用，会列出所有可用求解器并报错。
 */
GraphSLAM::GraphSLAM(const std::string& solver_type) {
    graph.reset(new g2o::SparseOptimizer());
    g2o::SparseOptimizer* g = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

    std::cout << "construct solver: " << solver_type << std::endl;
    // 通过工厂模式创建求解器
    g2o::OptimizationAlgorithmFactory* solver_factory =
        g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm* solver =
        solver_factory->construct(solver_type, solver_property);
    g->setAlgorithm(solver);

    if (!g->solver()) {
        std::cerr << "error : failed to allocate solver!!" << std::endl;
        // 列出所有可用求解器供用户参考
        solver_factory->listSolvers(std::cerr);
        std::cerr << "-------------" << std::endl;
        return;
    }
    std::cout << "done" << std::endl;

    // 获取鲁棒核函数工厂的单例实例
    robust_kernel_factory = g2o::RobustKernelFactory::instance();
}

GraphSLAM::~GraphSLAM() {
    graph.reset();
}

/**
 * @brief 切换优化求解器类型
 * @param solver_type 新的求解器类型
 *
 * 可在不重建整个 GraphSLAM 对象的情况下动态切换求解器。
 */
void GraphSLAM::set_solver(const std::string& solver_type) {
    g2o::SparseOptimizer* g = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

    std::cout << "construct solver: " << solver_type << std::endl;
    g2o::OptimizationAlgorithmFactory* solver_factory =
        g2o::OptimizationAlgorithmFactory::instance();
    g2o::OptimizationAlgorithmProperty solver_property;
    g2o::OptimizationAlgorithm* solver =
        solver_factory->construct(solver_type, solver_property);
    g->setAlgorithm(solver);

    if (!g->solver()) {
        std::cerr << "error : failed to allocate solver!!" << std::endl;
        solver_factory->listSolvers(std::cerr);
        std::cerr << "-------------" << std::endl;
        return;
    }
    std::cout << "done" << std::endl;
}

int GraphSLAM::num_vertices() const { return graph->vertices().size(); }
int GraphSLAM::num_edges() const { return graph->edges().size(); }

/**
 * @brief 创建一个新的 SE3 位姿节点
 * @param pose 节点的初始位姿（等距变换）
 * @return 创建的 SE3 顶点指针
 *
 * 节点的 ID 自动分配为当前图中的顶点数量（从 0 开始自增）。
 */
g2o::VertexSE3* GraphSLAM::add_se3_node(const Eigen::Isometry3d& pose) {
    g2o::VertexSE3* vertex(new g2o::VertexSE3());
    vertex->setId(static_cast<int>(graph->vertices().size()));  // 自动分配 ID
    vertex->setEstimate(pose);                                    // 设置初始位姿估计
    graph->addVertex(vertex);                                     // 添加到图
    return vertex;
}

/**
 * @brief 创建 SE3 相对位姿约束边
 * @param v1                第一个顶点（源）
 * @param v2                第二个顶点（目标）
 * @param relative_pose     从 v1 到 v2 的相对位姿变换
 * @param information_matrix 信息矩阵（6x6，表示约束的置信度/精度）
 * @return 创建的 SE3 边指针
 *
 * 信息矩阵是协方差矩阵的逆，值越大表示约束越可靠。
 * 对于 SE3 边，信息矩阵是 6x6 的，前 3 维对应平移，后 3 维对应旋转。
 */
g2o::EdgeSE3* GraphSLAM::add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2,
                                       const Eigen::Isometry3d& relative_pose,
                                       const Eigen::MatrixXd& information_matrix) {
    g2o::EdgeSE3* edge(new g2o::EdgeSE3());
    edge->setMeasurement(relative_pose);              // 设置测量值（相对位姿）
    edge->setInformation(information_matrix);          // 设置信息矩阵
    edge->vertices()[0] = v1;                          // 设置源顶点
    edge->vertices()[1] = v2;                          // 设置目标顶点
    graph->addEdge(edge);                              // 添加到图
    return edge;
}

/**
 * @brief 创建 XY 坐标先验边
 * @param v_se3             SE3 顶点
 * @param xy                先验的 XY 坐标
 * @param information_matrix 信息矩阵
 * @return 创建的 XY 先验边指针
 *
 * 用于约束顶点在水平面（XY）上的位置，例如 GPS 测量提供的水平位置约束。
 */
g2o::EdgeSE3PriorXY* GraphSLAM::add_se3_prior_xy_edge(
    g2o::VertexSE3* v_se3, const Eigen::Vector2d& xy,
    const Eigen::MatrixXd& information_matrix) {
    g2o::EdgeSE3PriorXY* edge(new g2o::EdgeSE3PriorXY());
    edge->setMeasurement(xy);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph->addEdge(edge);
    return edge;
}

/**
 * @brief 创建 XYZ 坐标先验边
 * @param v_se3             SE3 顶点
 * @param xyz               先验的三维坐标
 * @param information_matrix 信息矩阵
 * @return 创建的 XYZ 先验边指针
 *
 * 用于约束顶点在三维空间中的位置。
 */
g2o::EdgeSE3PriorXYZ* GraphSLAM::add_se3_prior_xyz_edge(
    g2o::VertexSE3* v_se3, const Eigen::Vector3d& xyz,
    const Eigen::MatrixXd& information_matrix) {
    g2o::EdgeSE3PriorXYZ* edge(new g2o::EdgeSE3PriorXYZ());
    edge->setMeasurement(xyz);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph->addEdge(edge);
    return edge;
}

/**
 * @brief 创建方向向量先验边
 * @param v_se3             SE3 顶点
 * @param direction         方向向量（3 维）
 * @param measurement       测量值（3 维）
 * @param information_matrix 信息矩阵（6x6）
 * @return 创建的方向向量先验边指针
 *
 * 测量向量 m = [direction; measurement]，其中 direction 表示约束的方向
 * （如重力方向），measurement 为该方向上的测量值。
 */
g2o::EdgeSE3PriorVec* GraphSLAM::add_se3_prior_vec_edge(
    g2o::VertexSE3* v_se3, const Eigen::Vector3d& direction,
    const Eigen::Vector3d& measurement, const Eigen::MatrixXd& information_matrix) {
    Eigen::Matrix<double, 6, 1> m;
    m.head<3>() = direction;    ///< 方向向量（前 3 维）
    m.tail<3>() = measurement;  ///< 测量值（后 3 维）

    g2o::EdgeSE3PriorVec* edge(new g2o::EdgeSE3PriorVec());
    edge->setMeasurement(m);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph->addEdge(edge);
    return edge;
}

/**
 * @brief 创建四元数姿态先验边
 * @param v_se3             SE3 顶点
 * @param quat              先验的姿态四元数
 * @param information_matrix 信息矩阵
 * @return 创建的四元数先验边指针
 *
 * 用于约束顶点的朝向（姿态），例如 IMU 航向角测量。
 */
g2o::EdgeSE3PriorQuat* GraphSLAM::add_se3_prior_quat_edge(
    g2o::VertexSE3* v_se3, const Eigen::Quaterniond& quat,
    const Eigen::MatrixXd& information_matrix) {
    g2o::EdgeSE3PriorQuat* edge(new g2o::EdgeSE3PriorQuat());
    edge->setMeasurement(quat);
    edge->setInformation(information_matrix);
    edge->vertices()[0] = v_se3;
    graph->addEdge(edge);
    return edge;
}

/**
 * @brief 为边添加鲁棒核函数
 * @param edge        目标边
 * @param kernel_type 核函数类型字符串（"NONE" 则跳过）
 * @param kernel_size 核函数的 delta 参数（阈值）
 *
 * 支持的核函数类型（需与 g2o::RobustKernelFactory 注册一致）：
 *   "Huber"、"Cauchy"、"DCS"、"Fair"、"GemanMcClure" 等。
 *
 * 当 kernel_type 为 "NONE" 时，不添加任何核函数，使用标准的二次代价函数。
 */
void GraphSLAM::add_robust_kernel(g2o::HyperGraph::Edge* edge,
                                   const std::string& kernel_type,
                                   double kernel_size) {
    if (kernel_type == "NONE") {
        return;  // 不添加核函数
    }

    // 通过工厂创建指定的鲁棒核函数
    g2o::RobustKernel* kernel = robust_kernel_factory->construct(kernel_type);
    if (kernel == nullptr) {
        std::cerr << "warning : invalid robust kernel type: " << kernel_type << std::endl;
        return;
    }

    kernel->setDelta(kernel_size);  // 设置核函数的阈值参数
    g2o::OptimizableGraph::Edge* e = dynamic_cast<g2o::OptimizableGraph::Edge*>(edge);
    e->setRobustKernel(kernel);
}

/**
 * @brief 执行图优化
 * @param num_iterations 最大迭代次数
 * @return 实际执行的迭代次数（边数不足 10 时返回 -1）
 *
 * 优化流程：
 *   1. 检查图中边数，少于 10 条时跳过优化（图约束不足）
 *   2. 输出优化前的统计信息（顶点数、边数）
 *   3. 初始化优化器
 *   4. 开启详细输出模式
 *   5. 记录优化前的卡方值（chi2）
 *   6. 执行高斯-牛顿或 LM 迭代优化
 *   7. 记录优化后的卡方值和耗时
 *   8. 输出优化结果摘要
 */
int GraphSLAM::optimize(int num_iterations) {
    g2o::SparseOptimizer* g = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
    // 边数太少时（少于 10 条）跳过优化，因为约束不足无法获得有意义的优化结果
    if (g->edges().size() < 10) {
        return -1;
    }

    std::cout << std::endl;
    std::cout << "--- pose graph optimization ---" << std::endl;
    std::cout << "nodes: " << g->vertices().size()
              << "   edges: " << g->edges().size() << std::endl;
    std::cout << "optimizing... " << std::flush;

    g->initializeOptimization();  // 初始化优化器（计算稀疏结构等）
    g->setVerbose(true);          // 开启详细输出

    double chi2 = g->chi2();      // 优化前的总体卡方值

    auto t1 = std::chrono::high_resolution_clock::now();
    int iterations = g->optimize(num_iterations);  // 执行优化
    auto t2 = std::chrono::high_resolution_clock::now();

    std::cout << "done" << std::endl;
    std::cout << "iterations: " << iterations << " / " << num_iterations << std::endl;
    std::cout << "chi2: (before)" << chi2 << " -> (after)" << g->chi2() << std::endl;
    double elapsed = std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1e9;
    std::cout << "time: " << boost::format("%.3f") % elapsed << "[sec]" << std::endl;

    return iterations;
}

/**
 * @brief 保存图到 .g2o 文件
 * @param filename 输出文件路径
 *
 * 保存后额外生成 <filename>.kernels 文件记录每条边的鲁棒核函数配置。
 */
void GraphSLAM::save(const std::string& filename) {
    g2o::SparseOptimizer* g = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
    std::ofstream ofs(filename);
    g->save(ofs);  // 以 g2o 格式保存图
    // 保存鲁棒核函数信息到独立的 kernels 文件
    g2o::save_robust_kernels(filename + ".kernels", g);
}

/**
 * @brief 从 .g2o 文件加载图
 * @param filename 输入的 .g2o 文件路径
 * @return 加载成功返回 true
 *
 * 加载流程：
 *   1. 打开并加载 .g2o 文件
 *   2. 输出加载结果的统计信息（顶点数和边数）
 *   3. 自动加载同名的 .kernels 文件恢复鲁棒核函数设置
 *
 * 如果 .kernels 文件不存在或加载失败，仍会返回 true，
 * 但图可能缺少鲁棒核函数信息。
 */
bool GraphSLAM::load(const std::string& filename) {
    std::cout << "loading pose graph..." << std::endl;
    g2o::SparseOptimizer* g = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());

    std::ifstream ifs(filename);
    if (!g->load(ifs)) {
        return false;  // 加载失败
    }

    std::cout << "nodes  : " << g->vertices().size() << std::endl;
    std::cout << "edges  : " << g->edges().size() << std::endl;

    // 加载鲁棒核函数信息
    if (!g2o::load_robust_kernels(filename + ".kernels", g)) {
        return false;  // kernels 文件加载失败
    }

    return true;
}

}  // namespace hdl_graph_slam
