// ============================================================================
// robust_kernel_io.cpp
// 鲁棒核函数的序列化与反序列化实现
//
// 本文件实现了鲁棒核函数的保存和加载功能，使得用户在图优化过程中
// 设置的各边鲁棒核参数可以在程序重启后恢复。
//
// 功能包括：
//   1. 识别鲁棒核函数的具体类型
//   2. 将图中所有边的核函数配置保存到文件
//   3. 从文件恢复核函数配置并应用到对应的边
// ============================================================================

#include "data/g2o/robust_kernel_io.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/sparse_optimizer.h>

namespace g2o {

// ============================================================================
// 核函数类型识别
// ============================================================================

/**
 * @brief 通过 dynamic_cast 获取鲁棒核函数的具体类型名称
 *
 * 逐一尝试将输入指针向下转型为各具体核函数类，命中后返回对应的名称字符串。
 * 识别顺序：Huber -> Cauchy -> DCS -> Fair -> GemanMcClure ->
 * PseudoHuber -> Saturated -> Tukey -> Welsch
 *
 * @param kernel 鲁棒核函数基类指针
 * @return 类型名称字符串（如 "Huber"、"Cauchy"），未知类型返回空字符串
 */
std::string kernel_type(g2o::RobustKernel* kernel) {
    if (dynamic_cast<g2o::RobustKernelHuber*>(kernel)) {
        return "Huber";
    }
    if (dynamic_cast<g2o::RobustKernelCauchy*>(kernel)) {
        return "Cauchy";
    }
    if (dynamic_cast<g2o::RobustKernelDCS*>(kernel)) {
        return "DCS";
    }
    if (dynamic_cast<g2o::RobustKernelFair*>(kernel)) {
        return "Fair";
    }
    if (dynamic_cast<g2o::RobustKernelGemanMcClure*>(kernel)) {
        return "GemanMcClure";
    }
    if (dynamic_cast<g2o::RobustKernelPseudoHuber*>(kernel)) {
        return "PseudoHuber";
    }
    if (dynamic_cast<g2o::RobustKernelSaturated*>(kernel)) {
        return "Saturated";
    }
    if (dynamic_cast<g2o::RobustKernelTukey*>(kernel)) {
        return "Tukey";
    }
    if (dynamic_cast<g2o::RobustKernelWelsch*>(kernel)) {
        return "Welsch";
    }
    return "";
}

// ============================================================================
// 保存鲁棒核函数配置
// ============================================================================

/**
 * @brief 将图中所有边的鲁棒核函数配置保存到文件
 *
 * 遍历优化器中的所有边，对有鲁棒核函数的边记录以下信息：
 *   - 边连接的顶点数量
 *   - 各顶点的 ID
 *   - 核函数类型名称
 *   - delta 参数值（核函数的阈值/尺度参数）
 *
 * 保存格式（每行）：顶点数 顶点ID1 顶点ID2 ... 核函数类型 delta
 * 示例：2 0 1 Huber 1.0
 *
 * @param filename 输出文件路径
 * @param graph    g2o 优化器指针
 * @return 保存是否成功
 */
bool save_robust_kernels(const std::string& filename, g2o::SparseOptimizer* graph) {
    std::ofstream ofs(filename);
    if (!ofs) {
        std::cerr << "failed to open output stream!!" << std::endl;
        return false;
    }

    // 遍历所有边，保存有鲁棒核函数的配置
    for (const auto& edge_ : graph->edges()) {
        g2o::OptimizableGraph::Edge* edge = static_cast<g2o::OptimizableGraph::Edge*>(edge_);
        g2o::RobustKernel* kernel = edge->robustKernel();
        if (!kernel) {
            continue;  // 跳过没有鲁棒核函数的边
        }

        std::string type = kernel_type(kernel);
        if (type.empty()) {
            std::cerr << "unknown kernel type!!" << std::endl;
            continue;
        }

        // 写入：顶点数量、各顶点 ID、核函数类型、delta 值
        ofs << edge->vertices().size() << " ";
        for (size_t i = 0; i < edge->vertices().size(); i++) {
            ofs << edge->vertices()[i]->id() << " ";
        }
        ofs << type << " " << kernel->delta() << std::endl;
    }

    return true;
}

// ============================================================================
// 核函数数据辅助类（用于反序列化）
// ============================================================================

/**
 * @brief 从文件解析并存储一个鲁棒核函数的配置信息
 *
 * 辅助类，用于在加载过程中解析文件的每一行并存储：
 *   - 边连接的顶点 ID 列表
 *   - 核函数类型
 *   - delta 参数
 * 并提供匹配和构造功能。
 */
class KernelData {
public:
    /**
     * @brief 从一行文本解析核函数数据
     *
     * 解析格式：顶点数 顶点ID1 顶点ID2 ... 核函数类型 delta
     *
     * @param line 文本行
     */
    KernelData(const std::string& line) {
        std::stringstream sst(line);
        size_t num_vertices;
        sst >> num_vertices;

        vertex_indices.resize(num_vertices);
        for (size_t i = 0; i < num_vertices; i++) {
            sst >> vertex_indices[i];
        }

        sst >> type >> delta;
    }

    /**
     * @brief 判断此核函数数据是否与给定的边匹配
     *
     * 通过比较顶点数量和顶点 ID 列表进行匹配。
     *
     * @param edge 待匹配的边
     * @return 匹配成功返回 true
     */
    bool match(g2o::OptimizableGraph::Edge* edge) const {
        // 首先检查顶点数量是否一致
        if (edge->vertices().size() != vertex_indices.size()) {
            return false;
        }

        // 检查所有顶点 ID 是否一致
        for (size_t i = 0; i < edge->vertices().size(); i++) {
            if (edge->vertices()[i]->id() != vertex_indices[i]) {
                return false;
            }
        }

        return true;
    }

    /**
     * @brief 根据存储的类型和参数创建鲁棒核函数实例
     *
     * 通过 RobustKernelFactory 使用类型名构造对应的核函数，
     * 然后设置 delta 参数。
     *
     * @return 新创建的鲁棒核函数指针（调用方负责管理生命周期）
     */
    g2o::RobustKernel* create() const {
        auto factory = g2o::RobustKernelFactory::instance();
        g2o::RobustKernel* kernel = factory->construct(type);
        kernel->setDelta(delta);
        return kernel;
    }

public:
    std::vector<int> vertex_indices;  ///< 边连接的顶点 ID 列表
    std::string type;                  ///< 鲁棒核函数类型名称
    double delta;                      ///< 核函数 delta 参数（阈值/尺度）
};

// ============================================================================
// 加载鲁棒核函数配置
// ============================================================================

/**
 * @brief 从文件加载鲁棒核函数配置并应用到图中对应的边
 *
 * 加载流程：
 *   1. 读取文件，解析每行数据创建 KernelData 列表
 *   2. 遍历图中的所有边
 *   3. 对每条边，在 KernelData 列表中查找匹配项
 *   4. 匹配成功则创建对应的鲁棒核函数并设置到边上
 *   5. 输出未匹配核函数的警告信息
 *
 * @param filename 保存的核函数配置文件路径
 * @param graph    g2o 优化器指针
 * @return 加载是否成功（文件不存在时返回 true，仅输出警告）
 */
bool load_robust_kernels(const std::string& filename, g2o::SparseOptimizer* graph) {
    std::ifstream ifs(filename);
    if (!ifs) {
        // 文件不存在不是致命错误，仅输出警告
        std::cerr << "warning: failed to open input stream!!" << std::endl;
        return true;
    }

    // 解析所有行，构造 KernelData 列表
    std::vector<KernelData> kernels;

    while (!ifs.eof()) {
        std::string line;
        std::getline(ifs, line);
        if (line.empty()) {
            continue;  // 跳过空行
        }

        kernels.push_back(KernelData(line));
    }
    std::cout << "kernels: " << kernels.size() << std::endl;

    // 遍历图中所有边，匹配并应用核函数
    for (auto& edge_ : graph->edges()) {
        g2o::OptimizableGraph::Edge* edge = static_cast<g2o::OptimizableGraph::Edge*>(edge_);

        // 在已解析的核函数数据中查找匹配项
        for (auto itr = kernels.begin(); itr != kernels.end(); itr++) {
            if (itr->match(edge)) {
                edge->setRobustKernel(itr->create());
                kernels.erase(itr);  // 匹配成功后移除该条目
                break;
            }
        }
    }

    // 检查是否有未匹配的核函数数据
    if (kernels.size() != 0) {
        std::cerr << "warning: there is non-associated kernels!!" << std::endl;
    }
    return true;
}

}  // namespace g2o
