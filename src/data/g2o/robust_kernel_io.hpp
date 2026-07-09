// ============================================================================
// robust_kernel_io.hpp
// 鲁棒核函数的序列化与反序列化工具
//
// 功能：提供 g2o 图中各边所使用的鲁棒核函数的保存与加载功能。
//       在交互式 SLAM 中，用户可能动态调整各边的鲁棒核类型，
//       此工具确保这些设置在程序重启后仍能恢复。
// ============================================================================

#pragma once

#include <string>
#include <g2o/core/sparse_optimizer.h>

namespace g2o {

/**
 * @brief 获取鲁棒核函数的类型名称字符串
 *
 * 通过 dynamic_cast 检测具体核函数类型，返回对应的字符串标识。
 * 支持的核函数类型：Huber, Cauchy, DCS, Fair, GemanMcClure,
 * PseudoHuber, Saturated, Tukey, Welsch
 *
 * @param kernel 鲁棒核函数指针
 * @return 类型名称字符串（未知类型返回空字符串）
 */
std::string kernel_type(g2o::RobustKernel* kernel);

/**
 * @brief 将图中所有边的鲁棒核函数配置保存到文件
 *
 * 遍历图中的所有边，对有鲁棒核函数的边记录其类型名称和 delta 参数。
 * 保存格式（每行）：顶点数 id1 id2 ... 类型名称 delta
 *
 * @param filename 输出文件名
 * @param graph    优化器图指针
 * @return 保存是否成功
 */
bool save_robust_kernels(const std::string& filename, g2o::SparseOptimizer* graph);

/**
 * @brief 从文件加载鲁棒核函数配置并应用到图中对应的边
 *
 * 读取保存的鲁棒核函数配置文件，通过匹配顶点 ID 找到对应的边，
 * 然后为其设置相应的鲁棒核函数。未匹配到的核函数会输出警告。
 *
 * @param filename 输入文件名
 * @param graph    优化器图指针
 * @return 加载是否成功（文件不存在也返回 true，仅输出警告）
 */
bool load_robust_kernels(const std::string& filename, g2o::SparseOptimizer* graph);

}  // namespace g2o
