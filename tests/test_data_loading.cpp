// ============================================================================
// test_data_loading.cpp
// 数据加载测试
//
// 功能：命令行测试程序，用于验证 InteractiveGraph 的地图数据加载功能。
//       从指定文件夹加载地图数据，输出加载结果的统计信息，
//       包括顶点数、边数、关键帧数以及每个关键帧的详情。
//
// 使用方式：
//   test_data_loading <map_folder>
//
// 进度报告：使用简单的控制台输出来报告加载进度。
// ============================================================================

#include "data/hdl_graph_slam/interactive_graph.hpp"
#include <iostream>
#include <cassert>

/**
 * @brief 控制台进度报告器
 *
 * 实现 ProgressInterface 接口，将加载进度输出到控制台。
 * 适用于无 GUI 的命令行测试环境。
 */
struct ConsoleProgress : hdl_graph_slam::ProgressInterface {
    /** @brief 设置标题（如 "正在加载地图..."） */
    void set_title(const std::string& title) override {
        std::cout << "=== " << title << " ===" << std::endl;
    }
    /** @brief 设置文本描述（如 "正在处理文件..."） */
    void set_text(const std::string& text) override {
        std::cout << "  " << text << "..." << std::endl;
    }
    /** @brief 进度递增（输出一个点号表示进度） */
    void increment() override { std::cout << "." << std::flush; }
};

/**
 * @brief 测试入口函数
 *
 * 从命令行参数获取地图文件夹路径，加载数据并输出统计结果。
 *
 * @param argc 命令行参数数量
 * @param argv 命令行参数数组
 *             参数 1：地图数据文件夹路径
 * @return 成功返回 0，失败返回 1
 */
int main(int argc, char** argv) {
    // 检查命令行参数
    if (argc < 2) {
        std::cerr << "Usage: test_data_loading <map_folder>" << std::endl;
        return 1;
    }

    // 创建图实例和进度报告器
    hdl_graph_slam::InteractiveGraph graph;
    ConsoleProgress progress;

    // 加载地图数据
    bool ok = graph.load_map_data(argv[1], progress);
    assert(ok && "Failed to load map data");  // 断言加载成功

    // 输出加载结果统计
    std::cout << "\n--- Load Results ---" << std::endl;
    std::cout << "Vertices: " << graph.num_vertices() << std::endl;  // 顶点数量
    std::cout << "Edges:    " << graph.num_edges() << std::endl;     // 边的数量
    std::cout << "Keyframes: " << graph.keyframes.size() << std::endl; // 关键帧数量

    // 如果存在锚点节点，输出其 ID
    if (graph.anchor_node_id() >= 0) {
        std::cout << "Anchor node ID: " << graph.anchor_node_id() << std::endl;
    }

    // 输出每个关键帧的详细信息
    for (auto& [id, kf] : graph.keyframes) {
        std::cout << "  KF " << id
                  << " | pts: " << kf->cloud->size()           // 点云点数
                  << " | pos: " << kf->estimate().translation().transpose()  // 位姿位置
                  << std::endl;
    }

    std::cout << "\n--- All tests passed ---" << std::endl;
    return 0;
}
