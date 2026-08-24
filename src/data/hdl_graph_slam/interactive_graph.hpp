/**
 * @file interactive_graph.hpp
 * @brief 交互式图（InteractiveGraph）定义
 *
 * InteractiveGraph 是 InterSLAM 系统的核心类，继承自 GraphSLAM，
 * 提供了对位姿图进行交互式操作的功能。
 *
 * 主要功能：
 *   1. 地图数据加载与保存（包括图文件、关键帧、特殊节点）
 *   2. 手动/自动添加循环闭合边（Edge）
 *   3. 删除指定的边
 *   4. 图优化（前台和后台线程）
 *   5. 图统计信息获取
 *   6. 导出点云和 LVBA 格式数据
 *
 * 本类移除了平面相关的操作（add_plane、add_edge_identity 等），
 * 仅支持 SE3 位姿图的加载、保存和优化。
 */
#pragma once

#include <regex>
#include <mutex>
#include <unordered_map>

#include <Eigen/Dense>
#include <boost/format.hpp>

#include "data/hdl_graph_slam/progress_interface.hpp"
#include "data/hdl_graph_slam/graph_slam.hpp"
#include "data/hdl_graph_slam/parameter_server.hpp"
#include "data/hdl_graph_slam/interactive_keyframe.hpp"
#include "data/hdl_graph_slam/information_matrix_calculator.hpp"

namespace g2o {
class VertexSE3;  ///< g2o SE3 位姿顶点（前向声明）
class EdgeSE3;    ///< g2o SE3 位姿边（前向声明）
}  // namespace g2o

namespace hdl_graph_slam {

/**
 * @brief 标记边的创建来源，用于 EdgeListPanel 过滤显示
 *
 * 不同的边来源在 UI 中具有不同的显示和可操作性：
 *   - Original：从文件加载，通常不可编辑
 *   - ManualLoop：用户手动添加，可删除
 *   - AutoLoop：自动检测添加，可查看/删除
 *   - Anchor：锚点约束边，通常隐藏
 */
enum class EdgeSource {
    Original,    ///< 从图文件加载的原始边（在 EdgeListPanel 中隐藏）
    ManualLoop,  ///< 用户手动添加的循环闭合边
    AutoLoop,    ///< 自动循环检测添加的边
    Anchor       ///< 锚点相关的约束边
};

/**
 * @brief 交互式位姿图操作核心类
 *
 * InteractiveGraph 提供了一套完整的交互式图操作接口，允许用户在
 * SLAM 过程中通过图形界面编辑和管理图结构。
 *
 * 核心能力：
 *   - 地图数据加载：加载 graph.g2o、关键帧序列和特殊节点
 *   - 边（Edge）管理：添加各种类型的边、按 ID 删除边
 *   - 图优化：前台同步优化（异步由上层以独立线程 + optimization_mutex 实现）
 *   - 数据导出：保存为标准格式或 LVBA 格式
 *   - 参数管理：通过 ParameterServer 配置优化器和信息矩阵参数
 *
 * 线程安全性：
 *   - 使用 optimization_mutex 保护优化过程中的数据访问
 *   - add_edge() 等写操作需要外部加锁
 *   - removeEdge() 使用 try_lock 避免阻塞 UI
 *
 * 继承关系：
 *   继承自 GraphSLAM，通过 using 声明暴露了基类的 graph、num_edges、
 *   num_vertices、save、set_solver 等关键接口。
 */
class InteractiveGraph : protected GraphSLAM {
public:
    InteractiveGraph();
    virtual ~InteractiveGraph();

    /**
     * @brief 从目录加载地图数据
     * @param directory 地图数据目录（应包含 graph.g2o 和关键帧子目录）
     * @param progress  进度回调接口
     * @return 加载成功返回 true，失败返回 false
     *
     * 加载流程：
     *   1. 加载 graph.g2o 文件
     *   2. 重新分配边 ID，标记所有边为 Original
     *   3. 加载关键帧数据
     *   4. 加载特殊节点（锚点等）
     */
    bool load_map_data(const std::string& directory, hdl_graph_slam::ProgressInterface& progress);

    /**
     * @brief 获取锚点节点的 ID
     * @return 锚点节点 ID，若不存在则返回 -1
     */
    long anchor_node_id() const;

    /**
     * @brief 在两个关键帧之间添加一条边（循环闭合约束）
     * @param key1              第一个关键帧
     * @param key2              第二个关键帧
     * @param relative_pose     相对位姿变换（从 key1 到 key2）
     * @param robust_kernel     鲁棒核函数类型（默认 "NONE"）
     * @param robust_kernel_delta 鲁棒核函数 delta 参数
     * @param source            边的来源类型（默认为 ManualLoop）
     * @return 创建的 g2o 边指针
     *
     * 添加流程：
     *   1. 使用信息矩阵计算器根据点云匹配计算信息矩阵
     *   2. 调用基类的 add_se3_edge 创建边
     *   3. 分配唯一 ID 并记录边来源
     *   4. 可选添加鲁棒核函数
     */
    g2o::EdgeSE3* add_edge(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2,
                           const Eigen::Isometry3d& relative_pose,
                           const std::string& robust_kernel = "NONE",
                           double robust_kernel_delta = 0.1,
                           EdgeSource source = EdgeSource::ManualLoop);

    /**
     * @brief 按 ID 删除一条边
     * @param edgeId 要删除的边 ID
     * @return 删除成功返回 true，失败返回 false
     *
     * 注意：使用 try_lock 尝试获取互斥锁，若后台优化正在进行则立即返回 false。
     */
    bool removeEdge(long edgeId);

    /**
     * @brief 获取指定边的来源类型
     * @param edge_id 边 ID
     * @return 边的来源枚举值，若未找到则返回 EdgeSource::Original
     */
    EdgeSource edge_source(long edge_id) const {
        auto it = edge_sources.find(edge_id);
        return (it != edge_sources.end()) ? it->second : EdgeSource::Original;
    }

    /**
     * @brief 执行图优化（前台同步）
     * @param num_iterations 优化迭代次数，负数则使用参数服务器中的默认值
     *
     * 优化流程：
     *   1. 更新锚点节点的估计值
     *   2. 重定向 cerr 输出以捕获优化日志
     *   3. 记录优化前的 chi2 值
     *   4. 执行图优化
     *   5. 记录优化后的 chi2 值和耗时
     */
    void optimize(int num_iterations = -1);

    /**
     * @brief 获取图的统计信息
     * @param update 是否强制更新缓存
     * @return 统计信息字符串（包含顶点数、边数、耗时、chi2 等）
     */
    std::string graph_statistics(bool update = false);

    /**
     * @brief 获取优化过程中的日志消息
     * @return 优化日志字符串
     */
    std::string optimization_messages() const;

    /**
     * @brief 将当前图数据导出到指定目录
     * @param directory 导出目标目录
     * @param progress  进度回调接口
     *
     * 导出内容包括：
     *   - graph.g2o：图文件
     *   - 各关键帧子目录（000000/、000001/ 等）
     *   - special_nodes.csv：特殊节点记录
     */
    void dump(const std::string& directory, hdl_graph_slam::ProgressInterface& progress);

    /**
     * @brief 保存全局地图点云
     * @param filename 输出 PCD 文件名
     * @param progress 进度回调接口
     * @return 保存成功返回 true
     *
     * 实现方式：将各关键帧的点云根据优化后的位姿变换到全局坐标系并拼接。
     */
    bool save_pointcloud(const std::string& filename, hdl_graph_slam::ProgressInterface& progress);

    /**
     * @brief 保存为 LVBA 格式（用于后端优化）
     * @param directory LVBA 输出目录的父目录
     * @param progress  进度回调接口
     * @return 保存成功返回 true
     *
     * LVBA 格式输出：
     *   - 在 <directory>/../all_pcd_body/ 下生成数据
     *   - lidar_poses.txt：TUM 格式的位姿文件
     *   - 命名如 "0.pcd"、"1.pcd" 等按时间排序的点云文件
     */
    bool saveLVBA(const std::string& directory, hdl_graph_slam::ProgressInterface& progress);

    // 暴露基类的关键接口
    using GraphSLAM::graph;           ///< g2o 图对象
    using GraphSLAM::num_edges;       ///< 边的数量查询
    using GraphSLAM::num_vertices;    ///< 顶点数量查询
    using GraphSLAM::save;            ///< 保存图文件
    using GraphSLAM::set_solver;      ///< 设置优化求解器类型

private:
    /**
     * @brief 加载特殊节点（锚点）
     * @param directory 地图数据目录
     * @param progress  进度回调接口
     * @return 加载成功返回 true
     *
     * 从 special_nodes.csv 加载锚点信息，若文件不存在或锚点无效则自动创建新锚点。
     */
    bool load_special_nodes(const std::string& directory, hdl_graph_slam::ProgressInterface& progress);

    /**
     * @brief 加载所有关键帧
     * @param directory 地图数据目录
     * @param progress  进度回调接口
     * @return 加载成功返回 true
     *
     * 遍历目录下的 000000/、000001/ 等子目录，依次加载 InteractiveKeyFrame。
     */
    bool load_keyframes(const std::string& directory, hdl_graph_slam::ProgressInterface& progress);

private:
    g2o::VertexSE3* anchor_node;  ///< 锚点节点（固定节点，防止图优化漂移）
    g2o::EdgeSE3* anchor_edge;    ///< 锚点与首个关键帧之间的约束边
    long edge_id_gen;             ///< 边 ID 自动生成器（自增计数器）

public:
    mutable std::mutex optimization_mutex;  ///< 优化互斥锁（用于线程间同步）

    std::string graph_stats;                 ///< 图统计信息缓存字符串
    std::stringstream optimization_stream;   ///< 优化日志输出流

    ParameterServer params;  ///< 参数服务器（存储各种可配置参数）

    int iterations;           ///< 最近一次优化的实际迭代次数
    double chi2_before;       ///< 优化前的卡方值
    double chi2_after;        ///< 优化后的卡方值
    double elapsed_time_msec; ///< 最近一次优化的耗时（毫秒）

    std::unordered_map<long, InteractiveKeyFrame::Ptr> keyframes;  ///< 关键帧字典（ID -> 关键帧）
    std::unordered_map<long, EdgeSource> edge_sources;             ///< 边来源字典（ID -> 来源类型）
    std::unique_ptr<InformationMatrixCalculator> inf_calclator;    ///< 信息矩阵计算器
};

}  // namespace hdl_graph_slam
