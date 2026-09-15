/**
 * @file interactive_graph.cpp
 * @brief 交互式图（InteractiveGraph）的实现
 *
 * 该文件实现了 InteractiveGraph 的全部核心功能，包括：
 *   1. 地图数据的加载与导出
 *   2. 关键帧和特殊节点（锚点）的加载
 *   3. 边的添加与删除
 *   4. 图优化（前台同步与后台异步）
 *   5. 图统计信息收集
 *   6. 全局地图点云导出
 *   7. LVBA 格式数据导出
 *
 * 线程安全设计：
 *   - optimization_mutex 保护优化期间的数据访问
 *   - removeEdge() 使用 try_lock 避免阻塞 UI 线程
 *   - optimize() 的前台版本直接使用，后台版本通过独立线程执行
 */
#include "data/hdl_graph_slam/interactive_graph.hpp"

#include <algorithm>
#include <chrono>
#include <fstream>
#include <boost/filesystem.hpp>

#include <g2o/core/factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>

namespace hdl_graph_slam {

/**
 * @brief 构造函数：初始化交互式图
 *
 * 初始化流程：
 *   1. 调用基类 GraphSLAM 构造函数，指定求解器类型 "lm_var_cholmod"
 *   2. 创建信息矩阵计算器并加载参数
 *   3. 初始化边 ID 生成器为 0
 *   4. 锚点节点和锚点边初始化为 nullptr
 */
InteractiveGraph::InteractiveGraph()
    : GraphSLAM("lm_var_cholmod"),
      iterations(0), chi2_before(0.0), chi2_after(0.0), elapsed_time_msec(0.0) {
    inf_calclator.reset(new InformationMatrixCalculator());
    inf_calclator->load(params);
    edge_id_gen = 0;
    anchor_node = nullptr;
    anchor_edge = nullptr;
}

InteractiveGraph::~InteractiveGraph() = default;

/**
 * @brief 从目录加载完整的地图数据
 * @param directory 地图数据目录路径
 * @param progress  进度回调接口
 * @return 加载成功返回 true
 *
 * 加载流程：
 *   1. 加载 graph.g2o 图文件
 *   2. 重新分配所有边的 ID（从 0 开始自增），标记为 Original
 *   3. 加载所有关键帧
 *   4. 加载特殊节点（锚点等）
 */
bool InteractiveGraph::load_map_data(const std::string& directory,
                                      hdl_graph_slam::ProgressInterface& progress) {
    progress.set_title_fmt("progress.opening", directory);
    progress.set_text("loading graph");
    // 第1步：加载 g2o 图文件
    if (!load(directory + "/graph.g2o")) {
        return false;
    }

    // 第2步：重新分配边 ID，将所有已加载的边标记为 Original
    edge_id_gen = 0;
    edge_sources.clear();
    for (auto& edge : graph->edges()) {
        edge->setId(edge_id_gen);
        edge_sources[edge_id_gen] = EdgeSource::Original;
        edge_id_gen++;
    }

    progress.increment();
    progress.set_text("loading keyframes");
    // 第3步：加载关键帧数据
    if (!load_keyframes(directory, progress)) {
        return false;
    }

    // 第4步：加载特殊节点
    if (!load_special_nodes(directory, progress)) {
        return false;
    }

    return true;
}

/**
 * @brief 加载特殊节点（锚点）
 * @param directory 地图数据目录
 * @param progress  进度回调接口
 * @return 加载成功返回 true
 *
 * 从 special_nodes.csv 文件中读取锚点信息。
 * CSV 格式：每行一个标签 + 值，如 "anchor_node 123"
 *
 * 如果锚点无效（节点在图中不存在、节点无边、边类型不匹配），
 * 或特殊节点文件不存在，则自动创建一个新锚点：
 *   1. 找到 ID 最小的关键帧作为首个关键帧
 *   2. 在该关键帧位置创建新的锚点节点
 *   3. 添加锚点与首个关键帧之间的约束边
 * 锚点节点被固定（setFixed(true)），防止图优化整体漂移。
 */
bool InteractiveGraph::load_special_nodes(const std::string& directory,
                                           hdl_graph_slam::ProgressInterface& progress) {
    std::ifstream ifs(directory + "/special_nodes.csv");
    if (ifs) {
        while (!ifs.eof()) {
            std::string line;
            std::getline(ifs, line);

            if (line.empty()) {
                continue;
            }

            std::stringstream sst(line);
            std::string tag;
            sst >> tag;

            if (tag == "anchor_node") {
                long anchor_node_id = -1;
                sst >> anchor_node_id;

                if (anchor_node_id < 0) {
                    continue;
                }

                // CSV 中的锚点可能引用 graph.g2o 中不存在的节点
                // （例如 LVBA 导出新 graph.g2o 时未包含锚点）
                // 此时优雅降级：发出警告并跳过，后续自动创建新锚点
                g2o::VertexSE3* candidate =
                    dynamic_cast<g2o::VertexSE3*>(graph->vertex(anchor_node_id));
                if (candidate == nullptr) {
                    std::cerr << "[load_special_nodes] CSV anchor node "
                              << anchor_node_id
                              << " not found in graph, will create new anchor"
                              << std::endl;
                    break;
                }
                if (candidate->edges().empty()) {
                    std::cerr << "[load_special_nodes] CSV anchor node "
                              << anchor_node_id
                              << " has no edges, will create new anchor"
                              << std::endl;
                    break;
                }

                g2o::EdgeSE3* candidate_edge =
                    dynamic_cast<g2o::EdgeSE3*>(*candidate->edges().begin());
                if (candidate_edge == nullptr) {
                    std::cerr << "[load_special_nodes] CSV anchor edge not "
                                 "EdgeSE3, will create new anchor"
                              << std::endl;
                    break;
                }

                // 所有验证通过，使用候选锚点
                anchor_node = candidate;
                anchor_edge = candidate_edge;
            }
        }
    }

    // 如果未能加载锚点，则自动创建一个新的锚点
    if (anchor_node == nullptr) {
        std::cout << "create new anchor" << std::endl;
        using ID_Keyframe = std::pair<long, InteractiveKeyFrame::Ptr>;
        // 找到 ID 最小的关键帧作为首个关键帧
        auto first_keyframe = std::min_element(
            keyframes.begin(), keyframes.end(),
            [=](const ID_Keyframe& lhs, const ID_Keyframe& rhs) {
                return lhs.first < rhs.first;
            });

        if (first_keyframe == keyframes.end()) {
            std::cerr << "corrupted graph file!!" << std::endl;
            return false;
        }

        // 在首个关键帧位置创建锚点节点和约束边
        anchor_node = add_se3_node(first_keyframe->second->node->estimate());
        anchor_edge = add_se3_edge(anchor_node, first_keyframe->second->node,
                                   Eigen::Isometry3d::Identity(),
                                   Eigen::MatrixXd::Identity(6, 6) * 0.1);
    }

    std::cout << "anchor_node:" << anchor_node->id() << std::endl;
    std::cout << "anchor_edge:" << anchor_edge->vertices()[0]->id()
              << " - " << anchor_edge->vertices()[1]->id() << std::endl;

    // 固定锚点节点，使其在图优化中保持不变
    anchor_node->setFixed(true);
    return true;
}

/**
 * @brief 加载所有关键帧
 * @param directory 地图数据目录
 * @param progress  进度回调接口
 * @return 加载成功返回 true
 *
 * 遍历目录下的 000000/、000001/ … 子目录，依次创建 InteractiveKeyFrame。
 * 子目录名称格式为 6 位数字（如 000000、000001）。
 * 遇到非目录项时停止遍历。
 */
bool InteractiveGraph::load_keyframes(const std::string& directory,
                                       hdl_graph_slam::ProgressInterface& progress) {
    progress.set_maximum(graph->vertices().size());
    for (int i = 0;; i++) {
        // 构造子目录路径：如 "graph_dir/000000"
        std::string keyframe_dir = (boost::format("%s/%06d") % directory % i).str();
        if (!boost::filesystem::is_directory(keyframe_dir)) {
            break;  // 目录不存在时停止遍历
        }

        // 创建交互式关键帧对象
        InteractiveKeyFrame::Ptr keyframe =
            std::make_shared<InteractiveKeyFrame>(keyframe_dir, graph.get());
        if (!keyframe->node) {
            std::cerr << "error : failed to load keyframe!!" << std::endl;
            std::cerr << "      : " << keyframe_dir << std::endl;
        } else {
            // 以关键帧 ID 为键存入字典
            keyframes[keyframe->id()] = keyframe;
            progress.increment();
        }
    }

    return true;
}

/**
 * @brief 按 ID 删除边
 * @param edgeId 要删除的边 ID
 * @return 删除成功返回 true，失败（边不存在或优化进行中）返回 false
 *
 * 使用 try_lock 尝试获取优化互斥锁，避免阻塞 UI 线程。
 * 如果后台优化正在进行，立即返回 false，调用者可稍后重试。
 */
bool InteractiveGraph::removeEdge(long edgeId) {
    // 尝试获取锁，不阻塞
    std::unique_lock<std::mutex> lock(optimization_mutex, std::try_to_lock);
    if (!lock.owns_lock()) {
        return false;  // 优化正在进行，调用者应稍后重试
    }
    g2o::SparseOptimizer* g = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
    if (!g) return false;

    // 遍历所有边，查找匹配 ID 的 EdgeSE3 边
    for (auto* edge : g->edges()) {
        auto* se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
        if (!se3) continue;
        if (static_cast<long>(se3->id()) == edgeId) {
            return g->removeEdge(se3);  // 删除边
        }
    }
    return false;  // 未找到匹配的边
}

/**
 * @brief 获取锚点节点 ID
 * @return 锚点节点 ID，若锚点不存在则返回 -1
 */
long InteractiveGraph::anchor_node_id() const {
    return anchor_node ? anchor_node->id() : -1;
}

/**
 * @brief 在两个关键帧之间添加一条边
 * @param key1              第一个关键帧
 * @param key2              第二个关键帧
 * @param relative_pose     从 key1 到 key2 的相对位姿变换
 * @param robust_kernel     鲁棒核函数类型（如 "Huber"、"Cauchy" 等）
 * @param robust_kernel_delta 鲁棒核函数 delta 参数
 * @param source            边的来源类型
 * @return 创建的 g2o::EdgeSE3 指针
 *
 * 实现步骤：
 *   1. 使用信息矩阵计算器，根据两点云及其相对位姿计算信息矩阵
 *   2. 调用基类的 add_se3_edge() 创建边
 *   3. 分配全局唯一边 ID
 *   4. 记录边的来源
 *   5. 如果需要，添加鲁棒核函数
 */
g2o::EdgeSE3* InteractiveGraph::add_edge(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2,
                                          const Eigen::Isometry3d& relative_pose,
                                          const std::string& robust_kernel,
                                          double robust_kernel_delta,
                                          EdgeSource source) {
    // 计算信息矩阵（基于点云匹配质量）
    Eigen::MatrixXd inf = inf_calclator->calc_information_matrix(key1->cloud, key2->cloud,
                                                                  relative_pose);
    // 创建 SE3 约束边
    g2o::EdgeSE3* edge = add_se3_edge(key1->node, key2->node, relative_pose, inf);
    long eid = edge_id_gen++;  // 分配唯一 ID
    edge->setId(eid);
    edge_sources[eid] = source;  // 记录来源

    // 可选添加鲁棒核函数，降低外点的影响
    if (robust_kernel != "NONE") {
        add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
    }

    return edge;
}

/**
 * @brief 执行图优化（前台同步）
 * @param num_iterations 优化迭代次数，负数使用参数服务器默认值（默认 64）
 *
 * 优化流程：
 *   1. 更新锚点节点的估计值为首个关键帧的当前估计值
 *   2. 清空优化日志流
 *   3. 重定向 cerr 到 optimization_stream 以捕获优化过程日志
 *   4. 获取优化前的卡方值（chi2）
 *   5. 调用基类的 optimize() 执行图优化
 *   6. 获取优化后的卡方值
 *   7. 计算耗时
 *   8. 恢复 cerr 重定向
 */
void InteractiveGraph::optimize(int num_iterations) {
    // 更新锚点节点的估计值
    if (anchor_node) {
        g2o::VertexSE3* first_keyframe =
            dynamic_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1]);
        if (first_keyframe == nullptr) {
            std::cerr << "failed to cast the first keyframe to VertexSE3" << std::endl;
        } else {
            anchor_node->setEstimate(first_keyframe->estimate());
        }
    }

    // 清空之前的优化日志
    optimization_stream.str("");
    optimization_stream.clear();

    // 重定向 cerr，捕获优化过程的日志输出
    std::streambuf* cerr_buf = std::cerr.rdbuf();
    std::cerr.rdbuf(optimization_stream.rdbuf());

    g2o::SparseOptimizer* g = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
    auto t1 = std::chrono::high_resolution_clock::now();

    // 确定迭代次数
    if (num_iterations < 0) {
        num_iterations = params.param<int>("g2o_solver_num_iterations", 64);
    }

    // 执行优化并记录统计信息
    chi2_before = g->chi2();
    iterations = GraphSLAM::optimize(num_iterations);
    chi2_after = g->chi2();

    auto t2 = std::chrono::high_resolution_clock::now();
    elapsed_time_msec =
        std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1000000.0;

    // 恢复 cerr 重定向
    std::cerr.rdbuf(cerr_buf);
}

/**
 * @brief 获取图统计信息
 * @param update 是否强制更新缓存
 * @return 包含顶点数、边数、优化耗时、chi2 变化和迭代次数的字符串
 *
 * 使用 try_lock 尝试获取优化互斥锁，如果优化正在进行则返回上一次的缓存值。
 */
std::string InteractiveGraph::graph_statistics(bool update) {
    if (optimization_mutex.try_lock()) {
        std::stringstream sst;
        sst << "Graph\n";
        sst << boost::format("# vertices: %d") % num_vertices() << "\n";
        sst << boost::format("# edges: %d") % num_edges() << "\n";
        sst << boost::format("time: %.1f[msec]") % elapsed_time_msec << "\n";
        sst << boost::format("chi2: %.3f -> %.3f") % chi2_before % chi2_after << "\n";
        sst << boost::format("iterations: %d") % iterations;

        graph_stats = sst.str();
        optimization_mutex.unlock();
    }

    return graph_stats;
}

/**
 * @brief 获取优化过程中的日志消息
 * @return 优化日志字符串流内容
 */
std::string InteractiveGraph::optimization_messages() const {
    return optimization_stream.str();
}

/**
 * @brief 将当前图数据导出到指定目录
 * @param directory 导出目标目录
 * @param progress  进度回调接口
 *
 * 导出结构：
 *   - graph.g2o：位姿图文件
 *   - 000000/、000001/ …：各关键帧的子目录（包含 data、cloud.pcd 等）
 *   - special_nodes.csv：特殊节点记录文件
 */
void InteractiveGraph::dump(const std::string& directory, hdl_graph_slam::ProgressInterface& progress) {
    progress.set_maximum(keyframes.size());
    progress.set_text("saving graph");
    progress.increment();

    // 保存图文件
    save(directory + "/graph.g2o");

    progress.set_text("saving keyframes");

    // 保存每个关键帧的数据
    int keyframe_id = 0;
    for (const auto& keyframe : keyframes) {
        progress.increment();
        std::stringstream sst;
        sst << boost::format("%s/%06d") % directory % (keyframe_id++);
        keyframe.second->save(sst.str());
    }

    // 保存特殊节点信息
    std::ofstream ofs(directory + "/special_nodes.csv");
    ofs << "anchor_node " << (anchor_node != nullptr ? anchor_node->id() : -1) << std::endl;
    ofs << "anchor_edge " << -1 << std::endl;
    ofs << "floor_node " << -1 << std::endl;
}

/**
 * @brief 保存全局地图点云
 * @param filename 输出的 PCD 文件路径
 * @param progress 进度回调接口
 * @return 保存成功返回 true
 *
 * 实现步骤：
 *   1. 将各关键帧的点云根据优化后的位姿变换到全局坐标系
 *   2. 将所有变换后的点云拼接为一个整体
 *   3. 保存为二进制 PCD 文件
 */
bool InteractiveGraph::save_pointcloud(const std::string& filename,
                                        hdl_graph_slam::ProgressInterface& progress) {
    progress.set_maximum(keyframes.size() + 1);
    progress.set_text("accumulate points");

    // 逐帧变换并拼接点云
    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto& keyframe : keyframes) {
        progress.increment();

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
        // 根据 g2o 顶点优化后的位姿变换点云到全局坐标系
        pcl::transformPointCloud(*keyframe.second->cloud, *transformed,
                                 keyframe.second->node->estimate().cast<float>());

        // 将变换后的点云追加到累积点云中
        std::copy(transformed->begin(), transformed->end(),
                  std::back_inserter(accumulated->points));
    }

    accumulated->is_dense = false;
    accumulated->width = accumulated->size();
    accumulated->height = 1;

    progress.set_text("saving pcd");
    progress.increment();

    return pcl::io::savePCDFileBinary(filename, *accumulated);
}

/**
 * @brief 保存为 LVBA 格式（Large-scale Visual-BA 兼容格式）
 * @param directory LVBA 输出目录的父目录
 * @param progress  进度回调接口
 * @return 保存成功返回 true
 *
 * LVBA 格式说明：
 *   输出到 <dir>/../all_pcd_body/ 目录下
 *
 * 输出内容：
 *   - lidar_poses.txt：TUM 格式的轨迹文件
 *     "timestamp tx ty tz qx qy qz qw"
 *   - 序列化的 PCD 文件（命名如 "0.pcd"、"1.pcd" …）
 *     每个 PCD 文件对应一个关键帧的 IMU 机体坐标系点云（不做额外变换）
 *
 * 排序规则：
 *   按时间戳（stamp_nsec）升序排列，时间戳相同时按 g2o 节点 ID 排序。
 */
bool InteractiveGraph::saveLVBA(const std::string& directory,
                                hdl_graph_slam::ProgressInterface& progress) {
    namespace fs = boost::filesystem;

    // LVBA 输出目录：<directory>/../all_pcd_body/
    fs::path lvba_dir = fs::path(directory).parent_path() / "all_pcd_body";
    try {
        if (!fs::is_directory(lvba_dir)) {
            fs::create_directory(lvba_dir);
        }
    } catch (const std::exception& e) {
        std::cerr << "[saveLVBA] Failed to create " << lvba_dir
                  << ": " << e.what() << std::endl;
        return false;
    }

    // 收集含有有效点云和节点的关键帧
    std::vector<InteractiveKeyFrame::Ptr> sorted;
    sorted.reserve(keyframes.size());
    for (const auto& kv : keyframes) {
        const auto& kf = kv.second;
        if (kf->cloud && !kf->cloud->empty() && kf->node) {
            sorted.push_back(kf);
        }
    }

    if (sorted.empty()) {
        std::cerr << "[saveLVBA] No keyframes with cloud data" << std::endl;
        return false;
    }

    // 按时间戳升序排序，时间戳相同时按 g2o 节点 ID 排序
    std::sort(sorted.begin(), sorted.end(),
        [](const InteractiveKeyFrame::Ptr& a, const InteractiveKeyFrame::Ptr& b) {
            if (a->stamp_nsec != b->stamp_nsec)
                return a->stamp_nsec < b->stamp_nsec;
            return a->id() < b->id();
        });

    // 写入 lidar_poses.txt（TUM 格式）
    std::string poses_path = (lvba_dir / "lidar_poses.txt").string();
    std::ofstream ofs(poses_path);
    if (!ofs) {
        std::cerr << "[saveLVBA] Failed to open " << poses_path << std::endl;
        return false;
    }

    progress.set_maximum(sorted.size());
    progress.set_text("saving LVBA format");
    progress.increment();

    bool all_ok = true;
    for (size_t seq_idx = 0; seq_idx < sorted.size(); ++seq_idx) {
        const auto& kf = sorted[seq_idx];
        progress.increment();

        // 从 g2o 顶点估计值获取位姿（权威数据源）
        const Eigen::Isometry3d& pose = kf->node->estimate();
        Eigen::Quaterniond q(pose.linear());
        Eigen::Vector3d t = pose.translation();

        // 时间戳：纳秒转换为秒
        double timestamp = static_cast<double>(kf->stamp_nsec) / 1e9;

        // TUM 格式：timestamp tx ty tz qx qy qz qw
        ofs << boost::format("%.6f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n")
                   % timestamp % t.x() % t.y() % t.z()
                   % q.x() % q.y() % q.z() % q.w();

        // 写入 PCD 文件（IMU 机体坐标系点云，不做变换）
        std::string pcd_name = (boost::format("%.6f.pcd") %
                                static_cast<double>(seq_idx)).str();
        std::string pcd_path = (lvba_dir / pcd_name).string();
        if (pcl::io::savePCDFileBinary(pcd_path, *kf->cloud) < 0) {
            std::cerr << "[saveLVBA] Failed to write " << pcd_path << std::endl;
            all_ok = false;
        }
    }

    ofs.close();
    return all_ok;
}

}  // namespace hdl_graph_slam
