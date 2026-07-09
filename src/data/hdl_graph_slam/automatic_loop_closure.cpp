/**
 * @file automatic_loop_closure.cpp
 * @brief 自动闭环检测模块的实现
 *
 * 该文件实现了完整的自动闭环检测流程：
 *
 * 主循环流程（loop_detection）：
 *   1. 选择源关键帧（顺序或随机）
 *   2. BFS 搜索：沿 g2o 边计算图距离，查找闭环候选
 *   3. ICP/GICP/NDT 配准验证：将候选点云与源点云对齐
 *   4. 计算适应度分数，评估匹配质量
 *   5. 对匹配成功的候选插入闭环边（带可选鲁棒核函数）
 *   6. 可选执行全局图优化
 *
 * BFS 搜索策略：
 *   从源关键帧出发，沿 g2o 边广度优先遍历，计算每个可达关键帧的
 *   累积空间距离。候选需要满足：
 *     - 与源的空间距离小于 distance_thresh
 *     - 沿图的累积距离大于 accum_distance_thresh（避免相邻帧误检）
 *     - 未与源直接连接（排除已有边的邻居）
 */
#include "data/hdl_graph_slam/automatic_loop_closure.hpp"
#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "data/hdl_graph_slam/interactive_keyframe.hpp"
#include "data/hdl_graph_slam/keyframe.hpp"
#include "data/hdl_graph_slam/information_matrix_calculator.hpp"

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include <pcl/point_cloud.h>
#include <pcl/registration/registration.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

namespace hdl_graph_slam {

// ── 鲁棒核函数名称表（必须与 g2o::RobustKernelFactory 注册顺序一致） ──────────
static const char* kKernelNames[] = {
    "NONE",          // 0：无核函数（标准二次代价函数）
    "Huber",         // 1：Huber 核函数
    "Cauchy",        // 2：Cauchy 核函数
    "DCS",           // 3：DCS（Davies-Crawford-Stewart）核函数
    "Fair",          // 4：Fair 核函数
    "GemanMcClure",  // 5：Geman-McClure 核函数
    "PseudoHuber",   // 6：伪 Huber 核函数
    "Saturated",     // 7：饱和核函数
    "Tukey",         // 8：Tukey 双权核函数
    "Welsch"         // 9：Welsch 核函数
};

static constexpr int kNumKernels = sizeof(kKernelNames) / sizeof(kKernelNames[0]);

/**
 * @brief 根据类型索引获取鲁棒核函数名称
 * @param type 核函数类型索引
 * @return 核函数名称字符串，无效索引返回 "NONE"
 */
const char* AutomaticLoopClosure::kernel_name(int type) {
    if (type < 0 || type >= kNumKernels) return kKernelNames[0];
    return kKernelNames[type];
}

// ── 构造 / 析构 ───────────────────────────────────────────────────────────────

AutomaticLoopClosure::AutomaticLoopClosure(InteractiveGraph* graph)
    : m_graph(graph) {}

AutomaticLoopClosure::~AutomaticLoopClosure() {
    stop();  // 确保线程在析构时结束
}

// ── 生命周期管理 ────────────────────────────────────────────────────────────────

/**
 * @brief 启动后台闭环检测线程
 *
 * 如果已在运行则直接返回。创建新线程执行 loop_detection()。
 */
void AutomaticLoopClosure::start() {
    if (m_running.load(std::memory_order_acquire)) return;

    m_running.store(true, std::memory_order_release);
    m_thread = std::thread(&AutomaticLoopClosure::loop_detection, this);
}

/**
 * @brief 停止后台闭环检测线程
 *
 * 设置原子标志 m_running 为 false，等待线程结束后返回。
 */
void AutomaticLoopClosure::stop() {
    m_running.store(false, std::memory_order_release);
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

// ── 线程安全状态快照 ──────────────────────────────────────────────────────────

/**
 * @brief 获取当前状态的线程安全快照
 * @return 当前 Status 结构体的拷贝
 */
AutomaticLoopClosure::Status AutomaticLoopClosure::snapshot() const {
    std::lock_guard<std::mutex> lock(m_status_mutex);
    return m_status;
}

/**
 * @brief 更新状态（源 ID 和候选列表）
 * @param source_id  当前处理的源关键帧 ID
 * @param candidates 候选关键帧 ID 列表
 */
void AutomaticLoopClosure::update_status(long source_id,
                                          const std::vector<long>& candidates) {
    std::lock_guard<std::mutex> lock(m_status_mutex);
    m_status.running           = true;
    m_status.current_source_id = source_id;
    m_status.candidate_ids     = candidates;
}

/**
 * @brief 更新最近一次匹配的结果
 * @param begin_id 源关键帧 ID
 * @param end_id   目标关键帧 ID
 * @param fitness  匹配适应度分数
 */
void AutomaticLoopClosure::update_last_match(long begin_id, long end_id,
                                              double fitness) {
    std::lock_guard<std::mutex> lock(m_status_mutex);
    m_status.last_begin_id     = begin_id;
    m_status.last_end_id       = end_id;
    m_status.last_fitness_score = fitness;
}

// ── 排序后的关键帧 ID 缓存 ───────────────────────────────────────────────────

/**
 * @brief 刷新关键帧 ID 排序列表缓存
 *
 * 将 graph->keyframes 中的所有 ID 提取到 m_sorted_ids 并排序，
 * 用于在顺序搜索模式中按 ID 顺序遍历。
 */
void AutomaticLoopClosure::refresh_sorted_ids() {
    m_sorted_ids.clear();
    m_sorted_ids.reserve(m_graph->keyframes.size());
    for (const auto& kv : m_graph->keyframes) {
        m_sorted_ids.push_back(kv.first);
    }
    std::sort(m_sorted_ids.begin(), m_sorted_ids.end());
}

// ── BFS 候选搜索 ─────────────────────────────────────────────────────────────

/**
 * @brief 通过 BFS 查找指定源关键帧的闭环候选
 * @param source_id 源关键帧的 ID
 * @return 候选关键帧 ID 列表
 *
 * 搜索策略：
 *   步骤 1：从源关键帧出发，沿 g2o 边执行 BFS 遍历
 *           计算每个可达关键帧沿图的累积空间距离
 *   步骤 2：排除与源关键帧直接相邻（已有边连接）的顶点
 *   步骤 3：过滤候选，需同时满足：
 *     - 空间距离 < distance_thresh
 *     - 图累积距离 > accum_distance_thresh（或不可达）
 *     - 不是源自身
 *     - 未与源直接连接
 *
 * BFS 说明：
 *   使用双端队列（deque）实现 BFS，每次从队列头部取出一个顶点，
 *   遍历其所有 SE3 边，将邻接顶点加入队列。
 *   使用 accum_distances 字典记录到每个顶点的最小图距离。
 */
std::vector<long> AutomaticLoopClosure::find_loop_candidates(long source_id) {
    auto src_it = m_graph->keyframes.find(source_id);
    if (src_it == m_graph->keyframes.end()) {
        return {};
    }

    auto& source_kf = src_it->second;
    g2o::VertexSE3* source_node = source_kf->node;
    Eigen::Vector3d source_pos  = source_node->estimate().translation();

    // ── 步骤 1：沿 g2o 边 BFS 遍历，计算累积图距离 ──
    std::unordered_map<long, float> accum_distances;
    accum_distances[source_id] = 0.0f;

    // BFS 队列元素：关键帧 ID 和对应的顶点指针
    struct QueueItem {
        long id;
        g2o::VertexSE3* node;
    };
    std::deque<QueueItem> search_queue;
    search_queue.push_back({source_id, source_node});

    while (!search_queue.empty()) {
        // 从队列头部取出当前元素
        auto [target_id, target_node] = search_queue.front();
        float target_accum = accum_distances[target_id];
        Eigen::Vector3d target_pos = target_node->estimate().translation();
        search_queue.pop_front();

        // 遍历当前顶点的所有边
        for (auto* edge_ptr : target_node->edges()) {
            g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(edge_ptr);
            if (!edge) continue;  // 忽略非 SE3 边

            g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[0]);
            g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[1]);
            if (!v1 || !v2) continue;

            // 确定邻接顶点 ID
            long next_id = (v1->id() == target_id) ? v2->id() : v1->id();

            // 只遍历关键帧对应的顶点（跳过锚点等非关键帧顶点）
            if (m_graph->keyframes.find(next_id) == m_graph->keyframes.end())
                continue;

            g2o::VertexSE3* next_node = (v1->id() == target_id) ? v2 : v1;
            // 计算到邻接顶点的累积距离 = 当前距离 + 两点之间的空间距离
            float delta = static_cast<float>(
                (next_node->estimate().translation() - target_pos).norm());
            float accum = target_accum + delta;

            // 如果该顶点尚未访问，或找到了更短的路径，则更新并加入队列
            auto found = accum_distances.find(next_id);
            if (found == accum_distances.end() || found->second > accum) {
                accum_distances[next_id] = accum;
                search_queue.push_back({next_id, next_node});
            }
        }
    }

    // ── 步骤 2：排除与源顶点直接相连的关键帧（已有边） ────
    std::unordered_set<long> excluded;
    for (auto* edge_ptr : source_node->edges()) {
        g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(edge_ptr);
        if (!edge) continue;
        for (size_t i = 0; i < edge->vertices().size(); ++i) {
            if (edge->vertices()[i]) {
                excluded.insert(edge->vertices()[i]->id());
            }
        }
    }

    // ── 步骤 3：过滤候选 ‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐‐
    std::vector<long> candidates;
    for (const auto& [cand_id, cand_kf] : m_graph->keyframes) {
        if (cand_id == source_id)          continue;   // 排除自身
        if (excluded.count(cand_id))       continue;   // 排除已直接相连的关键帧

        Eigen::Vector3d cand_pos = cand_kf->node->estimate().translation();
        double dist = (cand_pos - source_pos).norm();

        if (dist > m_distance_thresh) continue;        // 空间距离过远，排除

        auto found = accum_distances.find(cand_id);
        if (found != accum_distances.end()) {
            // 在图中可达 → 需要一定的图距离（避免相邻帧误检）
            if (found->second <= m_accum_distance_thresh) continue;
        }
        // 图中不可达（孤立组件）但空间上接近 → 仍作为候选

        candidates.push_back(cand_id);
    }

    return candidates;
}

// ── 后台检测线程主循环 ────────────────────────────────────────────────────────

/**
 * @brief 闭环检测主循环（在后台线程中运行）
 *
 * 主循环迭代流程：
 *   1. 等待至少 2 个关键帧存在
 *   2. 选择源关键帧（随机或顺序模式）
 *   3. 通过 BFS 查找闭环候选
 *   4. 对每个候选执行点云配准验证
 *   5. 计算适应度分数，判断是否接受匹配
 *   6. 对接受的匹配插入闭环边
 *   7. 可选执行全局图优化
 *   8. 推进到下一个关键帧索引
 *   9. 短暂休眠避免忙等待
 *
 * 每次迭代消耗约 50ms ~ 500ms（取决于配准方法和候选数量）。
 */
void AutomaticLoopClosure::loop_detection() {
    using PointT = pcl::PointXYZI;

    // 初始化：缓存排序后的 ID 列表用于顺序模式
    refresh_sorted_ids();

    // ── 创建配准器实例，在所有迭代中复用 ──────────────────────
    pcl::Registration<PointT, PointT>::Ptr registration;
    try {
        registration = m_reg_methods.method();
    } catch (const std::runtime_error&) {
        // 配准方法不可用（如缺少 OMP/FAST_GICP 支持），标记停止
        std::lock_guard<std::mutex> lock(m_status_mutex);
        m_status.running = false;
        return;
    }

    const char* method_name = m_reg_methods.method_names()[m_reg_methods.get_method_index()];
    std::cerr << "[auto-loop] using " << method_name << std::endl;

    // 主循环：持续运行直到被外部停止
    while (m_running.load(std::memory_order_acquire)) {
        // ── 检查：至少需要 2 个关键帧 ──────────────────────────
        if (m_graph->keyframes.size() < 2) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        // ── 步骤 1：如果关键帧数量变化，刷新排序缓存 ────────────
        if (m_sorted_ids.size() != m_graph->keyframes.size()) {
            refresh_sorted_ids();
        }

        // ── 步骤 2：选择源关键帧 ────────────────────────────────
        size_t n = m_sorted_ids.size();
        size_t idx;

        if (m_search_method == RANDOM) {
            // 随机模式：从所有关键帧中随机选择
            idx = static_cast<size_t>(std::rand()) % n;
            m_current_index = static_cast<int>(idx);
        } else {
            // 顺序模式：按索引顺序遍历
            idx = static_cast<size_t>(m_current_index) % n;
        }

        long source_id = m_sorted_ids[idx];

        auto src_it = m_graph->keyframes.find(source_id);
        if (src_it == m_graph->keyframes.end()) {
            m_current_index++;
            continue;
        }
        auto& source_kf = src_it->second;
        if (!source_kf->cloud || source_kf->cloud->empty()) {
            m_current_index++;
            continue;
        }

        Eigen::Isometry3d source_pose = source_kf->node->estimate();

        // ── 步骤 3：通过 BFS 查找闭环候选 ──────────────────────
        auto candidates = find_loop_candidates(source_id);
        update_status(source_id, candidates);

        // ── 步骤 4：用点云配准验证每个候选 ──────────────────────
        registration->setInputTarget(source_kf->cloud);  // 源点云作为配准目标

        // 迭代计时
        auto t_iter_start = std::chrono::steady_clock::now();

        bool edge_inserted = false;

        for (long cand_id : candidates) {
            // 检查外部停止信号
            if (!m_running.load(std::memory_order_acquire)) break;

            auto cand_it = m_graph->keyframes.find(cand_id);
            if (cand_it == m_graph->keyframes.end()) continue;
            auto& cand_kf = cand_it->second;
            if (!cand_kf->cloud || cand_kf->cloud->empty()) continue;

            // 初始猜测：当前图优化位姿之间的相对变换
            Eigen::Isometry3d relative =
                source_pose.inverse() * cand_kf->node->estimate();

            registration->setInputSource(cand_kf->cloud);  // 候选点云作为配准源

            // ── 执行配准对齐 ──────────────────────────────────
            auto t_align_start = std::chrono::steady_clock::now();

            pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
            registration->align(*aligned, relative.matrix().cast<float>());

            auto t_align_end = std::chrono::steady_clock::now();
            double align_ms = std::chrono::duration<double, std::milli>(t_align_end - t_align_start).count();

            // 获取配准后的精炼变换
            relative.matrix() =
                registration->getFinalTransformation().cast<double>();

            // ── 计算适应度分数 ────────────────────────────────
            auto t_fit_start = std::chrono::steady_clock::now();

            double fitness =
                InformationMatrixCalculator::calc_fitness_score(
                    source_kf->cloud, cand_kf->cloud, relative,
                    m_fitness_score_max_range);

            auto t_fit_end = std::chrono::steady_clock::now();
            double fit_ms = std::chrono::duration<double, std::milli>(t_fit_end - t_fit_start).count();

            // 输出调试信息
            std::cerr << "[auto-loop] " << method_name
                      << " | src=" << source_id << " → cand=" << cand_id
                      << " | align=" << align_ms << " ms"
                      << " | fitness=" << fit_ms << " ms"
                      << " | score=" << fitness
                      << (fitness < m_fitness_score_thresh ? " ✓" : "")
                      << std::endl;

            update_last_match(source_id, cand_id, fitness);

            // ── 步骤 5：判断是否接受匹配 ──────────────────────
            if (fitness < static_cast<double>(m_fitness_score_thresh)) {
                // 适应度分数低于阈值 → 视为成功匹配
                m_graph->add_edge(source_kf, cand_kf, relative,
                                  kernel_name(m_kernel_type),
                                  static_cast<double>(m_kernel_delta),
                                  EdgeSource::AutoLoop);
                edge_inserted = true;

                {
                    std::lock_guard<std::mutex> lock(m_status_mutex);
                    m_status.edges_inserted++;
                }
            }
        }

        auto t_iter_end = std::chrono::steady_clock::now();
        double iter_ms = std::chrono::duration<double, std::milli>(t_iter_end - t_iter_start).count();
        std::cerr << "[auto-loop] " << method_name
                  << " | iteration done: " << candidates.size() << " candidates"
                  << " | total=" << iter_ms << " ms"
                  << (edge_inserted ? " | edge inserted!" : "")
                  << std::endl;

        // ── 步骤 5：可选的全局优化 ──────────────────────────────
        if (edge_inserted && m_optimize) {
            std::lock_guard<std::mutex> lock(m_graph->optimization_mutex);
            m_graph->optimize();
        }

        // ── 步骤 6：推进到下一个关键帧 ──────────────────────────
        m_current_index++;

        // 暂停一段时间，避免 CPU 忙等待
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // 循环结束，标记状态
    {
        std::lock_guard<std::mutex> lock(m_status_mutex);
        m_status.running = false;
    }
}

}  // namespace hdl_graph_slam
