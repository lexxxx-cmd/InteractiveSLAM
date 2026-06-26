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

// ── Robust kernel name table (must match g2o RobustKernelFactory order) ──────
static const char* kKernelNames[] = {
    "NONE",          // 0
    "Huber",         // 1
    "Cauchy",        // 2
    "DCS",           // 3
    "Fair",          // 4
    "GemanMcClure",  // 5
    "PseudoHuber",   // 6
    "Saturated",     // 7
    "Tukey",         // 8
    "Welsch"         // 9
};

static constexpr int kNumKernels = sizeof(kKernelNames) / sizeof(kKernelNames[0]);

const char* AutomaticLoopClosure::kernel_name(int type) {
    if (type < 0 || type >= kNumKernels) return kKernelNames[0];
    return kKernelNames[type];
}

// ── Construction / Destruction ───────────────────────────────────────────────

AutomaticLoopClosure::AutomaticLoopClosure(InteractiveGraph* graph)
    : m_graph(graph) {}

AutomaticLoopClosure::~AutomaticLoopClosure() {
    stop();
}

// ── Lifecycle ────────────────────────────────────────────────────────────────

void AutomaticLoopClosure::start() {
    if (m_running.load(std::memory_order_acquire)) return;

    m_running.store(true, std::memory_order_release);
    m_thread = std::thread(&AutomaticLoopClosure::loop_detection, this);
}

void AutomaticLoopClosure::stop() {
    m_running.store(false, std::memory_order_release);
    if (m_thread.joinable()) {
        m_thread.join();
    }
}

// ── Thread-safe status snapshot ──────────────────────────────────────────────

AutomaticLoopClosure::Status AutomaticLoopClosure::snapshot() const {
    std::lock_guard<std::mutex> lock(m_status_mutex);
    return m_status;
}

void AutomaticLoopClosure::update_status(long source_id,
                                          const std::vector<long>& candidates) {
    std::lock_guard<std::mutex> lock(m_status_mutex);
    m_status.running           = true;
    m_status.current_source_id = source_id;
    m_status.candidate_ids     = candidates;
}

void AutomaticLoopClosure::update_last_match(long begin_id, long end_id,
                                              double fitness) {
    std::lock_guard<std::mutex> lock(m_status_mutex);
    m_status.last_begin_id     = begin_id;
    m_status.last_end_id       = end_id;
    m_status.last_fitness_score = fitness;
}

// ── Sorted keyframe ID cache ─────────────────────────────────────────────────

void AutomaticLoopClosure::refresh_sorted_ids() {
    m_sorted_ids.clear();
    m_sorted_ids.reserve(m_graph->keyframes.size());
    for (const auto& kv : m_graph->keyframes) {
        m_sorted_ids.push_back(kv.first);
    }
    std::sort(m_sorted_ids.begin(), m_sorted_ids.end());
}

// ── BFS candidate search ─────────────────────────────────────────────────────

std::vector<long> AutomaticLoopClosure::find_loop_candidates(long source_id) {
    auto src_it = m_graph->keyframes.find(source_id);
    if (src_it == m_graph->keyframes.end()) {
        return {};
    }

    auto& source_kf = src_it->second;
    g2o::VertexSE3* source_node = source_kf->node;
    Eigen::Vector3d source_pos  = source_node->estimate().translation();

    // ── Step 1: BFS along g2o edges to compute accumulated graph distance ──
    std::unordered_map<long, float> accum_distances;
    accum_distances[source_id] = 0.0f;

    struct QueueItem {
        long id;
        g2o::VertexSE3* node;
    };
    std::deque<QueueItem> search_queue;
    search_queue.push_back({source_id, source_node});

    while (!search_queue.empty()) {
        auto [target_id, target_node] = search_queue.front();
        float target_accum = accum_distances[target_id];
        Eigen::Vector3d target_pos = target_node->estimate().translation();
        search_queue.pop_front();

        for (auto* edge_ptr : target_node->edges()) {
            g2o::EdgeSE3* edge = dynamic_cast<g2o::EdgeSE3*>(edge_ptr);
            if (!edge) continue;

            g2o::VertexSE3* v1 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[0]);
            g2o::VertexSE3* v2 = dynamic_cast<g2o::VertexSE3*>(edge->vertices()[1]);
            if (!v1 || !v2) continue;

            long next_id = (v1->id() == target_id) ? v2->id() : v1->id();

            // Only traverse through keyframe vertices
            if (m_graph->keyframes.find(next_id) == m_graph->keyframes.end())
                continue;

            g2o::VertexSE3* next_node = (v1->id() == target_id) ? v2 : v1;
            float delta = static_cast<float>(
                (next_node->estimate().translation() - target_pos).norm());
            float accum = target_accum + delta;

            auto found = accum_distances.find(next_id);
            if (found == accum_distances.end() || found->second > accum) {
                accum_distances[next_id] = accum;
                search_queue.push_back({next_id, next_node});
            }
        }
    }

    // ── Step 2: Exclude vertices already directly connected to source ────
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

    // ── Step 3: Filter candidates ────────────────────────────────────────
    std::vector<long> candidates;
    for (const auto& [cand_id, cand_kf] : m_graph->keyframes) {
        if (cand_id == source_id)          continue;   // self
        if (excluded.count(cand_id))       continue;   // already connected

        Eigen::Vector3d cand_pos = cand_kf->node->estimate().translation();
        double dist = (cand_pos - source_pos).norm();

        if (dist > m_distance_thresh) continue;        // too far in space

        auto found = accum_distances.find(cand_id);
        if (found != accum_distances.end()) {
            // Reachable via BFS — must be far enough in graph distance
            if (found->second <= m_accum_distance_thresh) continue;
        }
        // Unreachable (disjoint component) but spatially close → include

        candidates.push_back(cand_id);
    }

    return candidates;
}

// ── Background detection thread ──────────────────────────────────────────────

void AutomaticLoopClosure::loop_detection() {
    using PointT = pcl::PointXYZI;

    // Cache sorted IDs for SEQUENTIAL mode
    refresh_sorted_ids();

    // ── Create registration ONCE, reused across all iterations ──────
    pcl::Registration<PointT, PointT>::Ptr registration;
    try {
        registration = m_reg_methods.method();
    } catch (const std::runtime_error&) {
        std::lock_guard<std::mutex> lock(m_status_mutex);
        m_status.running = false;
        return;
    }

    const char* method_name = m_reg_methods.method_names()[m_reg_methods.get_method_index()];
    std::cerr << "[auto-loop] using " << method_name << std::endl;

    while (m_running.load(std::memory_order_acquire)) {
        // ── Guard: need at least 2 keyframes ──────────────────────────
        if (m_graph->keyframes.size() < 2) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            continue;
        }

        // ── Step 1: Refresh sorted cache if keyframes changed ────────
        if (m_sorted_ids.size() != m_graph->keyframes.size()) {
            refresh_sorted_ids();
        }

        // ── Step 2: Select source keyframe ────────────────────────────
        size_t n = m_sorted_ids.size();
        size_t idx;

        if (m_search_method == RANDOM) {
            idx = static_cast<size_t>(std::rand()) % n;
            m_current_index = static_cast<int>(idx);
        } else {
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

        // ── Step 3: Find loop candidates via BFS ──────────────────────
        auto candidates = find_loop_candidates(source_id);
        update_status(source_id, candidates);

        // ── Step 4: Verify each candidate with scan matching ──────────
        registration->setInputTarget(source_kf->cloud);

        // ── Debug: timing for this iteration ─────────────────────────
        auto t_iter_start = std::chrono::steady_clock::now();

        bool edge_inserted = false;

        for (long cand_id : candidates) {
            if (!m_running.load(std::memory_order_acquire)) break;

            auto cand_it = m_graph->keyframes.find(cand_id);
            if (cand_it == m_graph->keyframes.end()) continue;
            auto& cand_kf = cand_it->second;
            if (!cand_kf->cloud || cand_kf->cloud->empty()) continue;

            // Initial guess: current relative pose from graph estimates
            Eigen::Isometry3d relative =
                source_pose.inverse() * cand_kf->node->estimate();

            registration->setInputSource(cand_kf->cloud);

            // ── Time the ICP align ──────────────────────────────────
            auto t_align_start = std::chrono::steady_clock::now();

            pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
            registration->align(*aligned, relative.matrix().cast<float>());

            auto t_align_end = std::chrono::steady_clock::now();
            double align_ms = std::chrono::duration<double, std::milli>(t_align_end - t_align_start).count();

            // Get refined transformation
            relative.matrix() =
                registration->getFinalTransformation().cast<double>();

            // ── Time the fitness calculation ────────────────────────
            auto t_fit_start = std::chrono::steady_clock::now();

            double fitness =
                InformationMatrixCalculator::calc_fitness_score(
                    source_kf->cloud, cand_kf->cloud, relative,
                    m_fitness_score_max_range);

            auto t_fit_end = std::chrono::steady_clock::now();
            double fit_ms = std::chrono::duration<double, std::milli>(t_fit_end - t_fit_start).count();

            std::cerr << "[auto-loop] " << method_name
                      << " | src=" << source_id << " → cand=" << cand_id
                      << " | align=" << align_ms << " ms"
                      << " | fitness=" << fit_ms << " ms"
                      << " | score=" << fitness
                      << (fitness < m_fitness_score_thresh ? " ✓" : "")
                      << std::endl;

            update_last_match(source_id, cand_id, fitness);

            if (fitness < static_cast<double>(m_fitness_score_thresh)) {
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

        // ── Step 5: Optional global optimization ──────────────────────
        if (edge_inserted && m_optimize) {
            std::lock_guard<std::mutex> lock(m_graph->optimization_mutex);
            m_graph->optimize();
        }

        // ── Step 6: Advance to next keyframe ──────────────────────────
        m_current_index++;

        // Yield to avoid CPU spin
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    // Mark stopped in status
    {
        std::lock_guard<std::mutex> lock(m_status_mutex);
        m_status.running = false;
    }
}

}  // namespace hdl_graph_slam
