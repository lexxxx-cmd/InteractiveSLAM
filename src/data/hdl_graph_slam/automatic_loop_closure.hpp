#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <deque>

#include <Eigen/Dense>

#include "data/hdl_graph_slam/registration_methods.hpp"

namespace hdl_graph_slam {

class InteractiveGraph;
class InteractiveKeyFrame;

/**
 * @brief Automatic loop closure detection — pure algorithm layer.
 *
 * Runs a background std::thread that iterates over keyframes, performs BFS
 * along g2o edges to find loop candidates, runs scan-matching to verify, and
 * inserts edges with optional graph optimization.
 *
 * Thread safety:
 *   - m_running is std::atomic_bool
 *   - m_status_mutex protects the Status snapshot read by Qt UI polling
 *   - graph->optimization_mutex is locked around optimize() calls
 *   - graph->add_edge() is single-writer (only this thread writes)
 *
 * No Qt / ImGui dependencies — designed to be owned by AutoLoopClosurePanel.
 */
class AutomaticLoopClosure {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// Thread-safe status snapshot consumed by Qt UI polling timer.
    struct Status {
        bool running = false;
        long  current_source_id = -1;
        std::vector<long> candidate_ids;
        long  last_begin_id      = -1;
        long  last_end_id        = -1;
        double last_fitness_score = 0.0;
        int   edges_inserted     = 0;
    };

    /// @param graph  Non-owning pointer. Caller must guarantee the graph
    ///               outlives this object (or stop() is called before
    ///               the graph is destroyed).
    explicit AutomaticLoopClosure(InteractiveGraph* graph);
    ~AutomaticLoopClosure();

    // ── Lifecycle ──────────────────────────────────────────────
    void start();
    void stop();
    bool is_running() const { return m_running.load(std::memory_order_acquire); }

    // ── Thread-safe status query ───────────────────────────────
    Status snapshot() const;

    // ── Parameters (set from UI thread before/during run) ──────
    enum SearchMethod { SEQUENTIAL = 0, RANDOM = 1 };

    void set_search_method(int m)         { m_search_method = m; }
    void set_distance_thresh(double v)    { m_distance_thresh = v; }
    void set_accum_distance_thresh(double v) { m_accum_distance_thresh = v; }
    void set_fitness_score_thresh(float v)   { m_fitness_score_thresh = v; }
    void set_fitness_score_max_range(float v) { m_fitness_score_max_range = v; }
    void set_registration_method_index(int idx) { m_reg_methods.set_method_index(idx); }
    void set_registration_max_iterations(int n)  { m_reg_methods.set_max_iterations(n); }
    void set_registration_epsilon(float eps)     { m_reg_methods.set_transformation_epsilon(eps); }
    void set_registration_resolution(float r)    { m_reg_methods.set_resolution(r); }
    void set_robust_kernel_type(int t)    { m_kernel_type = t; }
    void set_robust_kernel_delta(float d) { m_kernel_delta = d; }
    void set_optimize_after_insert(bool v) { m_optimize = v; }

    // ── Accessors for UI initialisation ────────────────────────
    const RegistrationMethods& reg_methods() const { return m_reg_methods; }

    int    search_method()          const { return m_search_method; }
    double distance_thresh()        const { return m_distance_thresh; }
    double accum_distance_thresh()  const { return m_accum_distance_thresh; }
    float  fitness_score_thresh()   const { return m_fitness_score_thresh; }
    float  fitness_score_max_range() const { return m_fitness_score_max_range; }
    int    robust_kernel_type()     const { return m_kernel_type; }
    float  robust_kernel_delta()    const { return m_kernel_delta; }
    bool   optimize_after_insert()  const { return m_optimize; }

private:
    // ── Background thread entry ────────────────────────────────
    void loop_detection();
    std::vector<long> find_loop_candidates(long source_id);

    // ── Helpers ────────────────────────────────────────────────
    void refresh_sorted_ids();
    static const char* kernel_name(int type);
    void update_status(long source_id, const std::vector<long>& candidates);
    void update_last_match(long begin_id, long end_id, double fitness);

    // ── Data ───────────────────────────────────────────────────
    InteractiveGraph* m_graph;          // non-owning

    // Threading
    std::thread  m_thread;
    std::atomic_bool m_running{false};

    // Status snapshot
    mutable std::mutex m_status_mutex;
    Status m_status;

    // Parameters
    RegistrationMethods m_reg_methods;

    int    m_search_method         = 1;      // RANDOM
    double m_distance_thresh       = 10.0;
    double m_accum_distance_thresh = 15.0;
    float  m_fitness_score_thresh  = 0.3f;
    float  m_fitness_score_max_range = 2.0f;
    int    m_kernel_type           = 0;      // NONE
    float  m_kernel_delta          = 0.01f;
    bool   m_optimize              = true;

    // Sorted keyframe IDs for sequential indexing
    std::vector<long> m_sorted_ids;
    int  m_current_index = 0;
    size_t m_last_keyframe_count = 0;
};

}  // namespace hdl_graph_slam
