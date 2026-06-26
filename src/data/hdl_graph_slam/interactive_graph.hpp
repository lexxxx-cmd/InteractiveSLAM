#pragma once

#include <regex>
#include <mutex>
#include <thread>
#include <unordered_map>

#include <Eigen/Dense>
#include <boost/format.hpp>

#include "data/hdl_graph_slam/progress_interface.hpp"
#include "data/hdl_graph_slam/graph_slam.hpp"
#include "data/hdl_graph_slam/parameter_server.hpp"
#include "data/hdl_graph_slam/interactive_keyframe.hpp"
#include "data/hdl_graph_slam/information_matrix_calculator.hpp"

namespace g2o {
class VertexSE3;
class EdgeSE3;
}  // namespace g2o

namespace hdl_graph_slam {

/**
 * @brief Marks how an edge was created — used by EdgeListPanel to filter display.
 */
enum class EdgeSource {
    Original,    // loaded from graph file (hidden in EdgeListPanel)
    ManualLoop,  // manually added loop closure
    AutoLoop,    // automatic loop detection
    Anchor       // anchor-related edge
};

/**
 * @brief Core class to interactively manipulate a pose graph — SE3-only adaptation
 *
 * Plane-related methods (add_plane, add_edge_identity, add_edge_parallel, etc.)
 * are removed. Only SE3 graph loading, saving, and optimization are supported
 * in this phase.
 */
class InteractiveGraph : protected GraphSLAM {
public:
    InteractiveGraph();
    virtual ~InteractiveGraph();

    bool load_map_data(const std::string& directory, hdl_graph_slam::ProgressInterface& progress);

    long anchor_node_id() const;

    g2o::EdgeSE3* add_edge(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2,
                           const Eigen::Isometry3d& relative_pose,
                           const std::string& robust_kernel = "NONE",
                           double robust_kernel_delta = 0.1,
                           EdgeSource source = EdgeSource::ManualLoop);

    bool removeEdge(long edgeId);

    EdgeSource edge_source(long edge_id) const {
        auto it = edge_sources.find(edge_id);
        return (it != edge_sources.end()) ? it->second : EdgeSource::Original;
    }

    void optimize(int num_iterations = -1);
    void optimize_background(int num_iterations = -1);

    std::string graph_statistics(bool update = false);
    std::string optimization_messages() const;

    void dump(const std::string& directory, hdl_graph_slam::ProgressInterface& progress);
    bool save_pointcloud(const std::string& filename, hdl_graph_slam::ProgressInterface& progress);

    using GraphSLAM::graph;
    using GraphSLAM::num_edges;
    using GraphSLAM::num_vertices;
    using GraphSLAM::save;
    using GraphSLAM::set_solver;

private:
    bool load_special_nodes(const std::string& directory, hdl_graph_slam::ProgressInterface& progress);
    bool load_keyframes(const std::string& directory, hdl_graph_slam::ProgressInterface& progress);

private:
    g2o::VertexSE3* anchor_node;
    g2o::EdgeSE3* anchor_edge;
    long edge_id_gen;

    std::thread optimization_thread;

public:
    mutable std::mutex optimization_mutex;

    std::string graph_stats;
    std::stringstream optimization_stream;

    ParameterServer params;

    int iterations;
    double chi2_before;
    double chi2_after;
    double elapsed_time_msec;

    std::unordered_map<long, InteractiveKeyFrame::Ptr> keyframes;
    std::unordered_map<long, EdgeSource> edge_sources;
    std::unique_ptr<InformationMatrixCalculator> inf_calclator;
};

}  // namespace hdl_graph_slam
