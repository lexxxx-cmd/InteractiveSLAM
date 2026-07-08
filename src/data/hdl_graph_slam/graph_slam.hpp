#pragma once

#include <memory>
#include <Eigen/Dense>

#include <g2o/core/hyper_graph.h>

namespace g2o {
class VertexSE3;
class EdgeSE3;
class EdgeSE3PriorXY;
class EdgeSE3PriorXYZ;
class EdgeSE3PriorVec;
class EdgeSE3PriorQuat;
class RobustKernelFactory;
}  // namespace g2o

namespace hdl_graph_slam {

/**
 * @brief g2o graph wrapper — SE3-only adaptation
 *
 * Removed from original:
 *   - #include <ros/time.h>
 *   - All Plane-related methods (add_plane_node, add_se3_plane_edge, etc.)
 *   - All Plane custom type registrations
 *
 * Kept: SE3 nodes, SE3 edges, SE3 prior edges, robust kernel support
 */
class GraphSLAM {
public:
    GraphSLAM(const std::string& solver_type = "lm_var");
    virtual ~GraphSLAM();

    int num_vertices() const;
    int num_edges() const;

    void set_solver(const std::string& solver_type);

    g2o::VertexSE3* add_se3_node(const Eigen::Isometry3d& pose);

    g2o::EdgeSE3* add_se3_edge(g2o::VertexSE3* v1, g2o::VertexSE3* v2,
                               const Eigen::Isometry3d& relative_pose,
                               const Eigen::MatrixXd& information_matrix);

    g2o::EdgeSE3PriorXY* add_se3_prior_xy_edge(g2o::VertexSE3* v_se3,
                                                const Eigen::Vector2d& xy,
                                                const Eigen::MatrixXd& information_matrix);

    g2o::EdgeSE3PriorXYZ* add_se3_prior_xyz_edge(g2o::VertexSE3* v_se3,
                                                  const Eigen::Vector3d& xyz,
                                                  const Eigen::MatrixXd& information_matrix);

    g2o::EdgeSE3PriorQuat* add_se3_prior_quat_edge(g2o::VertexSE3* v_se3,
                                                    const Eigen::Quaterniond& quat,
                                                    const Eigen::MatrixXd& information_matrix);

    g2o::EdgeSE3PriorVec* add_se3_prior_vec_edge(g2o::VertexSE3* v_se3,
                                                  const Eigen::Vector3d& direction,
                                                  const Eigen::Vector3d& measurement,
                                                  const Eigen::MatrixXd& information_matrix);

    void add_robust_kernel(g2o::HyperGraph::Edge* edge,
                           const std::string& kernel_type, double kernel_size);

    int optimize(int num_iterations);

    void save(const std::string& filename);
    bool load(const std::string& filename);

public:
    g2o::RobustKernelFactory* robust_kernel_factory;
    std::unique_ptr<g2o::HyperGraph> graph;  // g2o graph
};

}  // namespace hdl_graph_slam
