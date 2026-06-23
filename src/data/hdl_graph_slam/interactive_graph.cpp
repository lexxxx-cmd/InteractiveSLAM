#include "data/hdl_graph_slam/interactive_graph.hpp"

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

InteractiveGraph::InteractiveGraph()
    : GraphSLAM("lm_var_cholmod"),
      iterations(0), chi2_before(0.0), chi2_after(0.0), elapsed_time_msec(0.0) {
    inf_calclator.reset(new InformationMatrixCalculator());
    inf_calclator->load(params);
    edge_id_gen = 0;
    anchor_node = nullptr;
    anchor_edge = nullptr;
}

InteractiveGraph::~InteractiveGraph() {
    if (optimization_thread.joinable()) {
        optimization_thread.join();
    }
}

bool InteractiveGraph::load_map_data(const std::string& directory,
                                      hdl_graph_slam::ProgressInterface& progress) {
    progress.set_title("Opening " + directory);
    progress.set_text("loading graph");
    if (!load(directory + "/graph.g2o")) {
        return false;
    }

    // re-assign edge ids
    edge_id_gen = 0;
    for (auto& edge : graph->edges()) {
        edge->setId(edge_id_gen++);
    }

    progress.increment();
    progress.set_text("loading keyframes");
    if (!load_keyframes(directory, progress)) {
        return false;
    }

    if (!load_special_nodes(directory, progress)) {
        return false;
    }

    return true;
}

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

                anchor_node = dynamic_cast<g2o::VertexSE3*>(graph->vertex(anchor_node_id));
                if (anchor_node == nullptr) {
                    std::cerr << "failed to cast anchor node to VertexSE3!!" << std::endl;
                    return false;
                }
                if (anchor_node->edges().empty()) {
                    std::cerr << "anchor node is not connected with any edges!!" << std::endl;
                    return false;
                }

                anchor_edge = dynamic_cast<g2o::EdgeSE3*>(*anchor_node->edges().begin());
                if (anchor_edge == nullptr) {
                    std::cerr << "failed to cast anchor edge to EdgeSE3!!" << std::endl;
                    return false;
                }
            }
        }
    }

    // create anchor node if not loaded
    if (anchor_node == nullptr) {
        std::cout << "create new anchor" << std::endl;
        using ID_Keyframe = std::pair<long, InteractiveKeyFrame::Ptr>;
        auto first_keyframe = std::min_element(
            keyframes.begin(), keyframes.end(),
            [=](const ID_Keyframe& lhs, const ID_Keyframe& rhs) {
                return lhs.first < rhs.first;
            });

        if (first_keyframe == keyframes.end()) {
            std::cerr << "corrupted graph file!!" << std::endl;
            return false;
        }

        anchor_node = add_se3_node(first_keyframe->second->node->estimate());
        anchor_edge = add_se3_edge(anchor_node, first_keyframe->second->node,
                                   Eigen::Isometry3d::Identity(),
                                   Eigen::MatrixXd::Identity(6, 6) * 0.1);
    }

    std::cout << "anchor_node:" << anchor_node->id() << std::endl;
    std::cout << "anchor_edge:" << anchor_edge->vertices()[0]->id()
              << " - " << anchor_edge->vertices()[1]->id() << std::endl;

    anchor_node->setFixed(true);
    return true;
}

bool InteractiveGraph::load_keyframes(const std::string& directory,
                                       hdl_graph_slam::ProgressInterface& progress) {
    progress.set_maximum(graph->vertices().size());
    for (int i = 0;; i++) {
        std::string keyframe_dir = (boost::format("%s/%06d") % directory % i).str();
        if (!boost::filesystem::is_directory(keyframe_dir)) {
            break;
        }

        InteractiveKeyFrame::Ptr keyframe =
            std::make_shared<InteractiveKeyFrame>(keyframe_dir, graph.get());
        if (!keyframe->node) {
            std::cerr << "error : failed to load keyframe!!" << std::endl;
            std::cerr << "      : " << keyframe_dir << std::endl;
        } else {
            keyframes[keyframe->id()] = keyframe;
            progress.increment();
        }
    }

    return true;
}

bool InteractiveGraph::removeEdge(long edgeId) {
    std::lock_guard<std::mutex> lock(optimization_mutex);
    g2o::SparseOptimizer* g = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
    if (!g) return false;

    for (auto* edge : g->edges()) {
        auto* se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
        if (!se3) continue;
        if (static_cast<long>(se3->id()) == edgeId) {
            return g->removeEdge(se3);
        }
    }
    return false;
}

long InteractiveGraph::anchor_node_id() const {
    return anchor_node ? anchor_node->id() : -1;
}

g2o::EdgeSE3* InteractiveGraph::add_edge(const KeyFrame::Ptr& key1, const KeyFrame::Ptr& key2,
                                          const Eigen::Isometry3d& relative_pose,
                                          const std::string& robust_kernel,
                                          double robust_kernel_delta) {
    Eigen::MatrixXd inf = inf_calclator->calc_information_matrix(key1->cloud, key2->cloud,
                                                                  relative_pose);
    g2o::EdgeSE3* edge = add_se3_edge(key1->node, key2->node, relative_pose, inf);
    edge->setId(edge_id_gen++);

    if (robust_kernel != "NONE") {
        add_robust_kernel(edge, robust_kernel, robust_kernel_delta);
    }

    return edge;
}

void InteractiveGraph::optimize(int num_iterations) {
    if (anchor_node) {
        g2o::VertexSE3* first_keyframe =
            dynamic_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1]);
        if (first_keyframe == nullptr) {
            std::cerr << "failed to cast the first keyframe to VertexSE3" << std::endl;
        } else {
            anchor_node->setEstimate(first_keyframe->estimate());
        }
    }

    optimization_stream.str("");
    optimization_stream.clear();

    std::streambuf* cerr_buf = std::cerr.rdbuf();
    std::cerr.rdbuf(optimization_stream.rdbuf());

    g2o::SparseOptimizer* g = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
    auto t1 = std::chrono::high_resolution_clock::now();

    if (num_iterations < 0) {
        num_iterations = params.param<int>("g2o_solver_num_iterations", 64);
    }

    chi2_before = g->chi2();
    iterations = GraphSLAM::optimize(num_iterations);
    chi2_after = g->chi2();

    auto t2 = std::chrono::high_resolution_clock::now();
    elapsed_time_msec =
        std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1000000.0;

    std::cerr.rdbuf(cerr_buf);
}

void InteractiveGraph::optimize_background(int num_iterations) {
    if (optimization_thread.joinable()) {
        optimization_thread.join();
    }

    if (anchor_node) {
        g2o::VertexSE3* first_keyframe =
            dynamic_cast<g2o::VertexSE3*>(anchor_edge->vertices()[1]);
        if (first_keyframe == nullptr) {
            std::cerr << "failed to cast the first keyframe to VertexSE3" << std::endl;
        } else {
            anchor_node->setEstimate(first_keyframe->estimate());
        }
    }

    optimization_stream.str("");
    optimization_stream.clear();

    auto task = [this, num_iterations]() {
        std::streambuf* cerr_buf = std::cerr.rdbuf();
        std::cerr.rdbuf(optimization_stream.rdbuf());

        std::lock_guard<std::mutex> lock(optimization_mutex);
        g2o::SparseOptimizer* g = dynamic_cast<g2o::SparseOptimizer*>(this->graph.get());
        auto t1 = std::chrono::high_resolution_clock::now();

        int max_iterations = num_iterations;
        if (num_iterations < 0) {
            max_iterations = params.param<int>("g2o_solver_num_iterations", 64);
        }

        chi2_before = g->chi2();
        iterations = GraphSLAM::optimize(max_iterations);
        chi2_after = g->chi2();

        auto t2 = std::chrono::high_resolution_clock::now();
        elapsed_time_msec =
            std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count() / 1000000.0;

        std::cerr.rdbuf(cerr_buf);
    };

    optimization_thread = std::thread(task);
}

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

std::string InteractiveGraph::optimization_messages() const {
    return optimization_stream.str();
}

void InteractiveGraph::dump(const std::string& directory, hdl_graph_slam::ProgressInterface& progress) {
    progress.set_maximum(keyframes.size());
    progress.set_text("saving graph");
    progress.increment();

    save(directory + "/graph.g2o");

    progress.set_text("saving keyframes");

    int keyframe_id = 0;
    for (const auto& keyframe : keyframes) {
        progress.increment();
        std::stringstream sst;
        sst << boost::format("%s/%06d") % directory % (keyframe_id++);
        keyframe.second->save(sst.str());
    }

    std::ofstream ofs(directory + "/special_nodes.csv");
    ofs << "anchor_node " << (anchor_node != nullptr ? anchor_node->id() : -1) << std::endl;
    ofs << "anchor_edge " << -1 << std::endl;
    ofs << "floor_node " << -1 << std::endl;
}

bool InteractiveGraph::save_pointcloud(const std::string& filename,
                                        hdl_graph_slam::ProgressInterface& progress) {
    progress.set_maximum(keyframes.size() + 1);
    progress.set_text("accumulate points");

    pcl::PointCloud<pcl::PointXYZI>::Ptr accumulated(new pcl::PointCloud<pcl::PointXYZI>());
    for (const auto& keyframe : keyframes) {
        progress.increment();

        pcl::PointCloud<pcl::PointXYZI>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::transformPointCloud(*keyframe.second->cloud, *transformed,
                                 keyframe.second->node->estimate().cast<float>());

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

}  // namespace hdl_graph_slam
