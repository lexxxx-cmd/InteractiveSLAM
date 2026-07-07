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

    // re-assign edge ids and mark all loaded edges as Original
    edge_id_gen = 0;
    edge_sources.clear();
    for (auto& edge : graph->edges()) {
        edge->setId(edge_id_gen);
        edge_sources[edge_id_gen] = EdgeSource::Original;
        edge_id_gen++;
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

                // The CSV anchor may refer to a node that does not exist in
                // the current graph.g2o (e.g. after LVBA exports a fresh g2o
                // without the anchor).  Degrade gracefully: warn and fall
                // through to automatic anchor creation below.
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

                anchor_node = candidate;
                anchor_edge = candidate_edge;
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
    // Use try_lock to avoid blocking the UI thread when a background
    // optimization (e.g. auto loop closure) holds the mutex.
    std::unique_lock<std::mutex> lock(optimization_mutex, std::try_to_lock);
    if (!lock.owns_lock()) {
        return false;  // optimization in progress, caller should retry
    }
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
                                          double robust_kernel_delta,
                                          EdgeSource source) {
    Eigen::MatrixXd inf = inf_calclator->calc_information_matrix(key1->cloud, key2->cloud,
                                                                  relative_pose);
    g2o::EdgeSE3* edge = add_se3_edge(key1->node, key2->node, relative_pose, inf);
    long eid = edge_id_gen++;
    edge->setId(eid);
    edge_sources[eid] = source;

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

bool InteractiveGraph::saveLVBA(const std::string& directory,
                                hdl_graph_slam::ProgressInterface& progress) {
    namespace fs = boost::filesystem;

    // Output to parent directory: <dir>/../all_pcd_body/
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

    // Collect keyframes with valid cloud and node
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

    // Sort by stamp_nsec (ascending), tiebreak by g2o node ID
    std::sort(sorted.begin(), sorted.end(),
        [](const InteractiveKeyFrame::Ptr& a, const InteractiveKeyFrame::Ptr& b) {
            if (a->stamp_nsec != b->stamp_nsec)
                return a->stamp_nsec < b->stamp_nsec;
            return a->id() < b->id();
        });

    // Write lidar_poses.txt in TUM format
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

        // Pose from g2o vertex estimate (authoritative)
        const Eigen::Isometry3d& pose = kf->node->estimate();
        Eigen::Quaterniond q(pose.linear());
        Eigen::Vector3d t = pose.translation();

        // Timestamp: nanoseconds → seconds
        double timestamp = static_cast<double>(kf->stamp_nsec) / 1e9;

        // TUM: timestamp tx ty tz qx qy qz qw
        ofs << boost::format("%.6f %.9f %.9f %.9f %.9f %.9f %.9f %.9f\n")
                   % timestamp % t.x() % t.y() % t.z()
                   % q.x() % q.y() % q.z() % q.w();

        // Write PCD (IMU body-frame cloud, no transform)
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
