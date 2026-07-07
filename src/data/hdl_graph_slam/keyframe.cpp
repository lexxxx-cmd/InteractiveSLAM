#include "data/hdl_graph_slam/keyframe.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_se3.h>

namespace hdl_graph_slam {

KeyFrame::KeyFrame(const Eigen::Isometry3d& odom, double accum_distance,
                   const pcl::PointCloud<PointT>::ConstPtr& cloud)
    : stamp_nsec(0), odom(odom), accum_distance(accum_distance),
      cloud(cloud), node(nullptr) {}

KeyFrame::KeyFrame(const std::string& directory, g2o::HyperGraph* graph)
    : stamp_nsec(0), odom(Eigen::Isometry3d::Identity()), accum_distance(-1),
      cloud(nullptr), node(nullptr) {
    load(directory, graph);
}

KeyFrame::~KeyFrame() {}

void KeyFrame::save(const std::string& directory) {
    if (!boost::filesystem::is_directory(directory)) {
        boost::filesystem::create_directory(directory);
    }

    std::ofstream ofs(directory + "/data");
    uint64_t sec = stamp_nsec / 1000000000ull;
    uint64_t nsec = stamp_nsec % 1000000000ull;
    ofs << "stamp " << sec << " " << nsec << "\n";

    ofs << "estimate\n";
    ofs << node->estimate().matrix() << "\n";

    ofs << "odom\n";
    ofs << odom.matrix() << "\n";

    ofs << "accum_distance " << accum_distance << "\n";

    if (floor_coeffs) {
        ofs << "floor_coeffs " << floor_coeffs->transpose() << "\n";
    }

    if (utm_coord) {
        ofs << "utm_coord " << utm_coord->transpose() << "\n";
    }

    if (acceleration) {
        ofs << "acceleration " << acceleration->transpose() << "\n";
    }

    if (orientation) {
        ofs << "orientation " << orientation->w() << " "
            << orientation->x() << " " << orientation->y() << " "
            << orientation->z() << "\n";
    }

    if (node) {
        ofs << "id " << node->id() << "\n";
    }

    // Save raw (unsampled) cloud first
    pcl::io::savePCDFileBinary(directory + "/raw.pcd", *cloud);

    // Downsample to 0.02m voxel grid for cloud.pcd
    pcl::PointCloud<PointT>::Ptr sampled(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(0.02f, 0.02f, 0.02f);
    voxel.filter(*sampled);
    pcl::io::savePCDFileBinary(directory + "/cloud.pcd", *sampled);
}

bool KeyFrame::load(const std::string& directory, g2o::HyperGraph* graph) {
    std::ifstream ifs(directory + "/data");
    if (!ifs) {
        return false;
    }

    long node_id = -1;
    boost::optional<Eigen::Isometry3d> estimate;

    while (!ifs.eof()) {
        std::string token;
        ifs >> token;

        if (token == "stamp") {
            uint64_t sec = 0, nsec = 0;
            ifs >> sec >> nsec;
            stamp_nsec = sec * 1000000000ull + nsec;
        } else if (token == "estimate") {
            Eigen::Matrix4d mat;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    ifs >> mat(i, j);
                }
            }
            estimate = Eigen::Isometry3d::Identity();
            estimate->linear() = mat.block<3, 3>(0, 0);
            estimate->translation() = mat.block<3, 1>(0, 3);
        } else if (token == "odom") {
            Eigen::Matrix4d odom_mat = Eigen::Matrix4d::Identity();
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    ifs >> odom_mat(i, j);
                }
            }
            odom.setIdentity();
            odom.linear() = odom_mat.block<3, 3>(0, 0);
            odom.translation() = odom_mat.block<3, 1>(0, 3);
        } else if (token == "accum_distance") {
            ifs >> accum_distance;
        } else if (token == "floor_coeffs") {
            Eigen::Vector4d coeffs;
            ifs >> coeffs[0] >> coeffs[1] >> coeffs[2] >> coeffs[3];
            floor_coeffs = coeffs;
        } else if (token == "utm_coord") {
            Eigen::Vector3d coord;
            ifs >> coord[0] >> coord[1] >> coord[2];
            utm_coord = coord;
        } else if (token == "acceleration") {
            Eigen::Vector3d acc;
            ifs >> acc[0] >> acc[1] >> acc[2];
            acceleration = acc;
        } else if (token == "orientation") {
            Eigen::Quaterniond quat;
            ifs >> quat.w() >> quat.x() >> quat.y() >> quat.z();
            orientation = quat;
        } else if (token == "id") {
            ifs >> node_id;
        }
    }

    if (node_id < 0) {
        std::cerr << "error: invalid node id!!" << std::endl;
        std::cerr << "      : " << directory << std::endl;
        return false;
    }

    if (graph->vertices().find(node_id) == graph->vertices().end()) {
        std::cerr << "error: vertex ID=" << node_id << " does not exist!!" << std::endl;
        return false;
    }

    node = dynamic_cast<g2o::VertexSE3*>(graph->vertices()[node_id]);
    if (node == nullptr) {
        std::cerr << "error: failed to downcast vertex to VertexSE3!!" << std::endl;
        return false;
    }

    // The data file's "estimate" field is a stale cache from a previous
    // optimization run.  When graph.g2o and data/estimate disagree (e.g.
    // after LVBA exports a fresh graph.g2o without updating the keyframe
    // data files), blindly overwriting the g2o vertex produces an
    // inconsistent graph — wrong initial values paired with edges computed
    // from the correct g2o poses.
    //
    // graph.g2o is now treated as the single source of truth for vertex
    // estimates.  The data file's estimate is still parsed and stored for
    // reference, but no longer overwrites the authoritative g2o value.
    //
    // if (estimate) {
    //     node->setEstimate(*estimate);
    // }

    // Prefer raw.pcd (unsampled original); fallback to cloud.pcd for
    // backward compatibility with maps saved before the raw.pcd split.
    pcl::PointCloud<PointT>::Ptr cloud_(new pcl::PointCloud<PointT>());
    if (boost::filesystem::exists(directory + "/raw.pcd")) {
        pcl::io::loadPCDFile(directory + "/raw.pcd", *cloud_);
    } else {
        pcl::io::loadPCDFile(directory + "/cloud.pcd", *cloud_);
    }
    cloud = cloud_;

    return true;
}

long KeyFrame::id() const {
    return node->id();
}

Eigen::Isometry3d KeyFrame::estimate() const {
    return node->estimate();
}

KeyFrameSnapshot::KeyFrameSnapshot(const Eigen::Isometry3d& pose,
                                   const pcl::PointCloud<PointT>::ConstPtr& cloud)
    : pose(pose), cloud(cloud) {}

KeyFrameSnapshot::KeyFrameSnapshot(const KeyFrame::Ptr& key)
    : pose(key->node->estimate()), cloud(key->cloud) {}

KeyFrameSnapshot::~KeyFrameSnapshot() {}

}  // namespace hdl_graph_slam
