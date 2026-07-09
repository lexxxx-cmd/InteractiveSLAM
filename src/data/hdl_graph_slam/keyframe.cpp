/**
 * @file keyframe.cpp
 * @brief 关键帧（KeyFrame）与关键帧快照（KeyFrameSnapshot）的实现
 *
 * 该文件实现了关键帧的数据持久化（保存与加载）、位姿查询接口
 * 以及关键帧快照的构造。
 *
 * 保存流程：
 *   1. 序列化元数据（时间戳、位姿、里程计、传感器数据）到 data 文件
 *   2. 保存原始点云为 raw.pcd
 *   3. 体素滤波降采样后保存为 cloud.pcd
 *
 * 加载流程：
 *   1. 从 data 文件解析各字段
 *   2. 在 g2o 图中根据 ID 查找对应的顶点
 *   3. 优先加载 raw.pcd（原始点云），若不存在则回退使用 cloud.pcd
 */
#include "data/hdl_graph_slam/keyframe.hpp"

#include <boost/filesystem.hpp>
#include <fstream>
#include <iostream>

#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/vertex_se3.h>

namespace hdl_graph_slam {

/**
 * @brief 基于里程计和点云创建新关键帧
 * @param odom            扫描匹配里程计输出的位姿估计
 * @param accum_distance  从起始节点累计的移动距离
 * @param cloud           关联的点云数据
 *
 * 该构造函数主要用于在线 SLAM 过程中，当检测到新的关键帧时调用。
 * 时间戳初始化为 0，节点指针初始化为 nullptr（稍后由图优化模块赋值）。
 */
KeyFrame::KeyFrame(const Eigen::Isometry3d& odom, double accum_distance,
                   const pcl::PointCloud<PointT>::ConstPtr& cloud)
    : stamp_nsec(0), odom(odom), accum_distance(accum_distance),
      cloud(cloud), node(nullptr) {}

/**
 * @brief 从目录加载已有关键帧
 * @param directory 关键帧数据所在的目录路径
 * @param graph     g2o 图对象，用于根据 ID 查找对应顶点
 *
 * 构造时自动调用 load() 从指定目录加载所有数据。
 */
KeyFrame::KeyFrame(const std::string& directory, g2o::HyperGraph* graph)
    : stamp_nsec(0), odom(Eigen::Isometry3d::Identity()), accum_distance(-1),
      cloud(nullptr), node(nullptr) {
    load(directory, graph);
}

KeyFrame::~KeyFrame() {}

/**
 * @brief 将关键帧保存到磁盘
 * @param directory 目标目录路径
 *
 * 保存的数据文件格式说明：
 *   - data: 文本格式，包含时间戳、位姿估计、里程计、累计距离及可选传感器数据
 *   - raw.pcd: 二进制 PCD 格式，保存原始未经降采样的点云
 *   - cloud.pcd: 二进制 PCD 格式，经 0.02m 体素滤波降采样后的点云
 */
void KeyFrame::save(const std::string& directory) {
    // 如果目标目录不存在，则创建
    if (!boost::filesystem::is_directory(directory)) {
        boost::filesystem::create_directory(directory);
    }

    // 写入元数据文件
    std::ofstream ofs(directory + "/data");
    uint64_t sec = stamp_nsec / 1000000000ull;   ///< 将纳秒时间戳拆分为秒
    uint64_t nsec = stamp_nsec % 1000000000ull;  ///< 和纳秒两部分
    ofs << "stamp " << sec << " " << nsec << "\n";

    // 保存图优化后的位姿估计（当前顶点估计值）
    ofs << "estimate\n";
    ofs << node->estimate().matrix() << "\n";

    // 保存原始里程计位姿
    ofs << "odom\n";
    ofs << odom.matrix() << "\n";

    // 保存累计移动距离
    ofs << "accum_distance " << accum_distance << "\n";

    // 以下为可选数据字段，仅在存在时保存
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

    // 保存原始（未采样）点云
    pcl::io::savePCDFileBinary(directory + "/raw.pcd", *cloud);

    // 对点云进行体素滤波降采样（分辨率 0.02m），保存为 cloud.pcd
    pcl::PointCloud<PointT>::Ptr sampled(new pcl::PointCloud<PointT>());
    pcl::VoxelGrid<PointT> voxel;            ///< 体素滤波器
    voxel.setInputCloud(cloud);               ///< 设置输入点云
    voxel.setLeafSize(0.02f, 0.02f, 0.02f);  ///< 设置体素大小（2cm）
    voxel.filter(*sampled);                   ///< 执行滤波
    pcl::io::savePCDFileBinary(directory + "/cloud.pcd", *sampled);
}

/**
 * @brief 从磁盘加载关键帧数据
 * @param directory 关键帧数据目录路径
 * @param graph     g2o 图对象
 * @return 成功返回 true，失败返回 false
 *
 * 加载流程：
 *   1. 打开 data 文件，按 token 解析各字段
 *   2. 根据读取的节点 ID 在 g2o 图中查找对应顶点
 *   3. 加载点云数据，优先使用 raw.pcd（原始点云）
 *
 * 注意：data 文件中的 estimate 字段仅作为存档参考，不再覆盖 g2o 顶点值。
 * 详情请参见头文件中的相关说明。
 */
bool KeyFrame::load(const std::string& directory, g2o::HyperGraph* graph) {
    std::ifstream ifs(directory + "/data");
    if (!ifs) {
        return false;  ///< data 文件不存在，加载失败
    }

    long node_id = -1;
    boost::optional<Eigen::Isometry3d> estimate;  ///< 从文件读取的缓存位姿估计

    // 逐 token 解析 data 文件
    while (!ifs.eof()) {
        std::string token;
        ifs >> token;

        if (token == "stamp") {
            // 解析时间戳：秒 纳秒
            uint64_t sec = 0, nsec = 0;
            ifs >> sec >> nsec;
            stamp_nsec = sec * 1000000000ull + nsec;
        } else if (token == "estimate") {
            // 解析 4x4 位姿矩阵
            Eigen::Matrix4d mat;
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    ifs >> mat(i, j);
                }
            }
            estimate = Eigen::Isometry3d::Identity();
            estimate->linear() = mat.block<3, 3>(0, 0);       ///< 旋转矩阵
            estimate->translation() = mat.block<3, 1>(0, 3);  ///< 平移向量
        } else if (token == "odom") {
            // 解析里程计位姿矩阵
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
            // 解析地面平面系数 (a, b, c, d)，满足 ax+by+cz+d=0
            Eigen::Vector4d coeffs;
            ifs >> coeffs[0] >> coeffs[1] >> coeffs[2] >> coeffs[3];
            floor_coeffs = coeffs;
        } else if (token == "utm_coord") {
            // 解析 UTM 坐标（东向、北向、高度）
            Eigen::Vector3d coord;
            ifs >> coord[0] >> coord[1] >> coord[2];
            utm_coord = coord;
        } else if (token == "acceleration") {
            // 解析 IMU 加速度
            Eigen::Vector3d acc;
            ifs >> acc[0] >> acc[1] >> acc[2];
            acceleration = acc;
        } else if (token == "orientation") {
            // 解析 IMU 姿态四元数 (w, x, y, z)
            Eigen::Quaterniond quat;
            ifs >> quat.w() >> quat.x() >> quat.y() >> quat.z();
            orientation = quat;
        } else if (token == "id") {
            ifs >> node_id;
        }
    }

    // 验证节点 ID 是否有效
    if (node_id < 0) {
        std::cerr << "error: invalid node id!!" << std::endl;
        std::cerr << "      : " << directory << std::endl;
        return false;
    }

    // 在 g2o 图中查找对应顶点
    if (graph->vertices().find(node_id) == graph->vertices().end()) {
        std::cerr << "error: vertex ID=" << node_id << " does not exist!!" << std::endl;
        return false;
    }

    // 将顶点向下转型为 SE3 位姿顶点
    node = dynamic_cast<g2o::VertexSE3*>(graph->vertices()[node_id]);
    if (node == nullptr) {
        std::cerr << "error: failed to downcast vertex to VertexSE3!!" << std::endl;
        return false;
    }

    // 注意：data 文件中的 estimate 字段是先前优化运行的缓存值。
    // 当 graph.g2o 与 data/estimate 不一致时（例如 LVBA 导出新 graph.g2o
    // 但未更新关键帧数据文件），盲目覆盖 g2o 顶点会产生不一致的图——
    // 错误的初始值与基于正确 g2o 位姿计算的边不匹配。
    // graph.g2o 现在被视为顶点估计的权威数据源，data 文件中的 estimate
    // 仍然被解析和存储以供参考，但不再覆盖 g2o 值。
    // 以下代码已注释禁用：
    // if (estimate) {
    //     node->setEstimate(*estimate);
    // }

    // 加载点云：优先加载 raw.pcd（原始未采样），回退使用 cloud.pcd
    // 以保持与 raw.pcd 拆分前保存的地图的向后兼容性
    pcl::PointCloud<PointT>::Ptr cloud_(new pcl::PointCloud<PointT>());
    if (boost::filesystem::exists(directory + "/raw.pcd")) {
        pcl::io::loadPCDFile(directory + "/raw.pcd", *cloud_);
    } else {
        pcl::io::loadPCDFile(directory + "/cloud.pcd", *cloud_);
    }
    cloud = cloud_;

    return true;
}

/**
 * @brief 获取关键帧对应的 g2o 顶点 ID
 * @return 顶点 ID（long 类型）
 */
long KeyFrame::id() const {
    return node->id();
}

/**
 * @brief 获取图优化后的位姿估计
 * @return SE3 等距变换（Isometry3d）
 */
Eigen::Isometry3d KeyFrame::estimate() const {
    return node->estimate();
}

/**
 * @brief 构造函数：直接指定位姿和点云创建快照
 * @param pose  图优化后的全局位姿
 * @param cloud 关联的点云
 */
KeyFrameSnapshot::KeyFrameSnapshot(const Eigen::Isometry3d& pose,
                                   const pcl::PointCloud<PointT>::ConstPtr& cloud)
    : pose(pose), cloud(cloud) {}

/**
 * @brief 构造函数：从关键帧创建快照
 * @param key 源关键帧（共享指针）
 *
 * 从关键帧中提取图优化后的位姿（来自 g2o 顶点估计值）和点云数据。
 */
KeyFrameSnapshot::KeyFrameSnapshot(const KeyFrame::Ptr& key)
    : pose(key->node->estimate()), cloud(key->cloud) {}

KeyFrameSnapshot::~KeyFrameSnapshot() {}

}  // namespace hdl_graph_slam
