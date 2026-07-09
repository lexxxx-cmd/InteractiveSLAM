/**
 * @file keyframe.hpp
 * @brief 关键帧（KeyFrame）数据结构定义
 *
 * 该文件定义了 HDL Graph SLAM 中的核心数据结构——关键帧（KeyFrame）。
 * 关键帧代表 SLAM 轨迹中的一个位姿节点，包含：
 *   - 位姿估计（里程计与图优化后的结果）、
 *   - 关联的点云数据、
 *   - 可选的地面系数、GPS 坐标、IMU 加速度与姿态等附加信息。
 *
 * 本文件同时定义了 KeyFrameSnapshot，用于生成完整地图点云时的快照表示。
 *
 * 原始版本基于 ROS，本适配版本移除了 ROS 依赖：
 *   - ros::Time 替换为 uint64_t 纳秒时间戳
 *   - 移除了 ROS 头文件和 ROS_ERROR_STREAM
 *   - 移除了依赖 ROS 的构造函数，保留了从文件加载的构造函数
 */
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <boost/optional.hpp>

namespace g2o {
class VertexSE3;      ///< g2o SE3 位姿顶点（前向声明）
class HyperGraph;     ///< g2o 超图基类（前向声明）
class SparseOptimizer; ///< g2o 稀疏优化器（前向声明）
}  // namespace g2o

namespace hdl_graph_slam {

/**
 * @brief 关键帧结构体，表示 SLAM 轨迹中的单个位姿节点
 *
 * 每个 KeyFrame 包含位姿信息、关联的点云以及可选的传感器数据。
 * 它是图优化中顶点（g2o::VertexSE3）的数据载体，同时负责自身数据
 * 的序列化保存与加载。
 *
 * 使用方式：
 *   - 在扫描匹配里程计中创建新的关键帧
 *   - 通过 save()/load() 实现磁盘持久化存储
 *   - 通过 node 成员访问对应的 g2o 顶点，参与图优化
 */
struct KeyFrame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  ///< Eigen 内存对齐支持

    using PointT = pcl::PointXYZI;                     ///< 点云点类型（XYZ + 强度）
    using Ptr = std::shared_ptr<KeyFrame>;              ///< 共享指针类型别名

    /**
     * @brief 构造函数：基于里程计数据创建新的关键帧
     * @param odom         扫描匹配里程计给出的位姿估计
     * @param accum_distance 从起始节点累计的移动距离
     * @param cloud        关联的点云数据
     */
    KeyFrame(const Eigen::Isometry3d& odom, double accum_distance,
             const pcl::PointCloud<PointT>::ConstPtr& cloud);

    /**
     * @brief 构造函数：从目录加载关键帧数据
     * @param directory 关键帧数据所在的目录路径
     * @param graph     g2o 图，用于查找顶点
     */
    KeyFrame(const std::string& directory, g2o::HyperGraph* graph);

    virtual ~KeyFrame();

    /**
     * @brief 将关键帧数据保存到指定目录
     * @param directory 目标目录路径
     *
     * 保存内容包括：
     *   - data 文件：时间戳、位姿估计、里程计、累计距离、可选传感器数据
     *   - raw.pcd 文件：原始（未降采样）点云
     *   - cloud.pcd 文件：经 0.02m 体素滤波降采样后的点云
     */
    void save(const std::string& directory);

    /**
     * @brief 从指定目录加载关键帧数据
     * @param directory 关键帧数据目录路径
     * @param graph     g2o 图对象，用于根据 ID 查找对应的顶点
     * @return 加载成功返回 true，失败返回 false
     *
     * 注意：data 文件中保存的 "estimate" 字段是先前优化运行的缓存值。
     * 当 graph.g2o 与 data/estimate 不一致时（例如 LVBA 导出新 graph.g2o
     * 但未更新关键帧数据文件），盲目覆盖 g2o 顶点会导致图不一致。
     * 因此 graph.g2o 被视为顶点估计的权威数据源，data 文件中的 estimate
     * 仅供存档参考，不会覆盖 g2o 中的值。
     */
    bool load(const std::string& directory, g2o::HyperGraph* graph);

    /**
     * @brief 获取关键帧对应的 g2o 顶点 ID
     * @return 顶点 ID
     */
    long id() const;

    /**
     * @brief 获取当前图优化估计的位姿
     * @return 优化后的 SE3 位姿变换
     */
    Eigen::Isometry3d estimate() const;

public:
    uint64_t stamp_nsec = 0;                       ///< 时间戳（纳秒，替代 ros::Time）
    Eigen::Isometry3d odom;                        ///< 里程计位姿（由扫描匹配里程计估计）
    double accum_distance;                         ///< 从第一个节点开始的累计移动距离
    pcl::PointCloud<PointT>::ConstPtr cloud;       ///< 关联的点云数据（常指针）
    boost::optional<Eigen::Vector4d> floor_coeffs; ///< 检测到的地面平面系数（可选）
    boost::optional<Eigen::Vector3d> utm_coord;    ///< GPS 获取的 UTM 坐标（可选）

    boost::optional<Eigen::Vector3d> acceleration;      ///< IMU 加速度数据（可选）
    boost::optional<Eigen::Quaterniond> orientation;     ///< IMU 姿态四元数（可选）

    g2o::VertexSE3* node;  ///< g2o 图优化中的对应顶点实例
};

/**
 * @brief 用于生成地图点云的关键帧快照
 *
 * KeyFrameSnapshot 是对 KeyFrame 的轻量级封装，保存了图优化后的位姿
 * 和点云数据，主要用于：
 *   - 将各关键帧的点云变换到全局坐标系
 *   - 拼接生成完整的全局地图点云
 */
struct KeyFrameSnapshot {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  ///< Eigen 内存对齐支持

    using PointT = KeyFrame::PointT;   ///< 点云点类型
    using Ptr = std::shared_ptr<KeyFrameSnapshot>;  ///< 共享指针类型别名

    /**
     * @brief 构造函数：从关键帧创建快照
     * @param key 源关键帧（共享指针）
     */
    KeyFrameSnapshot(const KeyFrame::Ptr& key);

    /**
     * @brief 构造函数：直接指定位姿和点云
     * @param pose  图优化估计的位姿
     * @param cloud 关联的点云
     */
    KeyFrameSnapshot(const Eigen::Isometry3d& pose,
                     const pcl::PointCloud<PointT>::ConstPtr& cloud);

    ~KeyFrameSnapshot();

public:
    Eigen::Isometry3d pose;                   ///< 图优化后的全局位姿
    pcl::PointCloud<PointT>::ConstPtr cloud;  ///< 关联的点云数据
};

}  // namespace hdl_graph_slam
