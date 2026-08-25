/**
 * @file bag_importer.hpp
 * @brief ROS1 bag 导入器 —— 按 SCPGO 数据流将 bag 解析为标准地图目录
 *
 * 数据流（对齐 SCPGO，不含回环检测/优化）：
 *   bag（odom topic + PointCloud2 topic）
 *     → 逐帧解析位姿（T_WL 语义）与点云
 *     → 时间戳最近邻匹配
 *     → 关键帧抽稀（meter_gap / deg_gap，或 passthrough 全量）
 *     → 外参变换（T_WL → T_WI，点云变换到 IMU 局部系）
 *     → 导出标准地图目录：
 *         <outDir>/keyframes/graph.g2o   （纯里程计位姿图）
 *         <outDir>/keyframes/NNNNNN/     （data + raw.pcd + cloud.pcd）
 *
 * 全部参数由 YAML 配置（config/bag_import.yaml）控制。
 * 限制：仅支持未压缩的 ROS1 bag V2.0。
 */
#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "data/hdl_graph_slam/progress_interface.hpp"

namespace hdl_graph_slam {

/** @brief bag 导入配置（对应 config/bag_import.yaml） */
struct BagImportConfig {
    std::string bagPath;                ///< bag 文件路径
    std::string odomTopic = "/odom";    ///< 位姿 topic（nav_msgs/Odometry）
    std::string cloudTopic = "/velodyne_points";  ///< 点云 topic（sensor_msgs/PointCloud2）

    // Lidar↔IMU 外参：P_IMU = R_IL * P_Lidar + t_IL（单位阵 = 不转换）
    Eigen::Matrix3d calibR = Eigen::Matrix3d::Identity();
    Eigen::Vector3d calibT = Eigen::Vector3d::Zero();

    bool   passthroughMode = true;      ///< true = 逐帧全量关键帧
    double meterGap = 1.0;              ///< 关键帧抽稀：累计平移阈值（米）
    double degGap = 30.0;               ///< 关键帧抽稀：累计旋转阈值（度）

    std::string outputDir;              ///< 输出根目录（空 = 临时目录）
    bool saveFullCloud = true;          ///< 是否保存全分辨率 raw.pcd
};

/** @brief bag 导入结果 */
struct BagImportResult {
    bool success = false;               ///< 是否成功
    std::string mapDirectory;           ///< 生成的标准地图目录
    std::string error;                  ///< 失败原因（success=false 时有效）
    int keyframeCount = 0;              ///< 生成的关键帧数
};

/**
 * @brief ROS1 bag 导入器（纯计算，无 Qt 依赖，可在工作线程执行）
 */
class BagImporter {
public:
    /**
     * @brief 从 YAML 文件加载导入配置（解析失败用默认值）
     * @param yamlPath 配置文件路径
     * @return 配置（bagPath 需由调用方设置）
     */
    static BagImportConfig loadConfig(const std::string& yamlPath);

    /**
     * @brief 列出 bag 内的全部 topic（仅读索引，速度快）
     * @param bagPath bag 文件路径
     * @return topic 名列表（打开失败返回空）
     */
    static std::vector<std::string> listTopics(const std::string& bagPath);

    /**
     * @brief 按配置解析 bag 并生成标准地图目录（可在后台线程调用）
     * @param cfg      导入配置（含 bagPath / topics / 外参 / 抽稀 / 输出目录）
     * @param progress 进度回调
     * @return 导入结果
     */
    static BagImportResult import(const BagImportConfig& cfg,
                                  ProgressInterface& progress);
};

}  // namespace hdl_graph_slam
