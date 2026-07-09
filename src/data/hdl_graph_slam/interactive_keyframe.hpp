/**
 * @file interactive_keyframe.hpp
 * @brief 交互式关键帧（InteractiveKeyFrame）定义
 *
 * InteractiveKeyFrame 继承自 KeyFrame，增加了交互操作所需的额外功能：
 *   - KD-Tree 空间索引，支持快速邻域搜索
 *   - 法向量估计，支持点云法线计算
 *   - 边界盒（AABB）信息，用于空间查询和可视化
 *
 * 主要用途：
 *   在交互式 SLAM 系统中，用户需要对关键帧进行空间选择和编辑，
 *   例如选中关键帧的某个区域或计算局部法向量，这些功能由本类提供。
 */
#pragma once

#include <boost/any.hpp>
#include "data/hdl_graph_slam/keyframe.hpp"

namespace hdl_graph_slam {

/**
 * @brief 交互式关键帧，扩展 KeyFrame 以支持交互操作
 *
 * 相比基础 KeyFrame，InteractiveKeyFrame 增加了：
 *   - KD-Tree（存储在 kdtree_ 中）用于高效的半径邻域搜索
 *   - 法向量计算（首次调用时延迟计算，结果缓存在 normals_ 中）
 *   - 点云边界盒（min_pt/max_pt）用于快速空间判交
 *
 * 使用方式：
 *   - 通过 neighbors() 方法在关键帧的局部坐标系下查找近邻点
 *   - 通过 normals() 方法获取计算得到的点云法向量
 *   - 由 InteractiveGraph 统一管理，存储在 keyframes 映射表中
 */
struct InteractiveKeyFrame : public KeyFrame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  ///< Eigen 内存对齐支持

    using Ptr = std::shared_ptr<InteractiveKeyFrame>;  ///< 共享指针类型别名

    /**
     * @brief 构造函数：从目录加载关键帧并构建空间索引
     * @param directory 关键帧数据目录
     * @param graph     g2o 图对象
     *
     * 加载数据后立即为点云构建 KD-Tree 空间索引，以支持后续的邻域搜索。
     */
    InteractiveKeyFrame(const std::string& directory, g2o::HyperGraph* graph);

    virtual ~InteractiveKeyFrame() override;

    /**
     * @brief 在关键帧的局部坐标系下查找指定点的邻域点
     * @param pt     全局坐标系下的三维点
     * @param radius 搜索半径
     * @return 邻域点在点云中的索引列表
     *
     * 实现流程：
     *   1. 将查询点从全局坐标系变换到该关键帧的局部坐标系
     *   2. 使用 KD-Tree 进行半径搜索
     *   3. 返回匹配点的索引
     */
    std::vector<int> neighbors(const Eigen::Vector3f& pt, double radius);

    /**
     * @brief 获取关键帧点云的法向量（延迟计算）
     * @return 法向量点云指针
     *
     * 法向量在首次调用时计算并缓存，后续调用直接返回缓存结果。
     * 使用 OpenMP 加速的 NormalEstimation，搜索半径 0.25m。
     */
    pcl::PointCloud<pcl::Normal>::Ptr normals();

public:
    Eigen::Vector3f min_pt;  ///< 点云轴对齐边界盒（AABB）的最小角点
    Eigen::Vector3f max_pt;  ///< 点云轴对齐边界盒（AABB）的最大角点

    boost::any kdtree_;  ///< KD-Tree 空间索引（使用 boost::any 包装以避免模板暴露）
    pcl::PointCloud<pcl::Normal>::Ptr normals_;  ///< 点云法向量（延迟计算并缓存）
};

}  // namespace hdl_graph_slam
