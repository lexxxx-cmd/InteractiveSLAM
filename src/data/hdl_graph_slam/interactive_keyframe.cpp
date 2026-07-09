/**
 * @file interactive_keyframe.cpp
 * @brief 交互式关键帧（InteractiveKeyFrame）的实现
 *
 * 该文件实现了 InteractiveKeyFrame 的核心功能：
 *   - 构造函数中构建 KD-Tree 空间索引
 *   - neighbors()：局部坐标系下的半径邻域搜索
 *   - normals()：延迟计算点云法向量
 *
 * 邻域搜索的实现要点：
 *   查询点需从全局坐标系转换到关键帧的局部坐标系，
 *   这是因为点云数据存储在关键帧的局部坐标系下。
 */
#include "data/hdl_graph_slam/interactive_keyframe.hpp"

#include <g2o/types/slam3d/vertex_se3.h>

#include <pcl/search/kdtree.h>
#include <pcl/search/impl/kdtree.hpp>
#include <pcl/features/normal_3d_omp.h>

namespace hdl_graph_slam {

/**
 * @brief 构造函数：从目录加载关键帧并初始化 KD-Tree
 * @param directory 关键帧数据目录
 * @param graph     g2o 图对象
 *
 * 初始化流程：
 *   1. 调用基类 KeyFrame 的构造函数，从目录加载数据
 *   2. 为点云创建 KD-Tree 索引，支持后续的半径邻域搜索
 */
InteractiveKeyFrame::InteractiveKeyFrame(const std::string& directory, g2o::HyperGraph* graph)
    : KeyFrame(directory, graph) {
    // 创建并配置 KD-Tree，将点云作为输入
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZI>());
    kdtree->setInputCloud(cloud);
    kdtree_ = kdtree;  // 使用 boost::any 存储以隐藏模板类型
}

InteractiveKeyFrame::~InteractiveKeyFrame() {}

/**
 * @brief 在关键帧局部坐标系下查找指定点的半径邻域
 * @param pt     全局坐标系下的三维点
 * @param radius 搜索半径（单位：米）
 * @return 邻域点在点云中的索引列表
 *
 * 实现步骤：
 *   1. 从 kdtree_ 中还原 KD-Tree 指针
 *   2. 将查询点从全局坐标系变换到该关键帧的局部坐标系：
 *      使用 g2o 顶点估计值的逆变换
 *   3. 在 KD-Tree 中执行半径搜索
 *   4. 返回匹配点的索引数组
 */
std::vector<int> InteractiveKeyFrame::neighbors(const Eigen::Vector3f& pt, double radius) {
    // 从 boost::any 中还原 KD-Tree 指针
    pcl::search::KdTree<pcl::PointXYZI>::Ptr kdtree =
        boost::any_cast<pcl::search::KdTree<pcl::PointXYZI>::Ptr>(kdtree_);

    std::vector<int> indices;              ///< 搜索结果的索引列表
    std::vector<float> squared_distances;  ///< 搜索结果的平方距离列表

    // 将查询点变换到局部坐标系
    pcl::PointXYZI p;
    // 计算公式：局部坐标 = (顶点估计位姿)^{-1} * 全局坐标
    p.getVector4fMap() = node->estimate().inverse().cast<float>() *
                         Eigen::Vector4f(pt[0], pt[1], pt[2], 1.0f);
    // 执行 KD-Tree 半径搜索
    kdtree->radiusSearch(p, radius, indices, squared_distances);

    return indices;
}

/**
 * @brief 获取关键帧点云的法向量（延迟计算，结果缓存）
 * @return 法向量点云指针
 *
 * 实现步骤：
 *   1. 检查缓存 normals_，如果已计算则直接返回
 *   2. 创建法向量估计器 NormalEstimation（含 OpenMP 加速）
 *   3. 创建 KD-Tree 作为搜索方法
 *   4. 设置输入点云和搜索半径 0.25m
 *   5. 执行法向量估计
 *   6. 缓存结果并返回
 */
pcl::PointCloud<pcl::Normal>::Ptr InteractiveKeyFrame::normals() {
    if (!normals_) {
        // 首次调用，执行法向量计算
        normals_.reset(new pcl::PointCloud<pcl::Normal>());

        // 创建法向量估计器（使用 OpenMP 并行加速）
        pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>());

        ne.setInputCloud(cloud);       ///< 设置输入点云
        ne.setSearchMethod(tree);      ///< 设置 KD-Tree 搜索方法
        ne.setRadiusSearch(0.25f);     ///< 法向量估计的邻域搜索半径 0.25m
        ne.compute(*normals_);         ///< 执行法向量计算
    }

    return normals_;
}

}  // namespace hdl_graph_slam
