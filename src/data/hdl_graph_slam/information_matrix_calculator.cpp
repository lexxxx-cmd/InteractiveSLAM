/**
 * @file information_matrix_calculator.cpp
 * @brief 信息矩阵计算器的实现
 *
 * 该文件实现了信息矩阵计算的两个核心方法：
 *
 * 1. calc_information_matrix()：
 *    根据两个点云之间的匹配质量计算 6x6 信息矩阵。
 *    支持常量模式和自适应模式。
 *
 * 2. calc_fitness_score()（静态方法）：
 *    评估两个点云之间的匹配质量，返回平均近邻距离。
 *    该方法是计算适应度分数的核心算法，可被外部独立调用。
 *
 * 适应度分数计算流程：
 *   1. 将 cloud2 按相对位姿变换到 cloud1 的坐标系
 *   2. 对每个变换后的点，在 cloud1 的 KD-Tree 中找最近邻
 *   3. 计算平均最近邻距离作为适应度分数
 */
#include "data/hdl_graph_slam/information_matrix_calculator.hpp"

#include <pcl/search/kdtree.h>
#include <pcl/common/transforms.h>

namespace hdl_graph_slam {

/**
 * @brief 计算两个点云之间的信息矩阵
 * @param cloud1  源点云
 * @param cloud2  目标点云
 * @param relpose 从 cloud2 到 cloud1 的相对位姿变换
 * @return 6x6 信息矩阵
 *
 * 常量模式（use_const_inf_matrix=true）：
 *   直接返回对角矩阵，平移部分使用 const_stddev_x，旋转部分使用 const_stddev_q。
 *
 * 自适应模式（默认）：
 *   1. 计算两点云的适应度分数
 *   2. 根据适应度计算平移和旋转的等效标准差（使用 weight 函数）
 *   3. 标准差越小 → 方差越小 → 信息矩阵值越大 → 约束越强
 *
 * 注意：信息矩阵 = 协方差矩阵的逆 = 1/方差（对角矩阵情况）
 */
Eigen::MatrixXd InformationMatrixCalculator::calc_information_matrix(
    const pcl::PointCloud<PointT>::ConstPtr& cloud1,
    const pcl::PointCloud<PointT>::ConstPtr& cloud2,
    const Eigen::Isometry3d& relpose) const {
    // 常量模式：直接使用预设的标准差
    if (use_const_inf_matrix) {
        Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
        inf.topLeftCorner(3, 3).array() /= const_stddev_x;   ///< 平移部分：1/标准差
        inf.bottomRightCorner(3, 3).array() /= const_stddev_q; ///< 旋转部分：1/标准差
        return inf;
    }

    // 自适应模式：根据点云匹配质量计算
    double fitness_score = calc_fitness_score(cloud1, cloud2, relpose);

    // 计算平移和旋转的方差范围
    double min_var_x = std::pow(min_stddev_x, 2);  ///< 平移最小方差
    double max_var_x = std::pow(max_stddev_x, 2);  ///< 平移最大方差
    double min_var_q = std::pow(min_stddev_q, 2);  ///< 旋转最小方差
    double max_var_q = std::pow(max_stddev_q, 2);  ///< 旋转最大方差

    // 使用权重函数计算实际方差
    // 适应度低（匹配好）→ 权重接近 min_var（小方差，高置信度）
    // 适应度高（匹配差）→ 权重接近 max_var（大方差，低置信度）
    float w_x = weight(var_gain_a, fitness_score_thresh, min_var_x, max_var_x, fitness_score);
    float w_q = weight(var_gain_a, fitness_score_thresh, min_var_q, max_var_q, fitness_score);

    // 构建信息矩阵（对角矩阵，平移和旋转各 3 维）
    Eigen::MatrixXd inf = Eigen::MatrixXd::Identity(6, 6);
    inf.topLeftCorner(3, 3).array() /= w_x;    ///< 平移部分信息矩阵 = 1/方差
    inf.bottomRightCorner(3, 3).array() /= w_q;  ///< 旋转部分信息矩阵 = 1/方差
    return inf;
}

/**
 * @brief 计算两个点云之间的适应度分数（静态方法）
 * @param cloud1  源点云（作为参考/目标点云，构建 KD-Tree）
 * @param cloud2  目标点云（将被变换到 cloud1 的坐标系）
 * @param relpose 从 cloud2 到 cloud1 的等距变换
 * @param max_range 最大匹配距离阈值，超过此距离的点对不计入
 * @return 平均适应度分数
 *
 * 算法步骤：
 *   1. 为 cloud1 构建 KD-Tree（用于最近邻搜索）
 *   2. 将 cloud2 按 relpose 变换到 cloud1 的坐标系
 *   3. 遍历变换后的每个点，在 cloud1 中找最近邻
 *   4. 统计距离小于 max_range 的所有点对
 *   5. 返回平均最近邻距离
 *
 * 返回值说明：
 *   - 接近 0：两点云高度匹配
 *   - 越大：匹配质量越差
 *   - double::max()：没有点对的距离在 max_range 以内
 *
 * 适用场景：
 *   评估点云配准（ICP/GICP/NDT）的质量，作为是否接受闭环匹配的判据。
 */
double InformationMatrixCalculator::calc_fitness_score(
    const pcl::PointCloud<PointT>::ConstPtr& cloud1,
    const pcl::PointCloud<PointT>::ConstPtr& cloud2,
    const Eigen::Isometry3d& relpose, double max_range) {
    // 为 cloud1 构建 KD-Tree，然后委托给复用树的重载 ——
    // 保持本函数行为与签名不变（既有调用点无需改动），
    // 同时让"cloud1 固定、只变相对位姿"的场景可以跳过建树。
    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>());
    tree->setInputCloud(cloud1);
    return calc_fitness_score_with_tree(tree, cloud2, relpose, max_range);
}

double InformationMatrixCalculator::calc_fitness_score_with_tree(
    const pcl::search::KdTree<PointT>::Ptr& tree,
    const pcl::PointCloud<PointT>::ConstPtr& cloud2,
    const Eigen::Isometry3d& relpose, double max_range) {
    if (!tree) return std::numeric_limits<double>::max();

    double fitness_score = 0.0;

    // 将 cloud2 按相对位姿变换到 cloud1 的坐标系
    pcl::PointCloud<PointT> input_transformed;
    pcl::transformPointCloud(*cloud2, input_transformed, relpose.cast<float>());

    // 最近邻搜索缓冲区
    std::vector<int> nn_indices(1);     ///< 最近邻索引
    std::vector<float> nn_dists(1);     ///< 最近邻距离

    int nr = 0;  // 有效点对计数

    // 遍历变换后的每个点，查找最近邻
    for (size_t i = 0; i < input_transformed.points.size(); ++i) {
        // 在传入的 KD-Tree 中查找最近邻
        tree->nearestKSearch(input_transformed.points[i], 1, nn_indices, nn_dists);

        // 仅统计距离在 max_range 以内的点对
        if (nn_dists[0] <= max_range) {
            fitness_score += nn_dists[0];  // 累加距离
            nr++;                           // 计数加一
        }
    }

    // 返回平均适应度分数
    if (nr > 0)
        return (fitness_score / nr);  // 平均最近邻距离
    else
        return (std::numeric_limits<double>::max());  // 没有有效点对
}

}  // namespace hdl_graph_slam
