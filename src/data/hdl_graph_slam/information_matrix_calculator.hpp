/**
 * @file information_matrix_calculator.hpp
 * @brief 信息矩阵计算器定义
 *
 * InformationMatrixCalculator 负责计算图优化中边（Edge）的信息矩阵。
 * 信息矩阵是协方差矩阵的逆，表示约束的置信度/精度。
 *
 * 核心功能：
 *   1. 根据点云匹配质量动态计算信息矩阵
 *   2. 使用适应度分数（fitness score）评估匹配质量
 *   3. 支持常量信息矩阵和自适应信息矩阵两种模式
 *   4. 提供静态方法 calc_fitness_score 用于外部计算适应度
 *
 * 自适应策略：
 *   匹配质量越好（适应度低）→ 信息矩阵值越大 → 约束越强
 *   匹配质量越差（适应度高）→ 信息矩阵值越小 → 约束越弱
 *   使用 sigmoid-like 权重函数实现平滑过渡
 *
 * 本版本移除了 ROS 依赖（ros::NodeHandle 构造函数），
 * 改用模板化的 load() 方法与 ParameterServer 配合使用。
 */
#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace hdl_graph_slam {

/**
 * @brief 信息矩阵计算器
 *
 * 该类用于计算图优化中 SE3 约束边的信息矩阵。
 * 支持两种模式：
 *   1. 常量模式（use_const_inf_matrix=true）：使用固定的标准差
 *   2. 自适应模式（默认）：根据点云匹配的适应度分数动态调整
 *
 * 模板方法 load() 可以与任意参数服务器配合使用，
 * 只需提供 param<T>(name, default_value) 接口即可。
 */
class InformationMatrixCalculator {
public:
    using PointT = pcl::PointXYZI;  ///< 点云点类型

    InformationMatrixCalculator() {}
    ~InformationMatrixCalculator() {}

    /**
     * @brief 从参数服务器加载配置
     * @param params 参数服务器对象（需提供 template param<T> 方法）
     *
     * 配置参数说明：
     *   - use_const_inf_matrix：是否使用常量信息矩阵
     *   - const_stddev_x：常量模式下的平移标准差
     *   - const_stddev_q：常量模式下的旋转标准差
     *   - var_gain_a：自适应权重函数的增益系数
     *   - min_stddev_x / max_stddev_x：平移标准差的范围
     *   - min_stddev_q / max_stddev_q：旋转标准差的范围
     *   - fitness_score_thresh：适应度阈值
     */
    template<typename ParamServer>
    void load(ParamServer& params) {
        use_const_inf_matrix = params.template param<bool>("use_const_inf_matrix", false);
        const_stddev_x = params.template param<double>("const_stddev_x", 0.5);
        const_stddev_q = params.template param<double>("const_stddev_q", 0.1);

        var_gain_a = params.template param<double>("var_gain_a", 3.0);
        min_stddev_x = params.template param<double>("min_stddev_x", 0.1);
        max_stddev_x = params.template param<double>("max_stddev_x", 0.2);
        min_stddev_q = params.template param<double>("min_stddev_q", 0.05);
        max_stddev_q = params.template param<double>("max_stddev_q", 0.1);
        fitness_score_thresh = params.template param<double>("fitness_score_thresh", 2.5);
    }

    /**
     * @brief 计算两个点云之间的适应度分数（静态方法）
     * @param cloud1  源点云（作为 KD-Tree 目标）
     * @param cloud2  目标点云（将被变换到源坐标系）
     * @param relpose 从 cloud2 到 cloud1 的变换
     * @param max_range 最大匹配距离（超过此距离的点对不计入）
     * @return 平均适应度分数（越小表示匹配越好）
     *
     * 计算方式：
     *   1. 将 cloud2 按 relpose 变换到 cloud1 坐标系
     *   2. 对每个变换后的点，在 cloud1 的 KD-Tree 中查找最近邻
     *   3. 计算最近邻距离，仅统计距离小于 max_range 的点
     *   4. 返回平均距离
     *
     * 返回值含义：
     *   0.0：完美匹配
     *   越大：匹配质量越差
     *   double::max()：没有点对在 max_range 内
     */
    static double calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1,
                                     const pcl::PointCloud<PointT>::ConstPtr& cloud2,
                                     const Eigen::Isometry3d& relpose,
                                     double max_range = std::numeric_limits<double>::max());

    /**
     * @brief 计算信息矩阵
     * @param cloud1  源点云
     * @param cloud2  目标点云
     * @param relpose 从 cloud2 到 cloud1 的相对位姿
     * @return 6x6 信息矩阵
     *
     * 返回的 6x6 矩阵结构：
     *   左上 3x3：平移部分信息矩阵
     *   右下 3x3：旋转部分信息矩阵
     *   非对角线为零（假定平移和旋转不相关）
     *
     * 常量模式下直接使用 const_stddev_x 和 const_stddev_q。
     * 自适应模式下根据适应度分数动态调整标准差，使用 weight() 函数：
     *   适应度低 → 标准差小 → 信息矩阵值大（约束强）
     *   适应度高 → 标准差大 → 信息矩阵值小（约束弱）
     */
    Eigen::MatrixXd calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1,
                                            const pcl::PointCloud<PointT>::ConstPtr& cloud2,
                                            const Eigen::Isometry3d& relpose) const;

private:
    /**
     * @brief 自适应权重函数
     * @param a     增益系数（控制曲线陡峭程度）
     * @param max_x 最大输入值（归一化因子）
     * @param min_y 最小输出值（对应最佳匹配）
     * @param max_y 最大输出值（对应最差匹配）
     * @param x     输入值（适应度分数）
     * @return 计算得到的权重（介于 min_y 和 max_y 之间）
     *
     * 公式：y = min_y + (max_y - min_y) * (1 - exp(-a * x)) / (1 - exp(-a * max_x))
     * 这是一个经验公式，x 为适应度分数时：
     *   适应度接近 0 → 输出接近 min_y（小标准差，高置信度）
     *   适应度接近 max_x → 输出接近 max_y（大标准差，低置信度）
     */
    double weight(double a, double max_x, double min_y, double max_y, double x) const {
        double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
        return min_y + (max_y - min_y) * y;
    }

private:
    // ── 常量模式参数 ────────────────────────────────
    bool use_const_inf_matrix = false;   ///< 是否使用常量信息矩阵（默认关闭）
    double const_stddev_x = 0.5;         ///< 常量模式下平移的标准差
    double const_stddev_q = 0.1;         ///< 常量模式下旋转的标准差

    // ── 自适应模式参数 ────────────────────────────────
    double var_gain_a = 3.0;             ///< 自适应增益系数
    double min_stddev_x = 0.1;           ///< 平移标准差最小值（最佳匹配时）
    double max_stddev_x = 0.2;           ///< 平移标准差最大值（最差匹配时）
    double min_stddev_q = 0.05;          ///< 旋转标准差最小值
    double max_stddev_q = 0.1;           ///< 旋转标准差最大值
    double fitness_score_thresh = 2.5;   ///< 适应度分数阈值
};

}  // namespace hdl_graph_slam
