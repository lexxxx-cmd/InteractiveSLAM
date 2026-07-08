#pragma once

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace hdl_graph_slam {

/**
 * @brief Information matrix calculator — ROS-free adaptation
 *
 * Removed: ros::NodeHandle constructor (use template load() with ParameterServer instead)
 */
class InformationMatrixCalculator {
public:
    using PointT = pcl::PointXYZI;

    InformationMatrixCalculator() {}
    ~InformationMatrixCalculator() {}

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

    static double calc_fitness_score(const pcl::PointCloud<PointT>::ConstPtr& cloud1,
                                     const pcl::PointCloud<PointT>::ConstPtr& cloud2,
                                     const Eigen::Isometry3d& relpose,
                                     double max_range = std::numeric_limits<double>::max());

    Eigen::MatrixXd calc_information_matrix(const pcl::PointCloud<PointT>::ConstPtr& cloud1,
                                            const pcl::PointCloud<PointT>::ConstPtr& cloud2,
                                            const Eigen::Isometry3d& relpose) const;

private:
    double weight(double a, double max_x, double min_y, double max_y, double x) const {
        double y = (1.0 - std::exp(-a * x)) / (1.0 - std::exp(-a * max_x));
        return min_y + (max_y - min_y) * y;
    }

private:
    bool use_const_inf_matrix = false;
    double const_stddev_x = 0.5;
    double const_stddev_q = 0.1;

    double var_gain_a = 3.0;
    double min_stddev_x = 0.1;
    double max_stddev_x = 0.2;
    double min_stddev_q = 0.05;
    double max_stddev_q = 0.1;
    double fitness_score_thresh = 2.5;
};

}  // namespace hdl_graph_slam
