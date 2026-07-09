/**
 * @file registration_methods.hpp
 * @brief 点云配准方法工厂定义
 *
 * RegistrationMethods 管理多种点云配准（Registration）算法，
 * 提供统一的工厂接口来创建不同类型的配准器实例。
 *
 * 支持的配准方法：
 *   0: ICP  - 迭代最近点（Iterative Closest Point）
 *   1: GICP - 广义迭代最近点（Generalized ICP）
 *   2: NDT - 正态分布变换（Normal Distributions Transform）
 *   3: GICP_OMP - OpenMP 加速的 GICP（需 ndt_omp 库）
 *   4: NDT_OMP - OpenMP 加速的 NDT（需 ndt_omp 库）
 *   5: VGICP - 体素 GICP（需 fast_gicp 库）
 *
 * 使用方法：
 *   1. 通过 set_method_index() 选择配准方法
 *   2. 调用 method() 工厂方法获取配准器实例
 *   3. 使用返回的 pcl::Registration 指针进行配准操作
 */
#ifndef HDL_GRAPH_SLAM_REGISTRATION_METHODS_HPP
#define HDL_GRAPH_SLAM_REGISTRATION_METHODS_HPP

#include <string>
#include <vector>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>

namespace hdl_graph_slam {

/**
 * @brief 点云配准方法工厂类
 *
 * 封装了多种点云配准算法，提供统一的工厂接口。
 * 用于闭环检测和扫描匹配中的点云对齐操作。
 *
 * 特性：
 *   - 工厂模式：根据当前选中的方法索引创建对应的配准器
 *   - 参数统一管理：分辨率、收敛阈值、最大迭代次数
 *   - 可扩展：通过编译宏（HAS_PCLOMP、HAS_FAST_GICP）条件编译可选的方法
 *
 * 可选方法的条件编译：
 *   - GICP_OMP (3) 和 NDT_OMP (4)：需要 ndt_omp 库和 -DHAS_PCLOMP 编译标志
 *   - VGICP (5)：需要 fast_gicp 库和 -DHAS_FAST_GICP 编译标志
 *   如果编译时未启用，调用相应方法会抛出 std::runtime_error
 */
class RegistrationMethods {
public:
    RegistrationMethods();
    ~RegistrationMethods();

    /**
     * @brief 工厂方法：根据当前 method_index 创建对应的配准器实例
     * @return 配准器共享指针
     * @throws std::runtime_error 如果所选方法在编译时未启用
     *
     * 创建后的配准器已预设以下参数：
     *   - TransformationEpsilon：变换收敛阈值
     *   - MaximumIterations：最大迭代次数
     *   - Resolution（NDT类方法）：网格分辨率
     */
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr method() const;

    // ── 参数访问器 ────────────────────────────────────────────────────
    int get_method_index() const { return registration_method; }
    void set_method_index(int idx) { registration_method = idx; }

    float get_resolution() const { return registration_resolution; }
    void set_resolution(float r) { registration_resolution = r; }

    float get_transformation_epsilon() const { return transformation_epsilon; }
    void set_transformation_epsilon(float eps) { transformation_epsilon = eps; }

    int get_max_iterations() const { return max_iterations; }
    void set_max_iterations(int n) { max_iterations = n; }

    const std::vector<const char*>& method_names() const { return registration_methods; }

private:
    int registration_method;              ///< 当前选中的配准方法索引（0=ICP, 1=GICP, ...）
    float registration_resolution;        ///< NDT 方法的网格分辨率（米）
    float transformation_epsilon;         ///< 配准收敛的变换增量阈值
    int max_iterations;                   ///< 配准最大迭代次数
    std::vector<const char*> registration_methods;  ///< 配准方法名称列表
};

}  // namespace hdl_graph_slam

#endif
