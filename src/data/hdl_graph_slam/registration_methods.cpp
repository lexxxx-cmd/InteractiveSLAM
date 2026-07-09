/**
 * @file registration_methods.cpp
 * @brief 点云配准方法工厂的实现
 *
 * 该文件实现了 RegistrationMethods 类的工厂方法 method()，
 * 根据当前选中的方法索引创建对应的点云配准器实例。
 *
 * 支持的配准方法：
 *   - ICP (0)：标准迭代最近点算法
 *   - GICP (1)：广义迭代最近点算法（默认）
 *   - NDT (2)：正态分布变换算法
 *   - GICP_OMP (3)：OpenMP 并行加速的 GICP
 *   - NDT_OMP (4)：OpenMP 并行加速的 NDT
 *   - VGICP (5)：体素化加速的 GICP
 *
 * 方法 3-5 需要外部依赖库的支持，在编译时通过宏定义开关。
 */
#include "data/hdl_graph_slam/registration_methods.hpp"

#include <iostream>
#include <stdexcept>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/ndt.h>

#ifdef HAS_PCLOMP
#include <pclomp/gicp_omp.h>
#include <pclomp/ndt_omp.h>
#endif

#ifdef HAS_FAST_GICP
#include <fast_gicp/gicp/fast_vgicp.hpp>
#endif

namespace hdl_graph_slam {

/**
 * @brief 构造函数：初始化配准参数和方法列表
 *
 * 默认配置：
 *   - 配准方法：GICP（索引 1）
 *   - NDT 分辨率：2.0m
 *   - 变换收敛阈值：1e-4
 *   - 最大迭代次数：64
 */
RegistrationMethods::RegistrationMethods()
    : registration_method(1),           // 默认使用 GICP
      registration_resolution(2.0f),    // NDT 网格分辨率（米）
      transformation_epsilon(1e-4),     // 变换收敛阈值
      max_iterations(64),               // 最大迭代次数
      registration_methods({"ICP", "GICP", "NDT", "GICP_OMP", "NDT_OMP", "VGICP"})
{}

RegistrationMethods::~RegistrationMethods() {}

/**
 * @brief 工厂方法：创建配准器实例
 * @return 配置好的配准器共享指针
 * @throws std::runtime_error 如果所选方法在编译时未启用
 *
 * 创建流程：
 *   1. 根据 registration_method 索引选择配准算法
 *   2. 创建对应的配准器实例
 *   3. 设置 NDT 方法的分辨率（如适用）
 *   4. 设置所有方法的通用参数（收敛阈值和最大迭代次数）
 *
 * 注意：
 *   - 对于方法 0（ICP），标准 ICP 不设置分辨率，直接使用默认参数
 *   - 默认情况下（方法索引不在 0-5 范围内），回退到 GICP
 *   - GICP_OMP 和 NDT_OMP 需要 HAS_PCLOMP 编译标志
 *   - VGICP 需要 HAS_FAST_GICP 编译标志
 *   缺少依赖时会抛出异常，由调用者处理。
 */
pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr
RegistrationMethods::method() const
{
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;

    switch (registration_method) {
    case 0: {  // ICP：标准迭代最近点算法
        auto icp = pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr(
            new pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
        registration = icp;
    } break;

    default:
        // 未知方法索引，回退到 GICP 并发出警告
        std::cerr << "warning: unknown registration method, falling back to GICP" << std::endl;
    case 1: {  // GICP：广义迭代最近点算法（默认）
        auto gicp = pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr(
            new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
        registration = gicp;
    } break;

    case 2: {  // NDT：正态分布变换算法
        auto ndt = pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr(
            new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
        ndt->setResolution(registration_resolution);  // 设置网格分辨率
        registration = ndt;
    } break;

    case 3: {  // GICP_OMP：OpenMP 加速的 GICP
#ifdef HAS_PCLOMP
        auto gicp = pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr(
            new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
        registration = gicp;
#else
        throw std::runtime_error(
            "GICP_OMP is not available. "
            "To enable: git clone ndt_omp and rebuild with -DHAS_PCLOMP");
#endif
    } break;

    case 4: {  // NDT_OMP：OpenMP 加速的 NDT
#ifdef HAS_PCLOMP
        auto ndt = pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr(
            new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
        ndt->setResolution(registration_resolution);  // 设置网格分辨率
        registration = ndt;
#else
        throw std::runtime_error(
            "NDT_OMP is not available. "
            "To enable: git clone ndt_omp and rebuild with -DHAS_PCLOMP");
#endif
    } break;

    case 5: {  // VGICP：体素化 GICP（基于 fast_gicp）
#ifdef HAS_FAST_GICP
        auto vgicp = fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>::Ptr(
            new fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>());
        vgicp->setNumThreads(0);              // 0 表示使用所有可用线程
        vgicp->setResolution(1.0);            // 体素分辨率 1.0m
        vgicp->setCorrespondenceRandomness(20);  // 对应点随机采样数
        registration = vgicp;
#else
        throw std::runtime_error(
            "VGICP is not available. "
            "To enable: git clone fast_gicp and rebuild with -DHAS_FAST_GICP");
#endif
    } break;
    }

    // 设置所有配准方法的通用参数
    registration->setTransformationEpsilon(transformation_epsilon);  // 变换收敛阈值
    registration->setMaximumIterations(max_iterations);              // 最大迭代次数
    return registration;
}

}  // namespace hdl_graph_slam
