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

RegistrationMethods::RegistrationMethods()
    : registration_method(1),           // 默认 GICP
      registration_resolution(2.0f),
      transformation_epsilon(1e-4),
      max_iterations(64),
      registration_methods({"ICP", "GICP", "NDT", "GICP_OMP", "NDT_OMP", "VGICP"})
{}

RegistrationMethods::~RegistrationMethods() {}

pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr
RegistrationMethods::method() const
{
    pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr registration;

    switch (registration_method) {
    case 0: {  // ICP
        auto icp = pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr(
            new pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
        registration = icp;
    } break;

    default:
        std::cerr << "warning: unknown registration method, falling back to GICP" << std::endl;
    case 1: {  // GICP
        auto gicp = pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>::Ptr(
            new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI>());
        registration = gicp;
    } break;

    case 2: {  // NDT
        auto ndt = pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr(
            new pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
        ndt->setResolution(registration_resolution);
        registration = ndt;
    } break;

    case 3: {  // GICP_OMP — 占位符，后续引入 ndt_omp
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

    case 4: {  // NDT_OMP — 占位符，后续引入 ndt_omp
#ifdef HAS_PCLOMP
        auto ndt = pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>::Ptr(
            new pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>());
        ndt->setResolution(registration_resolution);
        registration = ndt;
#else
        throw std::runtime_error(
            "NDT_OMP is not available. "
            "To enable: git clone ndt_omp and rebuild with -DHAS_PCLOMP");
#endif
    } break;

    case 5: {  // VGICP — 占位符，后续引入 fast_gicp
#ifdef HAS_FAST_GICP
        auto vgicp = fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>::Ptr(
            new fast_gicp::FastVGICP<pcl::PointXYZI, pcl::PointXYZI>());
        vgicp->setNumThreads(0);
        vgicp->setResolution(1.0);
        vgicp->setCorrespondenceRandomness(20);
        registration = vgicp;
#else
        throw std::runtime_error(
            "VGICP is not available. "
            "To enable: git clone fast_gicp and rebuild with -DHAS_FAST_GICP");
#endif
    } break;
    }

    registration->setTransformationEpsilon(transformation_epsilon);
    registration->setMaximumIterations(max_iterations);
    return registration;
}

}  // namespace hdl_graph_slam
