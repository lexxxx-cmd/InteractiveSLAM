#ifndef HDL_GRAPH_SLAM_REGISTRATION_METHODS_HPP
#define HDL_GRAPH_SLAM_REGISTRATION_METHODS_HPP

#include <string>
#include <vector>
#include <memory>
#include <pcl/point_types.h>
#include <pcl/registration/registration.h>

namespace hdl_graph_slam {

class RegistrationMethods {
public:
  RegistrationMethods();
  ~RegistrationMethods();

  // Factory: returns registration instance by current method index
  pcl::Registration<pcl::PointXYZI, pcl::PointXYZI>::Ptr method() const;

  // Parameter getters/setters
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
  int registration_method;
  float registration_resolution;
  float transformation_epsilon;
  int max_iterations;
  std::vector<const char*> registration_methods;
};

}  // namespace hdl_graph_slam

#endif
