#pragma once

#include <memory>
#include <Eigen/Dense>
#include "glk/glsl_shader.hpp"
#include "glk/gl_context.hpp"

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace glk {

class PointCloudBuffer {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Ptr = std::shared_ptr<PointCloudBuffer>;

    PointCloudBuffer(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud);
    ~PointCloudBuffer();

    void draw(glk::GLSLShader& shader);

    int num_points() const { return m_num_points; }

private:
    GLuint vao = 0;
    GLuint vbo = 0;
    int stride = 0;
    int m_num_points = 0;
};

}  // namespace glk
