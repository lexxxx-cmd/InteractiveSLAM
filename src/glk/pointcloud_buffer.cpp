#include "glk/pointcloud_buffer.hpp"
#include <GL/gl.h>

namespace glk {

PointCloudBuffer::PointCloudBuffer(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& cloud)
    : m_num_points(static_cast<int>(cloud->size())) {

    auto* f = glk::gl();
    stride = sizeof(pcl::PointXYZI);

    f->glGenVertexArrays(1, &vao);
    f->glBindVertexArray(vao);

    f->glGenBuffers(1, &vbo);
    f->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    f->glBufferData(GL_ARRAY_BUFFER, cloud->size() * stride,
                    cloud->points.data(), GL_STATIC_DRAW);

    f->glBindVertexArray(0);
}

PointCloudBuffer::~PointCloudBuffer() {
    auto* f = glk::gl();
    f->glDeleteBuffers(1, &vbo);
    f->glDeleteVertexArrays(1, &vao);
}

void PointCloudBuffer::draw(glk::GLSLShader& shader) {
    if (m_num_points == 0) return;

    auto* f = glk::gl();
    f->glBindVertexArray(vao);
    f->glBindBuffer(GL_ARRAY_BUFFER, vbo);

    f->glVertexAttribPointer(shader.attrib("vert_position"), 3, GL_FLOAT, GL_FALSE,
                             stride, nullptr);
    f->glEnableVertexAttribArray(shader.attrib("vert_position"));

    f->glVertexAttribPointer(shader.attrib("vert_color"), 4, GL_UNSIGNED_BYTE, GL_TRUE,
                             stride, (void*)(4 * sizeof(float)));
    f->glEnableVertexAttribArray(shader.attrib("vert_color"));

    f->glDrawArrays(GL_POINTS, 0, m_num_points);

    f->glDisableVertexAttribArray(shader.attrib("vert_position"));
    f->glDisableVertexAttribArray(shader.attrib("vert_color"));
    f->glBindBuffer(GL_ARRAY_BUFFER, 0);
    f->glBindVertexArray(0);
}

}  // namespace glk
