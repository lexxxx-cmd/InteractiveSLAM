#pragma once

#include <vector>
#include <Eigen/Core>
#include "glk/lines.hpp"
#include "glk/glsl_shader.hpp"

namespace hdl_graph_slam {

/// CPU-side line batch collector. Edge views add segments each frame,
/// then draw() uploads all collected lines to GPU and renders them.
class LineBuffer {
public:
    void clear() {
        vertices.clear();
        colors.clear();
        infos.clear();
    }

    void add_line(const Eigen::Vector3f& v0, const Eigen::Vector3f& v1,
                  const Eigen::Vector4f& c0, const Eigen::Vector4f& c1,
                  const Eigen::Vector4i& info) {
        vertices.push_back(v0);
        vertices.push_back(v1);
        colors.push_back(c0);
        colors.push_back(c1);
        infos.push_back(info);
        infos.push_back(info);
    }

    void draw(glk::GLSLShader& shader) const {
        if (vertices.empty()) return;
        shader.set_uniform("color_mode", 2);
        Eigen::Matrix4f identity_m = Eigen::Matrix4f::Identity();
        shader.set_uniform("model_matrix", identity_m);
        glk::Lines lines(0.1f, vertices, colors, infos);
        lines.draw(shader);
    }

private:
    std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>> vertices;
    std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>> colors;
    std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>> infos;
};

}  // namespace hdl_graph_slam
