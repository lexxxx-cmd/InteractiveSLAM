#pragma once

#include <vector>
#include <Eigen/Core>
#include "glk/drawble.hpp"
#include "glk/glsl_shader.hpp"
#include "glk/gl_context.hpp"

namespace glk {

class Lines : public Drawable {
public:
    Lines(float line_width,
          const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices,
          const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& colors = {},
          const std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>>& infos = {});
    virtual ~Lines();

    virtual void draw(glk::GLSLShader& shader) const override;

private:
    int num_vertices;
    int num_indices;

    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint cbo = 0;
    GLuint ibo = 0;
    GLuint ebo = 0;
};

}  // namespace glk
