#pragma once

#include <vector>
#include <Eigen/Core>
#include "glk/drawble.hpp"
#include "glk/glsl_shader.hpp"
#include "glk/gl_context.hpp"

namespace glk {

class Mesh : public Drawable {
public:
    Mesh(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices,
         const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& normals,
         const std::vector<int>& indices);
    virtual ~Mesh();

    virtual void draw(glk::GLSLShader& shader) const override;

private:
    int num_vertices;
    int num_indices;
    GLuint vao = 0;
    GLuint vbo = 0;
    GLuint nbo = 0;
    GLuint ebo = 0;
};

}  // namespace glk
