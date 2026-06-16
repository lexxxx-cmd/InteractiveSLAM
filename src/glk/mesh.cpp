#include "glk/mesh.hpp"
#include <GL/gl.h>

namespace glk {

Mesh::Mesh(const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices,
           const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& normals,
           const std::vector<int>& indices,
           GLenum drawMode)
    : num_vertices(static_cast<int>(vertices.size())),
      num_indices(static_cast<int>(indices.size())),
      m_drawMode(drawMode) {

    auto* f = glk::gl();
    f->glGenVertexArrays(1, &vao);
    f->glBindVertexArray(vao);

    f->glGenBuffers(1, &vbo);
    f->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    f->glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Eigen::Vector3f),
                    vertices.data(), GL_STATIC_DRAW);

    f->glGenBuffers(1, &nbo);
    f->glBindBuffer(GL_ARRAY_BUFFER, nbo);
    f->glBufferData(GL_ARRAY_BUFFER, normals.size() * sizeof(Eigen::Vector3f),
                    normals.data(), GL_STATIC_DRAW);

    f->glGenBuffers(1, &ebo);
    f->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    f->glBufferData(GL_ELEMENT_ARRAY_BUFFER, indices.size() * sizeof(int),
                    indices.data(), GL_STATIC_DRAW);

    f->glBindVertexArray(0);
}

Mesh::~Mesh() {
    auto* f = glk::gl();
    f->glDeleteBuffers(1, &vbo);
    f->glDeleteBuffers(1, &nbo);
    f->glDeleteBuffers(1, &ebo);
    f->glDeleteVertexArrays(1, &vao);
}

Mesh::Mesh(Mesh&& other) noexcept
    : num_vertices(other.num_vertices)
    , num_indices(other.num_indices)
    , m_drawMode(other.m_drawMode)
    , vao(other.vao)
    , vbo(other.vbo)
    , nbo(other.nbo)
    , ebo(other.ebo)
{
    // Null out source to prevent double-deletion of GL resources
    other.vao = 0;
    other.vbo = 0;
    other.nbo = 0;
    other.ebo = 0;
}

void Mesh::draw(glk::GLSLShader& shader) const {
    auto* f = glk::gl();
    f->glBindVertexArray(vao);

    f->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    f->glVertexAttribPointer(shader.attrib("vert_position"), 3, GL_FLOAT, GL_FALSE, 0, nullptr);
    f->glEnableVertexAttribArray(shader.attrib("vert_position"));

    // Bind nbo (normals/colors) as vert_color.
    // Primitives stores per-vertex RGB colors in the "normals" parameter;
    // GL automatically pads the missing alpha to 1.0 for the shader's vec4.
    GLint colorLoc = shader.attrib("vert_color");
    if (colorLoc >= 0) {
        f->glBindBuffer(GL_ARRAY_BUFFER, nbo);
        f->glVertexAttribPointer(colorLoc, 3, GL_FLOAT, GL_FALSE, 0, nullptr);
        f->glEnableVertexAttribArray(colorLoc);
    }

    f->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo);
    f->glDrawElements(m_drawMode, num_indices, GL_UNSIGNED_INT, nullptr);

    f->glDisableVertexAttribArray(shader.attrib("vert_position"));
    if (colorLoc >= 0) f->glDisableVertexAttribArray(colorLoc);

    f->glBindBuffer(GL_ARRAY_BUFFER, 0);
    f->glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
    f->glBindVertexArray(0);
}

}  // namespace glk
