#include "glk/lines.hpp"
#include <GL/gl.h>

namespace glk {

Lines::Lines(float /*line_width*/,
             const std::vector<Eigen::Vector3f, Eigen::aligned_allocator<Eigen::Vector3f>>& vertices,
             const std::vector<Eigen::Vector4f, Eigen::aligned_allocator<Eigen::Vector4f>>& colors,
             const std::vector<Eigen::Vector4i, Eigen::aligned_allocator<Eigen::Vector4i>>& infos)
    : num_vertices(static_cast<int>(vertices.size())),
      num_indices(num_vertices) {  // GL_LINES: each pair = one line segment

    auto* f = glk::gl();

    f->glGenVertexArrays(1, &vao);
    f->glBindVertexArray(vao);

    // Vertex positions
    f->glGenBuffers(1, &vbo);
    f->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    f->glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(Eigen::Vector3f),
                    vertices.data(), GL_DYNAMIC_DRAW);

    // Colors (optional)
    if (!colors.empty()) {
        f->glGenBuffers(1, &cbo);
        f->glBindBuffer(GL_ARRAY_BUFFER, cbo);
        f->glBufferData(GL_ARRAY_BUFFER, colors.size() * sizeof(Eigen::Vector4f),
                        colors.data(), GL_DYNAMIC_DRAW);
    }

    // Info IDs (optional)
    if (!infos.empty()) {
        f->glGenBuffers(1, &ibo);
        f->glBindBuffer(GL_ARRAY_BUFFER, ibo);
        f->glBufferData(GL_ARRAY_BUFFER, infos.size() * sizeof(Eigen::Vector4i),
                        infos.data(), GL_DYNAMIC_DRAW);
    }

    f->glBindVertexArray(0);
}

Lines::~Lines() {
    auto* f = glk::gl();
    f->glDeleteBuffers(1, &vbo);
    if (cbo) f->glDeleteBuffers(1, &cbo);
    if (ibo) f->glDeleteBuffers(1, &ibo);
    f->glDeleteVertexArrays(1, &vao);
}

void Lines::draw(glk::GLSLShader& shader) const {
    auto* f = glk::gl();
    f->glBindVertexArray(vao);

    f->glBindBuffer(GL_ARRAY_BUFFER, vbo);
    f->glVertexAttribPointer(shader.attrib("vert_position"), 3, GL_FLOAT, GL_FALSE,
                             0, nullptr);
    f->glEnableVertexAttribArray(shader.attrib("vert_position"));

    if (cbo) {
        f->glBindBuffer(GL_ARRAY_BUFFER, cbo);
        f->glVertexAttribPointer(shader.attrib("vert_color"), 4, GL_FLOAT, GL_FALSE,
                                 0, nullptr);
        f->glEnableVertexAttribArray(shader.attrib("vert_color"));
    }

    if (ibo) {
        f->glBindBuffer(GL_ARRAY_BUFFER, ibo);
        f->glVertexAttribIPointer(shader.attrib("vert_info"), 4, GL_INT, 0, nullptr);
        f->glEnableVertexAttribArray(shader.attrib("vert_info"));
    }

    f->glDrawArrays(GL_LINES, 0, num_vertices);

    f->glDisableVertexAttribArray(shader.attrib("vert_position"));
    f->glDisableVertexAttribArray(shader.attrib("vert_color"));
    f->glDisableVertexAttribArray(shader.attrib("vert_info"));
    f->glBindVertexArray(0);
}

}  // namespace glk
