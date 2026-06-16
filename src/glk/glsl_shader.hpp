#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include <QFile>
#include <Eigen/Core>

#include "glk/gl_context.hpp"

namespace glk {

class GLSLShader {
public:
    GLSLShader() {}
    ~GLSLShader();

    bool init(const QString& vertRes, const QString& fragRes);
    bool init(const std::string& vertSrc, const std::string& fragSrc);

    void use() const { glk::gl()->glUseProgram(shader_program); }

    GLint attrib(const std::string& name);
    GLint uniform(const std::string& name);

    void set_uniform(const std::string& name, int value);
    void set_uniform(const std::string& name, float value);
    void set_uniform(const std::string& name, const Eigen::Vector2f& vector);
    void set_uniform(const std::string& name, const Eigen::Vector3f& vector);
    void set_uniform(const std::string& name, const Eigen::Vector4f& vector);
    void set_uniform(const std::string& name, const Eigen::Vector4i& vector);
    void set_uniform(const std::string& name, const Eigen::Matrix4f& matrix);

private:
    GLuint read_shader_from_source(const std::string& source, GLuint shader_type);

    GLuint shader_program = 0;
    std::unordered_map<std::string, GLint> attrib_cache;
    std::unordered_map<std::string, GLint> uniform_cache;
};

}  // namespace glk
