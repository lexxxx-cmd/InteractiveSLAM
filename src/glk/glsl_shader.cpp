#include "glk/glsl_shader.hpp"
#include <iostream>
#include <sstream>

namespace glk {

GLSLShader::~GLSLShader() {
    if (shader_program) {
        glk::gl()->glDeleteProgram(shader_program);
    }
}

bool GLSLShader::init(const QString& vertRes, const QString& fragRes) {
    QFile vertFile(vertRes), fragFile(fragRes);
    if (!vertFile.open(QIODevice::ReadOnly) || !fragFile.open(QIODevice::ReadOnly)) {
        std::cerr << "GLSLShader: failed to open shader files" << std::endl;
        return false;
    }
    return init(vertFile.readAll().toStdString(), fragFile.readAll().toStdString());
}

bool GLSLShader::init(const std::string& vertSrc, const std::string& fragSrc) {
    if (shader_program) {
        glk::gl()->glDeleteProgram(shader_program);
        attrib_cache.clear();
        uniform_cache.clear();
    }

    GLuint vert = read_shader_from_source(vertSrc, GL_VERTEX_SHADER);
    GLuint frag = read_shader_from_source(fragSrc, GL_FRAGMENT_SHADER);
    if (!vert || !frag) return false;

    auto* f = glk::gl();
    shader_program = f->glCreateProgram();
    f->glAttachShader(shader_program, vert);
    f->glAttachShader(shader_program, frag);
    f->glLinkProgram(shader_program);

    GLint linked = 0;
    f->glGetProgramiv(shader_program, GL_LINK_STATUS, &linked);
    if (!linked) {
        GLint len = 0;
        f->glGetProgramiv(shader_program, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> buf(len);
        f->glGetProgramInfoLog(shader_program, len, nullptr, buf.data());
        std::cerr << "GLSLShader: link error: " << buf.data() << std::endl;
        f->glDeleteProgram(shader_program);
        shader_program = 0;
        return false;
    }

    f->glDeleteShader(vert);
    f->glDeleteShader(frag);
    return true;
}

GLuint GLSLShader::read_shader_from_source(const std::string& source, GLuint shader_type) {
    auto* f = glk::gl();
    GLuint shader = f->glCreateShader(shader_type);
    const char* src = source.c_str();
    f->glShaderSource(shader, 1, &src, nullptr);
    f->glCompileShader(shader);

    GLint compiled = 0;
    f->glGetShaderiv(shader, GL_COMPILE_STATUS, &compiled);
    if (!compiled) {
        GLint len = 0;
        f->glGetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
        std::vector<char> buf(len);
        f->glGetShaderInfoLog(shader, len, nullptr, buf.data());
        std::cerr << "GLSLShader: compile error (" << shader_type << "): " << buf.data() << std::endl;
        f->glDeleteShader(shader);
        return 0;
    }
    return shader;
}

GLint GLSLShader::attrib(const std::string& name) {
    auto it = attrib_cache.find(name);
    if (it != attrib_cache.end()) return it->second;
    GLint loc = glk::gl()->glGetAttribLocation(shader_program, name.c_str());
    attrib_cache[name] = loc;
    return loc;
}

GLint GLSLShader::uniform(const std::string& name) {
    auto it = uniform_cache.find(name);
    if (it != uniform_cache.end()) return it->second;
    GLint loc = glk::gl()->glGetUniformLocation(shader_program, name.c_str());
    uniform_cache[name] = loc;
    return loc;
}

void GLSLShader::set_uniform(const std::string& name, int value)       { glk::gl()->glUniform1i(uniform(name), value); }
void GLSLShader::set_uniform(const std::string& name, float value)     { glk::gl()->glUniform1f(uniform(name), value); }
void GLSLShader::set_uniform(const std::string& name, const Eigen::Vector2f& v) { glk::gl()->glUniform2fv(uniform(name), 1, v.data()); }
void GLSLShader::set_uniform(const std::string& name, const Eigen::Vector3f& v) { glk::gl()->glUniform3fv(uniform(name), 1, v.data()); }
void GLSLShader::set_uniform(const std::string& name, const Eigen::Vector4f& v) { glk::gl()->glUniform4fv(uniform(name), 1, v.data()); }
void GLSLShader::set_uniform(const std::string& name, const Eigen::Vector4i& v) { glk::gl()->glUniform4iv(uniform(name), 1, v.data()); }
void GLSLShader::set_uniform(const std::string& name, const Eigen::Matrix4f& m) { glk::gl()->glUniformMatrix4fv(uniform(name), 1, GL_FALSE, m.data()); }

}  // namespace glk
