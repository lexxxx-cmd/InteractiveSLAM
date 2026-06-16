#pragma once

#include <vector>
#include <Eigen/Core>
#include <glk/glsl_shader.hpp>

namespace glk {

class Drawable {
public:
    virtual ~Drawable() {}
    virtual void draw(glk::GLSLShader& shader) const = 0;
};

}  // namespace glk
