#pragma once

#include <memory>
#include <Eigen/Core>
#include "glk/glsl_shader.hpp"

namespace hdl_graph_slam {

/// SE3-only DrawFlags — plane-related fields removed.
struct DrawFlags {
    bool draw_verticies = true;
    bool draw_edges = true;
    bool draw_keyframe_vertices = true;
    bool draw_se3_edges = true;
    bool z_clipping = true;
    int  selectedVertexId = -1;   // -1 = no selection; set by GraphViewport
};

class DrawableObject {
public:
    enum OBJECT_TYPE {
        POINTS   = 1 << 0,
        VERTEX   = 1 << 1,
        EDGE     = 1 << 2,
        KEYFRAME = 1 << 3,
    };

    using Ptr = std::shared_ptr<DrawableObject>;

    virtual ~DrawableObject() {}
    virtual bool available() const { return true; }
    virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader) = 0;
    virtual void draw(const DrawFlags& flags, glk::GLSLShader& shader,
                      const Eigen::Vector4f& color,
                      const Eigen::Matrix4f& model_matrix) {}
};

}  // namespace hdl_graph_slam
