#pragma once

#include <memory>
#include <Eigen/Core>
#include <g2o/core/hyper_graph.h>
#include "viewport/drawable_object.hpp"

namespace hdl_graph_slam {

class VertexView : public DrawableObject {
public:
    using Ptr = std::shared_ptr<VertexView>;

    VertexView(g2o::HyperGraph::Vertex* vertex) : vertex(vertex) {}
    virtual ~VertexView() {}

    static VertexView::Ptr create(g2o::HyperGraph::Vertex* vertex);

    long id() const { return vertex->id(); }

    // Pure virtual from DrawableObject — default empty (KeyFrameView overrides)
    void draw(const DrawFlags&, glk::GLSLShader&) override {}

protected:
    g2o::HyperGraph::Vertex* vertex;
};

}  // namespace hdl_graph_slam
