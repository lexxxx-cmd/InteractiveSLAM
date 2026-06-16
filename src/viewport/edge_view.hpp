#pragma once

#include <memory>
#include <Eigen/Core>
#include <g2o/core/hyper_graph.h>
#include "viewport/drawable_object.hpp"
#include "viewport/line_buffer.hpp"

namespace hdl_graph_slam {

class EdgeView : public DrawableObject {
public:
    using Ptr = std::shared_ptr<EdgeView>;

    static EdgeView::Ptr create(g2o::HyperGraph::Edge* edge, LineBuffer& line_buffer);

    EdgeView(g2o::HyperGraph::Edge* edge, LineBuffer& line_buffer)
        : edge(edge), line_buffer(line_buffer) {}
    virtual ~EdgeView() {}

    long id() const { return edge->id(); }
    virtual Eigen::Vector3f representative_point() const = 0;

    g2o::HyperGraph::Edge* edge;

protected:
    LineBuffer& line_buffer;
};

}  // namespace hdl_graph_slam
