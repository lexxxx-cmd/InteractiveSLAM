#include "viewport/vertex_view.hpp"
#include "viewport/keyframe_view.hpp"
#include <g2o/types/slam3d/vertex_se3.h>

namespace hdl_graph_slam {

VertexView::Ptr VertexView::create(g2o::HyperGraph::Vertex* vertex) {
    // SE3-only: only handle VertexSE3 type.
    // Plane vertices (VertexPlane) are removed from this phase.
    if (dynamic_cast<g2o::VertexSE3*>(vertex)) {
        return std::make_shared<VertexView>(vertex);
    }
    return nullptr;
}

}  // namespace hdl_graph_slam
