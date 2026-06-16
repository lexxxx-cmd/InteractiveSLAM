#include "viewport/edge_view.hpp"
#include "viewport/edge_se3_view.hpp"
#include <g2o/types/slam3d/edge_se3.h>

namespace hdl_graph_slam {

EdgeView::Ptr EdgeView::create(g2o::HyperGraph::Edge* edge, LineBuffer& line_buffer) {
    // SE3-only: only handle EdgeSE3 type
    if (dynamic_cast<g2o::EdgeSE3*>(edge)) {
        return std::make_shared<EdgeSE3View>(edge, line_buffer);
    }
    // Plane-related edges (EdgeSE3Plane, EdgePlane) are removed in this phase
    return nullptr;
}

}  // namespace hdl_graph_slam
