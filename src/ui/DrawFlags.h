#pragma once

namespace hdl_graph_slam {

/// @brief Draw flags controlling what is rendered in the 3D viewport.
///        Mirrors the flags originally in the deleted drawable_object.hpp.
struct DrawFlags {
    bool draw_verticies        = true;
    bool draw_edges            = true;
    bool draw_keyframe_vertices = true;
    bool draw_se3_edges        = true;
};

}  // namespace hdl_graph_slam
