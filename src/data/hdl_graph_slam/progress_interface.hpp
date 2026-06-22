#pragma once

#include <string>

namespace hdl_graph_slam {

/// @brief Abstract progress callback interface — framework-agnostic.
///        Replaces the deleted guik::ProgressInterface so the data layer
///        stays free of GUI dependencies.
struct ProgressInterface {
    virtual ~ProgressInterface() = default;
    virtual void set_title(const std::string&) {}
    virtual void set_text(const std::string&) {}
    virtual void set_maximum(int) {}
    virtual void set_current(int) {}
    virtual void increment() {}
};

}  // namespace hdl_graph_slam
