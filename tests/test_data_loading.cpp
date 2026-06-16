#include "data/hdl_graph_slam/interactive_graph.hpp"
#include <iostream>
#include <cassert>

struct ConsoleProgress : guik::ProgressInterface {
    void set_title(const std::string& title) override {
        std::cout << "=== " << title << " ===" << std::endl;
    }
    void set_text(const std::string& text) override {
        std::cout << "  " << text << "..." << std::endl;
    }
    void increment() override { std::cout << "." << std::flush; }
};

int main(int argc, char** argv) {
    if (argc < 2) {
        std::cerr << "Usage: test_data_loading <map_folder>" << std::endl;
        return 1;
    }

    hdl_graph_slam::InteractiveGraph graph;
    ConsoleProgress progress;

    bool ok = graph.load_map_data(argv[1], progress);
    assert(ok && "Failed to load map data");

    std::cout << "\n--- Load Results ---" << std::endl;
    std::cout << "Vertices: " << graph.num_vertices() << std::endl;
    std::cout << "Edges:    " << graph.num_edges() << std::endl;
    std::cout << "Keyframes: " << graph.keyframes.size() << std::endl;

    if (graph.anchor_node_id() >= 0) {
        std::cout << "Anchor node ID: " << graph.anchor_node_id() << std::endl;
    }

    for (auto& [id, kf] : graph.keyframes) {
        std::cout << "  KF " << id
                  << " | pts: " << kf->cloud->size()
                  << " | pos: " << kf->estimate().translation().transpose()
                  << std::endl;
    }

    std::cout << "\n--- All tests passed ---" << std::endl;
    return 0;
}
