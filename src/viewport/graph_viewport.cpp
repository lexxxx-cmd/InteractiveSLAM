#include "viewport/graph_viewport.hpp"
#include "viewport/graph_viewport_renderer.hpp"

GraphViewport::GraphViewport(QQuickItem* parent)
    : QQuickFramebufferObject(parent) {}

GraphManager* GraphViewport::graphManager() const { return m_graphManager; }
void GraphViewport::setGraphManager(GraphManager* manager) {
    if (m_graphManager != manager) {
        m_graphManager = manager;
        emit graphManagerChanged();
    }
}

QQuickFramebufferObject::Renderer* GraphViewport::createRenderer() const {
    return new GraphViewportRenderer();
}
