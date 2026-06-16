#pragma once

#include <QQuickFramebufferObject>
#include "backend/graph_manager.hpp"

class GraphViewportRenderer;

class GraphViewport : public QQuickFramebufferObject {
    Q_OBJECT
    Q_PROPERTY(GraphManager* graphManager READ graphManager
               WRITE setGraphManager NOTIFY graphManagerChanged)

public:
    explicit GraphViewport(QQuickItem* parent = nullptr);

    GraphManager* graphManager() const;
    void setGraphManager(GraphManager* manager);

    // FPS data (passed from renderer via synchronize())
    float currentFps() const { return m_currentFps; }

    Renderer* createRenderer() const override;

signals:
    void graphManagerChanged();
    void fpsUpdated(float fps);

private:
    friend class GraphViewportRenderer;
    void receiveFpsUpdate(float fps) {
        m_currentFps = fps;
        emit fpsUpdated(fps);
    }

    GraphManager* m_graphManager = nullptr;
    float m_currentFps = 0.0f;
};
