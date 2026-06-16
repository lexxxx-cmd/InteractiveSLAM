#pragma once

#include <QQuickFramebufferObject>
#include <Eigen/Core>
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

    float currentFps() const { return m_currentFps; }

    // Camera control (called from QML, main thread)
    Q_INVOKABLE void onMouseRotate(float dx, float dy);
    Q_INVOKABLE void onMousePan(float dx, float dy);
    Q_INVOKABLE void onMouseZoom(float delta);
    Q_INVOKABLE void resetCamera();
    Q_INVOKABLE void fitView(const Eigen::Vector3f& bboxMin,
                              const Eigen::Vector3f& bboxMax);

    // Camera state for renderer sync
    Eigen::Matrix4f cameraViewMatrix() const;
    bool cameraDirty() const { return m_cameraDirty; }
    void clearCameraDirty() { m_cameraDirty = false; }

    Renderer* createRenderer() const override;

signals:
    void graphManagerChanged();
    void fpsUpdated(float fps);

private:
    friend class GraphViewportRenderer;
    void receiveFpsUpdate(float fps);

    GraphManager* m_graphManager = nullptr;
    float m_currentFps = 0.0f;

    // Camera state (main thread)
    Eigen::Vector3f m_camCenter{0, 0, 0};
    double m_camDistance = 10.0;
    double m_camTheta = 0.0;
    double m_camPhi = -1.0472;  // -60°
    bool m_cameraDirty = true;
};
