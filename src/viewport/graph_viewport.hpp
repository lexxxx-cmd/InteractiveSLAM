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

    // Pick API (async: request now, result via pickResultReady signal)
    Q_INVOKABLE void requestPick(float mouseX, float mouseY);

    Q_PROPERTY(int selectedVertexId READ selectedVertexId WRITE setSelectedVertexId NOTIFY selectedVertexIdChanged)
    Q_PROPERTY(int pickedVertexId READ pickedVertexId NOTIFY pickResultReady)
    Q_PROPERTY(float pickedWorldX READ pickedWorldX NOTIFY pickResultReady)
    Q_PROPERTY(float pickedWorldY READ pickedWorldY NOTIFY pickResultReady)
    Q_PROPERTY(float pickedWorldZ READ pickedWorldZ NOTIFY pickResultReady)

    int   selectedVertexId() const       { return m_selectedVertexId; }
    void  setSelectedVertexId(int id);

    int   pickedVertexId() const { return m_pickedVertexId; }
    float pickedWorldX()   const { return m_pickedWorldPos.x(); }
    float pickedWorldY()   const { return m_pickedWorldPos.y(); }
    float pickedWorldZ()   const { return m_pickedWorldPos.z(); }

    // Camera state for renderer sync
    Eigen::Matrix4f cameraViewMatrix() const;
    bool cameraDirty() const { return m_cameraDirty; }
    void clearCameraDirty() { m_cameraDirty = false; }

    Renderer* createRenderer() const override;

signals:
    void graphManagerChanged();
    void fpsUpdated(float fps);
    void pickResultReady();
    void selectedVertexIdChanged();

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

    // Pick state (main thread ↔ render thread via synchronize)
    bool   m_pickPending = false;
    float  m_pickMouseX = 0, m_pickMouseY = 0;
    int    m_pickedVertexId = -1;
    int    m_selectedVertexId = -1;
    Eigen::Vector3f m_pickedWorldPos{0, 0, 0};
};
