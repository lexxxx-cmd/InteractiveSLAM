#pragma once

#include <QQuickFramebufferObject>
#include <QMutex>
#include <QPoint>
#include <memory>
#include <vector>

#include "backend/graph_manager.hpp"
#include "viewport/drawable_object.hpp"

class QMouseEvent;
class QWheelEvent;
class QKeyEvent;

// OSGViewport — QML entry point for OSG rendering.
//
// It is a QQuickFramebufferObject so it can live inside a QtQuick scene. Its
// companion OSGViewportRenderer::render() calls OSGRenderer::frame() to draw
// the OSG scene graph into the Qt-managed framebuffer object.
//
// Input events arrive on the GUI thread (mouse/wheel/key overrides below). They
// are cloned into a thread-safe queue and drained by the renderer during
// synchronize() on the render thread, then forwarded into OSG's event queue.
//
// This is the OSG-powered replacement for the glk-based GraphViewport. For now
// it renders an axes scene to prove the OSG ↔ QML pipeline; point cloud /
// keyframe / edge migration is a follow-up.
class OSGViewport : public QQuickFramebufferObject
{
    Q_OBJECT
    Q_PROPERTY(float currentFps READ currentFps NOTIFY fpsUpdated)
    Q_PROPERTY(GraphManager* graphManager READ graphManager
               WRITE setGraphManager NOTIFY graphManagerChanged)
    Q_PROPERTY(bool drawVertices READ drawVertices WRITE setDrawVertices NOTIFY drawFlagsChanged)
    Q_PROPERTY(bool drawEdges READ drawEdges WRITE setDrawEdges NOTIFY drawFlagsChanged)
    Q_PROPERTY(bool drawKeyframeVertices READ drawKeyframeVertices WRITE setDrawKeyframeVertices NOTIFY drawFlagsChanged)
    Q_PROPERTY(bool drawSE3Edges READ drawSE3Edges WRITE setDrawSE3Edges NOTIFY drawFlagsChanged)

public:
    explicit OSGViewport(QQuickItem* parent = nullptr);

    float currentFps() const { return m_currentFps; }

    // GraphManager access (mirrors GraphViewport)
    GraphManager* graphManager() const { return m_graphManager; }
    void setGraphManager(GraphManager* manager);

    // DrawFlags accessors
    bool drawVertices() const { return m_drawFlags.draw_verticies; }
    void setDrawVertices(bool v);
    bool drawEdges() const { return m_drawFlags.draw_edges; }
    void setDrawEdges(bool v);
    bool drawKeyframeVertices() const { return m_drawFlags.draw_keyframe_vertices; }
    void setDrawKeyframeVertices(bool v);
    bool drawSE3Edges() const { return m_drawFlags.draw_se3_edges; }
    void setDrawSE3Edges(bool v);

    // QML-callable: reset the OSG camera manipulator to its home position.
    Q_INVOKABLE void resetCamera();

    QQuickFramebufferObject::Renderer* createRenderer() const override;

signals:
    void fpsUpdated(float fps);
    void graphManagerChanged();
    void drawFlagsChanged();

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

private:
    friend class OSGViewportRenderer;
    void receiveFpsUpdate(float fps);

    // Thread-safe input queue. GUI thread enqueues clones; render thread drains
    // them in synchronize(). Using QEvent clones keeps position/button/delta data.
    struct CloneEvent {
        enum Type { MousePress, MouseRelease, MouseMove, MouseDoubleClick, Wheel, KeyPress, KeyRelease } type;
        QPointF pos;            // mouse / wheel position
        Qt::MouseButtons buttons;
        Qt::MouseButton  button;
        QPoint angleDelta;      // wheel
        int key;                // key event
        Qt::KeyboardModifiers modifiers;
    };
    void enqueue(const CloneEvent& e);

    mutable QMutex m_queueMutex;
    std::vector<CloneEvent> m_eventQueue;

    // Camera-reset request from QML (main thread → render thread via synchronize).
    bool   m_cameraResetRequested = false;

    GraphManager* m_graphManager = nullptr;
    hdl_graph_slam::DrawFlags m_drawFlags;

    float m_currentFps = 0.0f;
};

