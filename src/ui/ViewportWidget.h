#pragma once

#include <QWidget>
#include <QTimer>
#include <memory>
#include <osg/Group>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include "visualizers/GraphSceneVisualizer.h"
#include "ui/DrawFlags.h"

class osgQOpenGLWidget;
class GraphManager;

namespace hdl_graph_slam {
class InteractiveGraph;
}

/// @brief Central 3D viewport widget hosting the osgQOpenGLWidget.
///        Modeled after 3DPCViewer's VisualAreaWidget.
class ViewportWidget : public QWidget {
    Q_OBJECT

public:
    explicit ViewportWidget(QWidget* parent = nullptr);
    ~ViewportWidget() override;

public slots:
    void onGraphLoaded(std::shared_ptr<hdl_graph_slam::InteractiveGraph> graph);
    void onGraphClosed();

    // Rendering controls
    void setDrawVertices(bool v);
    void setDrawEdges(bool v);
    void setDrawKeyframeClouds(bool v);
    void setDrawSE3Edges(bool v);
    void setSphereRadius(float radius);
    void setPointSize(int size);
    void setPointOpacity(int opacity);
    void setBackgroundColor(const QColor& color);
    void resetCamera();

signals:
    void fpsUpdated(float fps);
    void initialized();

private slots:
    void initOsg();        // called on osgQOpenGLWidget::initialized
    void updateScene();    // timer-driven pose/edge refresh

private:
    osgQOpenGLWidget* m_osgWidget = nullptr;
    std::unique_ptr<GraphSceneVisualizer> m_sceneViz;
    std::shared_ptr<hdl_graph_slam::InteractiveGraph> m_graph;
    hdl_graph_slam::DrawFlags m_flags;

    QTimer* m_updateTimer = nullptr;

    // FPS tracking (rolling average)
    int m_frameCount = 0;
    float m_fpsAccum = 0.0f;
    float m_fps = 0.0f;
};
