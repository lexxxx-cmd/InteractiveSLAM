#pragma once

#include <QWidget>
#include <QTimer>
#include <memory>
#include <set>
#include <vector>
#include <osg/Group>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include "visualizers/GraphSceneVisualizer.h"
#include "ui/DrawFlags.h"

class osgQOpenGLWidget;
class GraphManager;
class OverlayPanelWidget;
class SpherePickingHandler;

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
    void refreshScene();
    void rebuildPointClouds();  // heavy — call only after optimization

    // Rendering controls
    void setDrawVertices(bool v);
    void setDrawEdges(bool v);
    void setDrawKeyframeClouds(bool v);
    void setDrawSE3Edges(bool v);
    void setEdgeWidth(int width);
    void setSphereRadius(float radius);
    void setPointSize(int size);
    void setPointOpacity(int opacity);
    void setBackgroundColor(const QColor& color);
    void setHiddenEdges(const std::set<long>& ids);
    void setLoopHighlight(long sourceId, const std::vector<long>& candidateIds);
    void resetCamera();

    // Z-clip + elevation color range
    void setZClipping(bool enabled);
    void setZClipMin(double minZ);
    void setZClipMax(double maxZ);
    void setColorZMin(double minZ);
    void setColorZMax(double maxZ);
    void setAutoColorRange(bool autoRange);

    /// Replace the default perspective projection with orthographic.
    void applyOrthographicProjection();
    /// Restore perspective projection.
    void applyPerspectiveProjection();
    /// Apply current projection based on m_useOrthographic flag.
    void applyProjection();

    /// Toggle between perspective and orthographic projection.
    void setUseOrthographic(bool enabled);
    bool isOrthographic() const { return m_useOrthographic; }

    // Overlay panel management (floating panels over viewport)
    void registerOverlay(OverlayPanelWidget* overlay);
    void updateOverlayPositions();

signals:
    void fpsUpdated(float fps);
    void initialized();
    void cloudDataReady(float dataZMin, float dataZMax);
    void vertexSelected(long vertexId);
    void contextMenuRequested(long vertexId, long edgeId,
                              long edgeV1, long edgeV2,
                              double edgeDist, const QString& edgeKernel,
                              QPoint screenPos,
                              long vtxCloudSize,
                              double vtxPosX, double vtxPosY, double vtxPosZ,
                              double vtxAccumDist, int vtxDegree);

protected:
    void resizeEvent(QResizeEvent* event) override;

private slots:
    void initOsg();        // called on osgQOpenGLWidget::initialized
    void updateScene();    // timer-driven pose/edge refresh
    void onVertexPicked(long vertexId);  // picking callback

private:
    osgQOpenGLWidget* m_osgWidget = nullptr;
    std::unique_ptr<GraphSceneVisualizer> m_sceneViz;
    std::shared_ptr<hdl_graph_slam::InteractiveGraph> m_graph;
    hdl_graph_slam::DrawFlags m_flags;

    QTimer* m_updateTimer = nullptr;
    osg::ref_ptr<SpherePickingHandler> m_pickingHandler;

    // Overlay panels (floating over viewport)
    QVector<OverlayPanelWidget*> m_overlays;
    int m_overlayMargin = 10;

    // Orthographic projection tracking
    double m_orthoHalfHeight = 10.0;
    bool m_useOrthographic = false;   // default: perspective

    // FPS tracking (rolling average)
    int m_frameCount = 0;
    float m_fpsAccum = 0.0f;
    float m_fps = 0.0f;
};
