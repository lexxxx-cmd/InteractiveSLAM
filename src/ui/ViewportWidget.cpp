#include "ui/ViewportWidget.h"

#include <QVBoxLayout>
#include <QColor>
#include <QResizeEvent>
#include <chrono>

#include <osg/Notify>

#include "osgQOpenGL/osgQOpenGLWidget.h"
#include "osgQOpenGL/OSGRenderer.h"
#include "backend/graph_manager.hpp"
#include "visualizers/SpherePickingHandler.h"
#include "ui/OverlayPanelWidget.h"

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

ViewportWidget::ViewportWidget(QWidget* parent)
    : QWidget(parent) {

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    m_osgWidget = new osgQOpenGLWidget(this);
    layout->addWidget(m_osgWidget);

    m_sceneViz = std::make_unique<GraphSceneVisualizer>();

    connect(m_osgWidget, &osgQOpenGLWidget::initialized,
            this, &ViewportWidget::initOsg);

    // Pose update timer (~60 Hz)
    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, &ViewportWidget::updateScene);
    m_updateTimer->start(16);
}

ViewportWidget::~ViewportWidget() = default;

// ---------------------------------------------------------------------------
// OSG Initialization (called once after the OpenGL context is ready)
// ---------------------------------------------------------------------------

void ViewportWidget::initOsg() {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;

    osg::State* state = viewer->getCamera()->getGraphicsContext()->getState();
    if (state) {
        state->setUseModelViewAndProjectionUniforms(true);
        state->setUseVertexAttributeAliasing(true);
    }

    // Dark background (matching interactive_slam)
    viewer->getCamera()->setClearColor(osg::Vec4(0.1f, 0.1f, 0.12f, 1.0f));

    // Set up scene root
    viewer->setSceneData(m_sceneViz->getRootNode());

    // Trackball camera
    viewer->setCameraManipulator(new osgGA::TrackballManipulator);

    // Register picking handler with lazy providers — data is queried
    // on each event, so graph load / pose updates are always reflected.
    m_pickingHandler = new SpherePickingHandler(
        // sphere centers provider
        [this]() -> const std::vector<std::pair<osg::Vec3d, long>>* {
            return &m_sceneViz->sphereCenters();
        },
        // edge segments provider
        [this]() -> const std::vector<EdgeSegment>* {
            return &m_sceneViz->edgeSegments();
        },
        m_sceneViz->sphereRadius(),
        // --- selection callback (Ctrl+Click) ---
        [this](long vertexId) {
            QMetaObject::invokeMethod(this, [this, vertexId]() {
                onVertexPicked(vertexId);
            }, Qt::QueuedConnection);
        },
        // --- context menu callback (RightClick) ---
        [this](const PickingHit& hit) {
            QMetaObject::invokeMethod(this, [this, hit]() {
                // Enrich vertex fields from graph data
                PickingHit enriched = hit;
                if (hit.vertexId >= 0 && m_graph) {
                    auto it = m_graph->keyframes.find(hit.vertexId);
                    if (it != m_graph->keyframes.end()) {
                        auto& kf = it->second;
                        auto pos = kf->estimate().translation();
                        enriched.vtxPosX = pos.x();
                        enriched.vtxPosY = pos.y();
                        enriched.vtxPosZ = pos.z();
                        enriched.vtxCloudSize = kf->cloud ? static_cast<long>(kf->cloud->size()) : 0;
                        enriched.vtxAccumDist = kf->accum_distance;
                        enriched.vtxDegree = static_cast<int>(kf->node->edges().size());
                    }
                }
                QPoint globalPos = m_osgWidget->mapToGlobal(
                    QPoint(static_cast<int>(hit.screenX),
                           static_cast<int>(hit.screenY)));
                emit contextMenuRequested(
                    enriched.vertexId, enriched.edgeId,
                    enriched.edgeV1, enriched.edgeV2,
                    enriched.edgeDist,
                    QString::fromStdString(enriched.edgeKernel),
                    globalPos,
                    enriched.vtxCloudSize,
                    enriched.vtxPosX, enriched.vtxPosY, enriched.vtxPosZ,
                    enriched.vtxAccumDist, enriched.vtxDegree);
            }, Qt::QueuedConnection);
        });
    viewer->addEventHandler(m_pickingHandler);

    emit initialized();
}

// ---------------------------------------------------------------------------
// Graph loading
// ---------------------------------------------------------------------------

void ViewportWidget::onGraphLoaded(std::shared_ptr<hdl_graph_slam::InteractiveGraph> graph) {
    m_graph = graph;
    m_sceneViz->buildFromGraph(graph, m_flags);

    // Apply current settings
    m_sceneViz->setPointOpacity(m_flags.draw_keyframe_vertices ? 1.0f : 0.0f);
    m_osgWidget->update();

    // Home camera to see the whole scene
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (viewer) {
        viewer->home();
    }
}

void ViewportWidget::onGraphClosed() {
    m_graph.reset();
    m_sceneViz->clear();
    m_osgWidget->update();
}

// ---------------------------------------------------------------------------
// Rendering controls
// ---------------------------------------------------------------------------

void ViewportWidget::setDrawVertices(bool v) {
    m_flags.draw_verticies = v;
    m_flags.draw_keyframe_vertices = v;
    m_sceneViz->setDrawVertices(v);
    m_osgWidget->update();
}

void ViewportWidget::setDrawEdges(bool v) {
    m_flags.draw_edges = v;
    m_sceneViz->setDrawEdges(v);
    m_osgWidget->update();
}

void ViewportWidget::setDrawKeyframeClouds(bool v) {
    m_flags.draw_keyframe_vertices = v;
    m_sceneViz->setDrawKeyframeClouds(v);
    m_osgWidget->update();
}

void ViewportWidget::setDrawSE3Edges(bool v) {
    m_flags.draw_se3_edges = v;
    // SE3 edges are part of the regular edge set
    m_sceneViz->setDrawEdges(v);
    m_osgWidget->update();
}

void ViewportWidget::setEdgeWidth(int width) {
    m_sceneViz->setEdgeWidth(static_cast<float>(width));
    m_osgWidget->update();
}

void ViewportWidget::setSphereRadius(float radius) {
    m_sceneViz->setSphereRadius(radius);
    m_osgWidget->update();
}

void ViewportWidget::setPointSize(int size) {
    m_sceneViz->setPointSize(static_cast<float>(size));
    m_osgWidget->update();
}

void ViewportWidget::setPointOpacity(int opacity) {
    m_sceneViz->setPointOpacity(opacity / 100.0f);
    m_osgWidget->update();
}

void ViewportWidget::setBackgroundColor(const QColor& color) {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (viewer) {
        viewer->getCamera()->setClearColor(
            osg::Vec4(color.redF(), color.greenF(), color.blueF(), 1.0f));
    }
    m_osgWidget->update();
}

void ViewportWidget::setHiddenEdges(const std::set<long>& ids) {
    m_sceneViz->setHiddenEdges(ids);
}

void ViewportWidget::setLoopHighlight(long sourceId, const std::vector<long>& candidateIds) {
    m_sceneViz->setLoopHighlight(sourceId, candidateIds);
}

void ViewportWidget::resetCamera() {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (viewer) {
        viewer->home();
    }
    m_osgWidget->update();
}

// ---------------------------------------------------------------------------
// Manual scene refresh (called after data changes like edge deletion)
// ---------------------------------------------------------------------------

void ViewportWidget::refreshScene() {
    if (!m_graph) return;
    m_sceneViz->updatePoses(m_graph);
    m_osgWidget->update();
}

void ViewportWidget::rebuildPointClouds() {
    if (!m_graph) return;
    m_sceneViz->rebuildPointClouds(m_graph);
    m_osgWidget->update();
}

// ---------------------------------------------------------------------------
// Frame update (timer-driven)
// ---------------------------------------------------------------------------

void ViewportWidget::updateScene() {
    if (!m_graph) return;

    // Update keyframe transforms and edge geometry from g2o estimates
    m_sceneViz->updatePoses(m_graph);
    m_osgWidget->update();

    // FPS tracking (rolling 1-second average)
    m_frameCount++;
    static auto lastTime = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - lastTime).count();
    if (elapsed >= 1.0f) {
        m_fps = static_cast<float>(m_frameCount) / elapsed;
        m_frameCount = 0;
        lastTime = now;
        emit fpsUpdated(m_fps);
    }
}

// ---------------------------------------------------------------------------
// Picking
// ---------------------------------------------------------------------------

void ViewportWidget::onVertexPicked(long vertexId) {
    m_sceneViz->setSelectedVertex(vertexId);

    // Rebuild spheres to apply/dismiss highlight colour
    if (m_graph) {
        m_sceneViz->updatePoses(m_graph);
        m_osgWidget->update();
    }

    emit vertexSelected(vertexId);
}

// ---------------------------------------------------------------------------
// Overlay panel management
// ---------------------------------------------------------------------------

void ViewportWidget::registerOverlay(OverlayPanelWidget* overlay) {
    if (!overlay) return;

    // Reparent to this viewport
    overlay->setParent(this);

    // Add to tracking list
    m_overlays.append(overlay);

    // Set initial size hint
    overlay->adjustSize();

    // Position and show
    updateOverlayPositions();
    overlay->raise();
    overlay->show();
}

void ViewportWidget::updateOverlayPositions() {
    int yOffset = m_overlayMargin;

    for (auto* overlay : m_overlays) {
        if (!overlay->isVisible()) continue;

        int panelW = overlay->width();
        int panelH = overlay->height();

        // Anchor to top-right corner
        int newX = width() - panelW - m_overlayMargin;
        int newY = yOffset;

        // Clamp within viewport bounds
        newX = std::max(m_overlayMargin,
                        std::min(newX, width() - panelW - m_overlayMargin));
        newY = std::max(m_overlayMargin,
                        std::min(newY, height() - panelH - m_overlayMargin));

        overlay->move(newX, newY);

        // Next overlay stacks below
        yOffset = newY + panelH + 6;
    }
}

void ViewportWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    updateOverlayPositions();
}
