#include "ui/MainWindow.h"
#include "ui/ViewportWidget.h"
#include "ui/GraphInfoPanel.h"
#include "backend/graph_manager.hpp"

#include <QFileDialog>
#include <QMessageBox>
#include <QMenu>
#include <QApplication>

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

MainWindow::MainWindow(GraphManager* manager, QWidget* parent)
    : QMainWindow(parent), m_manager(manager) {

    setWindowTitle("Interactive SLAM");
    resize(1280, 720);

    setupUi();
    setupMenus();

    // --- GraphManager signals ---
    connect(m_manager, &GraphManager::loadingStarted,
            this, &MainWindow::onLoadingStarted);
    connect(m_manager, &GraphManager::loadingSucceeded,
            this, &MainWindow::onLoadingSucceeded);
    connect(m_manager, &GraphManager::loadingFailed,
            this, &MainWindow::onLoadingFailed);
    connect(m_manager, &GraphManager::lastMessageChanged,
            this, &MainWindow::onLogMessage);

    // Picking feedback in status bar
    connect(m_viewport, &ViewportWidget::vertexSelected,
            this, [this](long vertexId) {
        if (vertexId >= 0)
            statusBar()->showMessage(tr("Selected vertex: %1").arg(vertexId));
        else
            statusBar()->clearMessage();
    });

    // Right-click context menu
    connect(m_viewport, &ViewportWidget::contextMenuRequested,
            this, [this](long vertexId, long edgeId,
                         long edgeV1, long edgeV2,
                         double edgeDist, const QString& edgeKernel,
                         QPoint pos,
                         long vtxCloudSize,
                         double vtxPosX, double vtxPosY, double vtxPosZ,
                         double vtxAccumDist, int vtxDegree) {
        if (vertexId < 0 && edgeId < 0) return;
        QMenu menu;
        if (vertexId >= 0) {
            menu.addAction(tr("Vertex ID: %1").arg(vertexId))->setEnabled(false);
            menu.addAction(tr("Position: (%1, %2, %3)")
                .arg(vtxPosX, 0, 'f', 2)
                .arg(vtxPosY, 0, 'f', 2)
                .arg(vtxPosZ, 0, 'f', 2))->setEnabled(false);
            menu.addAction(tr("Distance: %1 m").arg(vtxAccumDist, 0, 'f', 2))
                ->setEnabled(false);
            menu.addAction(tr("Degree: %1 edges").arg(vtxDegree))->setEnabled(false);
            menu.addAction(tr("Cloud: %1 points").arg(vtxCloudSize))->setEnabled(false);
            menu.addSeparator();

            // --- Loop Begin ---
            QAction* loopBeginAction = menu.addAction(tr("Loop Begin"));
            connect(loopBeginAction, &QAction::triggered, this, [this, vertexId]() {
                m_loopBeginVertexId = vertexId;
                statusBar()->showMessage(
                    tr("Loop begin set to vertex %1. Right-click another vertex → Loop End.")
                        .arg(vertexId), 5000);
            });

            // --- Loop End ---
            QAction* loopEndAction = menu.addAction(tr("Loop End"));
            if (m_loopBeginVertexId < 0) {
                loopEndAction->setEnabled(false);       // 未设 begin 时灰色
            }
            connect(loopEndAction, &QAction::triggered, this, [this, vertexId]() {
                if (m_loopBeginVertexId < 0) return;

                auto* graph = m_manager->graph();
                if (!graph) {
                    statusBar()->showMessage(tr("No graph loaded"), 3000);
                    m_loopBeginVertexId = -1;
                    return;
                }

                // 拒绝相同顶点
                if (m_loopBeginVertexId == vertexId) {
                    statusBar()->showMessage(tr("Cannot loop to the same vertex"), 3000);
                    m_loopBeginVertexId = -1;
                    return;
                }

                // 查找两端 KeyFrame
                auto itBegin = graph->keyframes.find(m_loopBeginVertexId);
                auto itEnd   = graph->keyframes.find(vertexId);
                if (itBegin == graph->keyframes.end() || itEnd == graph->keyframes.end()) {
                    statusBar()->showMessage(tr("Vertex not found"), 3000);
                    m_loopBeginVertexId = -1;
                    return;
                }

                auto& beginKf = itBegin->second;
                auto& endKf   = itEnd->second;

                // 检查是否已连接（使用 g2o 基类 API，无需 include SE3 头文件）
                bool alreadyConnected = false;
                if (beginKf->node && endKf->node) {
                    for (auto* edge : beginKf->node->edges()) {
                        const auto& verts = edge->vertices();
                        bool hasBegin = false, hasEnd = false;
                        for (size_t i = 0; i < verts.size(); ++i) {
                            if (verts[i] == beginKf->node) hasBegin = true;
                            if (verts[i] == endKf->node)   hasEnd   = true;
                        }
                        if (hasBegin && hasEnd) { alreadyConnected = true; break; }
                    }
                }
                if (alreadyConnected) {
                    statusBar()->showMessage(
                        tr("Vertices %1 and %2 are already connected")
                            .arg(m_loopBeginVertexId).arg(vertexId), 3000);
                    m_loopBeginVertexId = -1;
                    return;
                }

                // 以单位矩阵为相对位姿添加边（不做配准）
                graph->add_edge(beginKf, endKf, Eigen::Isometry3d::Identity());

                // 全局优化 + 刷新场景
                graph->optimize();
                m_viewport->refreshScene();

                statusBar()->showMessage(
                    tr("Loop edge added: %1 → %2").arg(m_loopBeginVertexId).arg(vertexId), 5000);

                m_loopBeginVertexId = -1;
            });

            menu.addSeparator();
            menu.addAction(tr("Go to Vertex"))->setEnabled(false);
            menu.addAction(tr("Vertex Details..."))->setEnabled(false);
        } else if (edgeId >= 0) {
            menu.addAction(tr("Edge ID: %1").arg(edgeId))->setEnabled(false);
            menu.addAction(tr("Vertices: %1 → %2").arg(edgeV1).arg(edgeV2))
                ->setEnabled(false);
            menu.addAction(tr("Length: %1 m").arg(edgeDist, 0, 'f', 2))
                ->setEnabled(false);
            menu.addAction(tr("Kernel: %1").arg(edgeKernel))->setEnabled(false);
            menu.addSeparator();
            menu.addAction(tr("Go to Edge"))->setEnabled(false);
            menu.addAction(tr("Edge Details..."))->setEnabled(false);
            menu.addSeparator();
            QAction* deleteAction = menu.addAction(tr("Delete Edge"));
            connect(deleteAction, &QAction::triggered, this, [this, edgeId]() {
                auto* graph = m_manager->graph();
                if (!graph) return;
                if (graph->removeEdge(edgeId)) {
                    m_viewport->refreshScene();
                    statusBar()->showMessage(
                        tr("Edge %1 deleted").arg(edgeId), 3000);
                } else {
                    statusBar()->showMessage(
                        tr("Failed to delete edge %1").arg(edgeId), 3000);
                }
            });
        }
        menu.exec(pos);
    });
}

// ---------------------------------------------------------------------------
// UI Setup
// ---------------------------------------------------------------------------

void MainWindow::setupUi() {
    // Central viewport (3D rendering area)
    m_viewport = new ViewportWidget(this);
    setCentralWidget(m_viewport);

    // Left dock panel (statistics + rendering controls)
    auto* dock = new QDockWidget(tr("Controls"), this);
    dock->setFeatures(QDockWidget::NoDockWidgetFeatures);
    dock->setAllowedAreas(Qt::LeftDockWidgetArea);

    m_infoPanel = new GraphInfoPanel(m_manager, m_viewport, dock);
    dock->setWidget(m_infoPanel);
    addDockWidget(Qt::LeftDockWidgetArea, dock);

    // Status bar
    statusBar()->showMessage(tr("Ready — open a map folder to begin"));
}

void MainWindow::setupMenus() {
    // ---- File menu ----
    auto* fileMenu = menuBar()->addMenu(tr("&File"));

    auto* openAction = fileMenu->addAction(tr("&Open Map..."));
    openAction->setShortcut(QKeySequence::Open);
    connect(openAction, &QAction::triggered, this, &MainWindow::onOpenMap);

    auto* closeAction = fileMenu->addAction(tr("&Close Map"));
    closeAction->setShortcut(QKeySequence::Close);
    connect(closeAction, &QAction::triggered, this, &MainWindow::onCloseMap);

    fileMenu->addSeparator();

    auto* quitAction = fileMenu->addAction(tr("&Quit"));
    quitAction->setShortcut(QKeySequence::Quit);
    connect(quitAction, &QAction::triggered, qApp, &QApplication::quit);

    // ---- View menu ----
    auto* viewMenu = menuBar()->addMenu(tr("&View"));

    auto* resetCamAction = viewMenu->addAction(tr("&Reset Camera"));
    resetCamAction->setShortcut(QKeySequence(Qt::Key_R));
    connect(resetCamAction, &QAction::triggered, this, &MainWindow::onResetCamera);

    // ---- Graph menu ----
    auto* graphMenu = menuBar()->addMenu(tr("&Graph"));

    auto* optimizeAction = graphMenu->addAction(tr("&Optimize"));
    optimizeAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_O));
    connect(optimizeAction, &QAction::triggered, this, &MainWindow::onOptimize);
}

// ---------------------------------------------------------------------------
// Slots — File operations
// ---------------------------------------------------------------------------

void MainWindow::onOpenMap() {
    QString dir = QFileDialog::getExistingDirectory(
        this, tr("Open Map Directory"), QString(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    if (dir.isEmpty()) return;

    m_manager->openMapData(QUrl::fromLocalFile(dir));
}

void MainWindow::onCloseMap() {
    m_manager->closeMap();
    m_viewport->onGraphClosed();
    m_loopBeginVertexId = -1;
    statusBar()->showMessage(tr("Map closed"));
}

// ---------------------------------------------------------------------------
// Slots — Graph operations
// ---------------------------------------------------------------------------

void MainWindow::onOptimize() {
    if (!m_manager->isLoaded()) {
        statusBar()->showMessage(tr("No graph loaded"));
        return;
    }

    statusBar()->showMessage(tr("Optimizing..."));
    // Trigger optimization via the data layer
    auto* graph = m_manager->graph();
    if (graph) {
        graph->optimize();
    }
    statusBar()->showMessage(tr("Optimization complete"), 3000);
}

void MainWindow::onResetCamera() {
    m_viewport->resetCamera();
}

// ---------------------------------------------------------------------------
// Slots — Loading state
// ---------------------------------------------------------------------------

void MainWindow::onLoadingStarted() {
    statusBar()->showMessage(tr("Loading map..."));
}

void MainWindow::onLoadingSucceeded() {
    m_loopBeginVertexId = -1;
    statusBar()->showMessage(
        tr("Map loaded — %1 vertices, %2 edges, %3 keyframes")
            .arg(m_manager->vertexCount())
            .arg(m_manager->edgeCount())
            .arg(m_manager->keyframeCount()),
        5000);

    m_viewport->onGraphLoaded(m_manager->sharedGraph());
}

void MainWindow::onLoadingFailed(const QString& error) {
    statusBar()->showMessage(tr("Loading failed: %1").arg(error));
    QMessageBox::warning(this, tr("Load Error"), error);
}

void MainWindow::onLogMessage(const QString& message) {
    statusBar()->showMessage(message, 5000);
}
