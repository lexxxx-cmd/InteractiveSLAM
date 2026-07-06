#include "ui/MainWindow.h"
#include "ui/ViewportWidget.h"
#include "ui/GraphStatsPanel.h"
#include "ui/RenderingPanel.h"
#include "ui/ZClippingPanel.h"
#include "ui/ColorRangePanel.h"
#include "ui/AutoLoopClosurePanel.h"
#include "ui/EdgeListPanel.h"
#include "ui/OverlayPanelWidget.h"
#include "ui/LoopClosureDialog.h"
#include "backend/graph_manager.hpp"

#include <QFileDialog>
#include <QMessageBox>
#include <QMenu>
#include <QDialog>
#include <QFormLayout>
#include <QSpinBox>
#include <QLabel>
#include <QDialogButtonBox>
#include <QApplication>
#include <vector>

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
                loopEndAction->setEnabled(false);
            }
            connect(loopEndAction, &QAction::triggered, this, [this, vertexId]() {
                if (m_loopBeginVertexId < 0) return;

                auto* graph = m_manager->graph();
                if (!graph) {
                    statusBar()->showMessage(tr("No graph loaded"), 3000);
                    m_loopBeginVertexId = -1;
                    return;
                }

                if (m_loopBeginVertexId == vertexId) {
                    statusBar()->showMessage(tr("Cannot loop to the same vertex"), 3000);
                    m_loopBeginVertexId = -1;
                    return;
                }

                auto mergedBegin = mergeAdjacentClouds(graph, m_loopBeginVertexId, m_submapWindowHalfSize);
                auto mergedEnd   = mergeAdjacentClouds(graph, vertexId, m_submapWindowHalfSize);
                if (!mergedBegin || !mergedEnd ||
                    mergedBegin->empty() || mergedEnd->empty()) {
                    statusBar()->showMessage(
                        tr("Vertex has no point cloud data"), 3000);
                    m_loopBeginVertexId = -1;
                    return;
                }

                long beginId = m_loopBeginVertexId;
                m_loopBeginVertexId = -1;

                LoopClosureDialog dlg(beginId, vertexId, m_manager,
                                      mergedBegin, mergedEnd, this);
                if (dlg.exec() == QDialog::Accepted) {
                    m_viewport->refreshScene();
                    m_viewport->rebuildPointClouds();
                    statusBar()->showMessage(
                        tr("Loop edge added: %1 → %2").arg(beginId).arg(vertexId), 5000);
                } else {
                    statusBar()->showMessage(tr("Loop closure cancelled"), 3000);
                }
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
                        tr("Cannot delete edge %1 — optimization running, retry shortly")
                            .arg(edgeId), 3000);
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

    // ── New floating overlay panels (default hidden) ──────────────────
    m_statsPanel  = new GraphStatsPanel(m_manager, m_viewport, nullptr);
    m_renderPanel = new RenderingPanel(m_viewport, nullptr);
    m_zClipPanel  = new ZClippingPanel(m_viewport, nullptr);
    m_colorPanel  = new ColorRangePanel(m_viewport, nullptr);

    m_statsOverlay  = new OverlayPanelWidget(tr("Graph Statistics"), m_statsPanel);
    m_renderOverlay = new OverlayPanelWidget(tr("Rendering"), m_renderPanel);
    m_zClipOverlay  = new OverlayPanelWidget(tr("Z-Clipping"), m_zClipPanel);
    m_colorOverlay  = new OverlayPanelWidget(tr("Color Range"), m_colorPanel);

    // Register with viewport (reparents, positions, shows)
    m_viewport->registerOverlay(m_statsOverlay);
    m_viewport->registerOverlay(m_renderOverlay);
    m_viewport->registerOverlay(m_zClipOverlay);
    m_viewport->registerOverlay(m_colorOverlay);

    // Hide all 4 by default (they'll be toggled from View menu)
    m_statsOverlay->hide();
    m_renderOverlay->hide();
    m_zClipOverlay->hide();
    m_colorOverlay->hide();
    m_viewport->updateOverlayPositions();

    // Close button → hide + sync menu action
    connect(m_statsOverlay, &OverlayPanelWidget::closeRequested,
            this, [this]() {
        if (m_statsOverlay) {
            m_statsOverlay->hide();
            m_viewport->updateOverlayPositions();
            if (m_statsViewAction) m_statsViewAction->setChecked(false);
        }
    });
    connect(m_renderOverlay, &OverlayPanelWidget::closeRequested,
            this, [this]() {
        if (m_renderOverlay) {
            m_renderOverlay->hide();
            m_viewport->updateOverlayPositions();
            if (m_renderViewAction) m_renderViewAction->setChecked(false);
        }
    });
    connect(m_zClipOverlay, &OverlayPanelWidget::closeRequested,
            this, [this]() {
        if (m_zClipOverlay) {
            m_zClipOverlay->hide();
            m_viewport->updateOverlayPositions();
            if (m_zClipViewAction) m_zClipViewAction->setChecked(false);
        }
    });
    connect(m_colorOverlay, &OverlayPanelWidget::closeRequested,
            this, [this]() {
        if (m_colorOverlay) {
            m_colorOverlay->hide();
            m_viewport->updateOverlayPositions();
            if (m_colorViewAction) m_colorViewAction->setChecked(false);
        }
    });

    // ── Existing floating overlay panels ──────────────────────────────
    m_autoLoopPanel = new AutoLoopClosurePanel(m_manager, nullptr);
    m_edgeListPanel = new EdgeListPanel(m_manager, nullptr);

    m_autoLoopOverlay = new OverlayPanelWidget(tr("Auto Loop Closure"), m_autoLoopPanel);
    m_edgeListOverlay = new OverlayPanelWidget(tr("Loop Edges"), m_edgeListPanel);

    m_viewport->registerOverlay(m_autoLoopOverlay);
    m_viewport->registerOverlay(m_edgeListOverlay);

    // Default hidden (toggled from View menu)
    m_autoLoopOverlay->hide();
    m_edgeListOverlay->hide();
    m_viewport->updateOverlayPositions();

    connect(m_autoLoopOverlay, &OverlayPanelWidget::closeRequested,
            this, [this]() {
        if (m_autoLoopOverlay) {
            m_autoLoopOverlay->hide();
            m_viewport->updateOverlayPositions();
            if (m_autoLoopViewAction) m_autoLoopViewAction->setChecked(false);
        }
    });
    connect(m_edgeListOverlay, &OverlayPanelWidget::closeRequested,
            this, [this]() {
        if (m_edgeListOverlay) {
            m_edgeListOverlay->hide();
            m_viewport->updateOverlayPositions();
            if (m_edgeListViewAction) m_edgeListViewAction->setChecked(false);
        }
    });

    // Refresh viewport when a loop edge is inserted by auto detection
    connect(m_autoLoopPanel, &AutoLoopClosurePanel::loopEdgeInserted,
            this, [this]() {
        m_viewport->refreshScene();
        m_viewport->rebuildPointClouds();
        m_edgeListPanel->refreshList();
        statusBar()->showMessage(tr("Loop edge inserted by auto detection"), 3000);
    });

    // Sphere highlight: blue=source, green=candidates during auto loop search
    connect(m_autoLoopPanel, &AutoLoopClosurePanel::loopDetectionStatus,
            this, [this](long sourceId, QVector<long> candidateIds) {
        std::vector<long> vec(candidateIds.begin(), candidateIds.end());
        m_viewport->setLoopHighlight(sourceId, vec);
        m_viewport->refreshScene();
    });

    // Refresh viewport when hidden edges change
    connect(m_edgeListPanel, &EdgeListPanel::hiddenEdgesChanged,
            this, [this]() {
        m_viewport->setHiddenEdges(m_edgeListPanel->hiddenEdgeIds());
        m_viewport->refreshScene();
    });

    // Status bar
    statusBar()->showMessage(tr("Ready — open a map folder to begin"));
}

// ---------------------------------------------------------------------------
// Menus
// ---------------------------------------------------------------------------

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

    auto* savePoseAction = fileMenu->addAction(tr("Save Pose Graph..."));
    savePoseAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_S));
    connect(savePoseAction, &QAction::triggered, this, &MainWindow::onSavePoseGraph);

    auto* saveMapAction = fileMenu->addAction(tr("Save Map..."));
    saveMapAction->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_S));
    connect(saveMapAction, &QAction::triggered, this, &MainWindow::onSaveMap);

    fileMenu->addSeparator();

    auto* quitAction = fileMenu->addAction(tr("&Quit"));
    quitAction->setShortcut(QKeySequence::Quit);
    connect(quitAction, &QAction::triggered, qApp, &QApplication::quit);

    // ---- View menu ----
    auto* viewMenu = menuBar()->addMenu(tr("&View"));

    auto* resetCamAction = viewMenu->addAction(tr("&Reset Camera"));
    resetCamAction->setShortcut(QKeySequence(Qt::Key_R));
    connect(resetCamAction, &QAction::triggered, this, &MainWindow::onResetCamera);

    // Orthographic view toggle
    m_orthoViewAction = viewMenu->addAction(tr("Orthographic View"));
    m_orthoViewAction->setCheckable(true);
    m_orthoViewAction->setChecked(false);  // default: perspective
    connect(m_orthoViewAction, &QAction::toggled, this, [this](bool checked) {
        m_viewport->setUseOrthographic(checked);
    });

    viewMenu->addSeparator();

    // Graph Statistics toggle
    m_statsViewAction = viewMenu->addAction(tr("Graph Statistics"));
    m_statsViewAction->setCheckable(true);
    m_statsViewAction->setChecked(false);
    connect(m_statsViewAction, &QAction::toggled, this, [this](bool checked) {
        if (m_statsOverlay) {
            m_statsOverlay->setVisible(checked);
            m_viewport->updateOverlayPositions();
        }
    });

    // Rendering toggle
    m_renderViewAction = viewMenu->addAction(tr("Rendering"));
    m_renderViewAction->setCheckable(true);
    m_renderViewAction->setChecked(false);
    connect(m_renderViewAction, &QAction::toggled, this, [this](bool checked) {
        if (m_renderOverlay) {
            m_renderOverlay->setVisible(checked);
            m_viewport->updateOverlayPositions();
        }
    });

    // Z-Clipping toggle
    m_zClipViewAction = viewMenu->addAction(tr("Z-Clipping"));
    m_zClipViewAction->setCheckable(true);
    m_zClipViewAction->setChecked(false);
    connect(m_zClipViewAction, &QAction::toggled, this, [this](bool checked) {
        if (m_zClipOverlay) {
            m_zClipOverlay->setVisible(checked);
            m_viewport->updateOverlayPositions();
        }
    });

    // Color Range toggle
    m_colorViewAction = viewMenu->addAction(tr("Color Range"));
    m_colorViewAction->setCheckable(true);
    m_colorViewAction->setChecked(false);
    connect(m_colorViewAction, &QAction::toggled, this, [this](bool checked) {
        if (m_colorOverlay) {
            m_colorOverlay->setVisible(checked);
            m_viewport->updateOverlayPositions();
        }
    });

    viewMenu->addSeparator();

    // Auto Loop Closure toggle
    m_autoLoopViewAction = viewMenu->addAction(tr("Auto Loop Closure Panel"));
    m_autoLoopViewAction->setCheckable(true);
    m_autoLoopViewAction->setChecked(false);
    connect(m_autoLoopViewAction, &QAction::toggled, this, [this](bool checked) {
        if (m_autoLoopOverlay) {
            m_autoLoopOverlay->setVisible(checked);
            m_viewport->updateOverlayPositions();
        }
    });

    // Loop Edges toggle
    m_edgeListViewAction = viewMenu->addAction(tr("Loop Edges Panel"));
    m_edgeListViewAction->setCheckable(true);
    m_edgeListViewAction->setChecked(false);
    connect(m_edgeListViewAction, &QAction::toggled, this, [this](bool checked) {
        if (m_edgeListOverlay) {
            m_edgeListOverlay->setVisible(checked);
            m_viewport->updateOverlayPositions();
        }
    });

    // ---- Graph menu ----
    auto* graphMenu = menuBar()->addMenu(tr("&Graph"));

    auto* optimizeAction = graphMenu->addAction(tr("&Optimize"));
    optimizeAction->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_O));
    m_optimizeAction = optimizeAction;
    connect(optimizeAction, &QAction::triggered, this, &MainWindow::onOptimize);

    // Submap window size configuration
    graphMenu->addSeparator();
    auto* submapWindowAction = graphMenu->addAction(tr("Submap Window Size..."));
    connect(submapWindowAction, &QAction::triggered, this, [this]() {
        QDialog dlg(this);
        dlg.setWindowTitle(tr("Submap Merge Window"));
        dlg.setModal(true);

        auto* layout = new QFormLayout(&dlg);

        auto* label = new QLabel(
            tr("Number of adjacent keyframes to merge on each side\n"
               "of the selected vertex for loop closure matching.\n"
               "N = 1 (default) merges 3 keyframes: center-1, center, center+1.\n"
               "N = 0 merges only the selected keyframe itself."));
        label->setWordWrap(true);
        layout->addRow(label);

        auto* spinBox = new QSpinBox;
        spinBox->setRange(0, 10);
        spinBox->setValue(m_submapWindowHalfSize);
        spinBox->setSuffix(tr(" keyframe(s) each side"));
        layout->addRow(tr("Window half-size:"), spinBox);

        auto* buttons = new QDialogButtonBox(
            QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
        layout->addRow(buttons);
        connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
        connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

        if (dlg.exec() == QDialog::Accepted) {
            m_submapWindowHalfSize = spinBox->value();
            m_viewport->setHighlightWindowHalf(m_submapWindowHalfSize);
            statusBar()->showMessage(
                tr("Submap window size set to ±%1 keyframe(s)")
                    .arg(m_submapWindowHalfSize), 3000);
        }
    });
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
    if (m_autoLoopPanel) {
        m_autoLoopPanel->stopDetection();
    }

    m_manager->closeMap();
    m_viewport->onGraphClosed();
    m_viewport->setLoopHighlight(-1, {});  // clear loop highlights
    if (m_edgeListPanel) {
        m_edgeListPanel->clearList();
    }
    m_loopBeginVertexId = -1;
    statusBar()->showMessage(tr("Map closed"));
}

void MainWindow::onSavePoseGraph() {
    if (!m_manager->isLoaded()) {
        statusBar()->showMessage(tr("No graph loaded"), 3000);
        return;
    }

    QString path = QFileDialog::getSaveFileName(
        this, tr("Save Pose Graph"), QString(),
        tr("Pose Graph Files (*.g2o);;All Files (*)"));
    if (path.isEmpty()) return;

    try {
        m_manager->graph()->save(path.toStdString());
        statusBar()->showMessage(
            tr("Pose graph saved: %1").arg(path), 5000);
    } catch (const std::exception& e) {
        statusBar()->showMessage(
            tr("Save failed: %1").arg(e.what()), 5000);
    }
}

void MainWindow::onSaveMap() {
    if (!m_manager->isLoaded()) {
        statusBar()->showMessage(tr("No graph loaded"), 3000);
        return;
    }

    QString dir = QFileDialog::getExistingDirectory(
        this, tr("Save Map Directory"), QString(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    if (dir.isEmpty()) return;

    try {
        auto* graph = m_manager->graph();
        graph->dump(dir.toStdString(), *m_manager->progress());
        statusBar()->showMessage(
            tr("Map saved: %1").arg(dir), 5000);
    } catch (const std::exception& e) {
        statusBar()->showMessage(
            tr("Save failed: %1").arg(e.what()), 5000);
    }
}

// ---------------------------------------------------------------------------
// Slots — Graph operations
// ---------------------------------------------------------------------------

void MainWindow::onOptimize() {
    if (!m_manager->isLoaded()) {
        statusBar()->showMessage(tr("No graph loaded"));
        return;
    }

    if (m_optimizePending) {
        statusBar()->showMessage(tr("Optimization already in progress"), 3000);
        return;
    }

    m_optimizePending = true;
    if (m_optimizeAction) m_optimizeAction->setEnabled(false);
    statusBar()->showMessage(tr("Optimizing (background)..."));

    auto* graph = m_manager->graph();
    if (!graph) {
        m_optimizePending = false;
        if (m_optimizeAction) m_optimizeAction->setEnabled(true);
        return;
    }

    // Run g2o optimization in a detached thread to keep UI responsive.
    // Post scene refresh back to the main thread when done.
    std::thread([this, graph]() {
        graph->optimize();
        QMetaObject::invokeMethod(this, [this]() {
            m_optimizePending = false;
            if (m_optimizeAction) m_optimizeAction->setEnabled(true);
            m_viewport->refreshScene();
            m_viewport->rebuildPointClouds();
            statusBar()->showMessage(tr("Optimization complete"), 3000);
        }, Qt::QueuedConnection);
    }).detach();
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
    m_viewport->setHighlightWindowHalf(m_submapWindowHalfSize);
    if (m_edgeListPanel) {
        m_edgeListPanel->refreshList();
    }
}

void MainWindow::onLoadingFailed(const QString& error) {
    statusBar()->showMessage(tr("Loading failed: %1").arg(error));
    QMessageBox::warning(this, tr("Load Error"), error);
}

void MainWindow::onLogMessage(const QString& message) {
    statusBar()->showMessage(message, 5000);
}
