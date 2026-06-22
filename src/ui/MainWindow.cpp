#include "ui/MainWindow.h"
#include "ui/ViewportWidget.h"
#include "ui/GraphInfoPanel.h"
#include "backend/graph_manager.hpp"

#include <QFileDialog>
#include <QMessageBox>
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
