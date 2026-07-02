#pragma once

#include <QMainWindow>
#include <QStatusBar>
#include <QMenuBar>
#include <QDockWidget>

class QAction;
class GraphManager;
class ViewportWidget;
class GraphInfoPanel;
class AutoLoopClosurePanel;
class EdgeListPanel;
class OverlayPanelWidget;

/// @brief Main application window — replaces Main.qml.
///        Layout: central ViewportWidget + left-docked GraphInfoPanel
///                + floating overlay panels (AutoLoopClosure / LoopEdges).
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(GraphManager* manager, QWidget* parent = nullptr);

private slots:
    void onOpenMap();
    void onCloseMap();
    void onSavePoseGraph();
    void onSaveMap();
    void onOptimize();
    void onResetCamera();
    void onLoadingStarted();
    void onLoadingSucceeded();
    void onLoadingFailed(const QString& error);
    void onLogMessage(const QString& message);

private:
    void setupMenus();
    void setupUi();

    GraphManager* m_manager;
    ViewportWidget* m_viewport;
    GraphInfoPanel* m_infoPanel;

    AutoLoopClosurePanel* m_autoLoopPanel = nullptr;
    EdgeListPanel* m_edgeListPanel = nullptr;

    // View menu actions (stored for check-state sync with overlays)
    QAction* m_autoLoopViewAction = nullptr;
    QAction* m_edgeListViewAction = nullptr;

    // Overlay wrappers (floating over viewport)
    OverlayPanelWidget* m_autoLoopOverlay = nullptr;
    OverlayPanelWidget* m_edgeListOverlay = nullptr;

    long m_loopBeginVertexId = -1;  // -1 = 未选择 Loop Begin
};
