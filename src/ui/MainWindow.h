#pragma once

#include <QMainWindow>
#include <QStatusBar>
#include <QMenuBar>

class QAction;
class GraphManager;
class ViewportWidget;
class GraphStatsPanel;
class RenderingPanel;
class ZClippingPanel;
class ColorRangePanel;
class AutoLoopClosurePanel;
class EdgeListPanel;
class OverlayPanelWidget;

/// @brief Main application window — replaces Main.qml.
///        Layout: central ViewportWidget + floating overlay panels.
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

    // New overlay panels (content + wrapper)
    GraphStatsPanel*    m_statsPanel    = nullptr;
    RenderingPanel*     m_renderPanel   = nullptr;
    ZClippingPanel*     m_zClipPanel    = nullptr;
    ColorRangePanel*    m_colorPanel    = nullptr;

    AutoLoopClosurePanel* m_autoLoopPanel = nullptr;
    EdgeListPanel* m_edgeListPanel = nullptr;

    // View menu actions (stored for check-state sync with overlays)
    QAction* m_statsViewAction  = nullptr;
    QAction* m_renderViewAction = nullptr;
    QAction* m_zClipViewAction  = nullptr;
    QAction* m_colorViewAction  = nullptr;
    QAction* m_autoLoopViewAction = nullptr;
    QAction* m_edgeListViewAction = nullptr;

    // Overlay wrappers (floating over viewport)
    OverlayPanelWidget* m_statsOverlay  = nullptr;
    OverlayPanelWidget* m_renderOverlay = nullptr;
    OverlayPanelWidget* m_zClipOverlay  = nullptr;
    OverlayPanelWidget* m_colorOverlay  = nullptr;
    OverlayPanelWidget* m_autoLoopOverlay = nullptr;
    OverlayPanelWidget* m_edgeListOverlay = nullptr;

    long m_loopBeginVertexId = -1;  // -1 = 未选择 Loop Begin
    int m_submapWindowHalfSize = 1;  // ±N keyframes merged for loop closure matching
    QAction* m_optimizeAction = nullptr;
    bool m_optimizePending = false;
};
