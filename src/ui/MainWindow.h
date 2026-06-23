#pragma once

#include <QMainWindow>
#include <QStatusBar>
#include <QMenuBar>
#include <QDockWidget>

class GraphManager;
class ViewportWidget;
class GraphInfoPanel;
class AutoLoopClosurePanel;

/// @brief Main application window — replaces Main.qml.
///        Layout: central ViewportWidget + left-docked GraphInfoPanel
///                + right-docked AutoLoopClosurePanel.
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(GraphManager* manager, QWidget* parent = nullptr);

private slots:
    void onOpenMap();
    void onCloseMap();
    void onSavePoseGraph();
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

    // Auto loop closure
    AutoLoopClosurePanel* m_autoLoopPanel = nullptr;
    QDockWidget* m_autoLoopDock = nullptr;

    long m_loopBeginVertexId = -1;  // -1 = 未选择 Loop Begin
};
