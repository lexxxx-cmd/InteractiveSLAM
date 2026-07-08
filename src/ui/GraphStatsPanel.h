#pragma once

#include <QWidget>
#include <QLabel>

class ViewportWidget;
class GraphManager;

/// @brief Floating panel showing graph statistics (vertex/edge/keyframe/FPS counts).
class GraphStatsPanel : public QWidget {
    Q_OBJECT

public:
    explicit GraphStatsPanel(GraphManager* manager, ViewportWidget* viewport,
                             QWidget* parent = nullptr);

public slots:
    void onStatsChanged();
    void onLoadingStateChanged();

private:
    void setupUi();

    GraphManager* m_manager;
    ViewportWidget* m_viewport;

    QLabel* m_vertexCountLabel;
    QLabel* m_edgeCountLabel;
    QLabel* m_keyframeCountLabel;
    QLabel* m_fpsLabel;
    QLabel* m_loadingLabel;
};
