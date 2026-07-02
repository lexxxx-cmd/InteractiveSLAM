#pragma once

#include <QWidget>
#include <QLabel>
#include <QCheckBox>
#include <QSlider>
#include <QPushButton>
#include <QDoubleSpinBox>

class ViewportWidget;
class GraphManager;

/// @brief Left panel providing graph statistics and rendering controls.
///        Replaces the QML statsPanel + rendering config popup.
class GraphInfoPanel : public QWidget {
    Q_OBJECT

public:
    explicit GraphInfoPanel(GraphManager* manager, ViewportWidget* viewport,
                            QWidget* parent = nullptr);

public slots:
    void onStatsChanged();    // connected to GraphManager::statsChanged
    void onLoadingStateChanged();  // connected to GraphManager::isLoadingChanged
    void onCloudDataReady(float dataZMin, float dataZMax);

signals:
    void resetCameraRequested();

private:
    void setupUi();

    GraphManager* m_manager;
    ViewportWidget* m_viewport;

    // Statistics
    QLabel* m_vertexCountLabel;
    QLabel* m_edgeCountLabel;
    QLabel* m_keyframeCountLabel;
    QLabel* m_fpsLabel;

    // Rendering toggles
    QCheckBox* m_drawVerticesCb;
    QCheckBox* m_drawEdgesCb;
    QCheckBox* m_drawCloudsCb;
    QCheckBox* m_drawSE3EdgesCb;

    // Sphere radius
    QSlider* m_sphereRadiusSlider;
    QLabel* m_sphereRadiusLabel;

    // Edge width
    QSlider* m_edgeWidthSlider;
    QLabel* m_edgeWidthLabel;

    // Point settings
    QSlider* m_pointSizeSlider;
    QLabel* m_pointSizeLabel;
    QSlider* m_pointOpacitySlider;
    QLabel* m_pointOpacityLabel;

    // Z-Clipping controls
    QCheckBox* m_zClipCb;
    QDoubleSpinBox* m_zClipMinSpinBox;
    QDoubleSpinBox* m_zClipMaxSpinBox;

    // Color elevation range controls
    QLabel* m_dataZRangeLabel;
    QCheckBox* m_autoColorRangeCb;
    QDoubleSpinBox* m_colorZMinSpinBox;
    QDoubleSpinBox* m_colorZMaxSpinBox;

    // Loading indicator
    QLabel* m_loadingLabel;
};
