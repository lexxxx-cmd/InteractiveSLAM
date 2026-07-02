#pragma once

#include <QWidget>
#include <QLabel>
#include <QCheckBox>
#include <QSlider>

class ViewportWidget;

/// @brief Floating panel for rendering controls (visibility toggles, sliders).
class RenderingPanel : public QWidget {
    Q_OBJECT

public:
    explicit RenderingPanel(ViewportWidget* viewport, QWidget* parent = nullptr);

private:
    void setupUi();

    ViewportWidget* m_viewport;

    QCheckBox* m_drawVerticesCb;
    QCheckBox* m_drawEdgesCb;
    QCheckBox* m_drawCloudsCb;
    QCheckBox* m_drawSE3EdgesCb;

    QSlider* m_sphereRadiusSlider;
    QLabel* m_sphereRadiusLabel;

    QSlider* m_edgeWidthSlider;
    QLabel* m_edgeWidthLabel;

    QSlider* m_pointSizeSlider;
    QLabel* m_pointSizeLabel;

    QSlider* m_pointOpacitySlider;
    QLabel* m_pointOpacityLabel;
};
