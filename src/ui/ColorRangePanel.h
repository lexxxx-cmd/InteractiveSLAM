#pragma once

#include <QWidget>
#include <QCheckBox>
#include <QDoubleSpinBox>

class ViewportWidget;

/// @brief Floating panel for elevation color range controls.
class ColorRangePanel : public QWidget {
    Q_OBJECT

public:
    explicit ColorRangePanel(ViewportWidget* viewport, QWidget* parent = nullptr);

public slots:
    void onCloudDataReady(float dataZMin, float dataZMax);

private:
    void setupUi();

    ViewportWidget* m_viewport;

    QCheckBox* m_autoColorRangeCb;
    QDoubleSpinBox* m_colorZMinSpinBox;
    QDoubleSpinBox* m_colorZMaxSpinBox;
};
