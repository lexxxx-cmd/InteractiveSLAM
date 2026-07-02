#pragma once

#include <QWidget>
#include <QLabel>
#include <QCheckBox>
#include <QDoubleSpinBox>

class ViewportWidget;

/// @brief Floating panel for Z-clipping controls.
class ZClippingPanel : public QWidget {
    Q_OBJECT

public:
    explicit ZClippingPanel(ViewportWidget* viewport, QWidget* parent = nullptr);

public slots:
    void onCloudDataReady(float dataZMin, float dataZMax);

private:
    void setupUi();

    ViewportWidget* m_viewport;

    QCheckBox* m_zClipCb;
    QLabel* m_dataZRangeLabel;
    QDoubleSpinBox* m_zClipMinSpinBox;
    QDoubleSpinBox* m_zClipMaxSpinBox;
};
