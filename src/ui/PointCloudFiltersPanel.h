/**
 * @file PointCloudFiltersPanel.h
 * @brief 点云过滤面板头文件
 *
 * 合并了 Z-Clipping 和 Elevation Color Range 两个面板，
 * 在一个面板中以两个 QGroupBox 分组展示。
 */

#pragma once

#include <QWidget>
#include <QLabel>
#include <QCheckBox>
#include <QDoubleSpinBox>

class ViewportWidget;

class PointCloudFiltersPanel : public QWidget {
    Q_OBJECT

public:
    explicit PointCloudFiltersPanel(ViewportWidget* viewport, QWidget* parent = nullptr);

public slots:
    void onCloudDataReady(float dataZMin, float dataZMax);

private:
    void setupUi();

    ViewportWidget* m_viewport;

    // --- Z-Clipping 控件 ---
    QCheckBox*       m_zClipCb;
    QLabel*          m_dataZRangeLabel;
    QDoubleSpinBox*  m_zClipMinSpinBox;
    QDoubleSpinBox*  m_zClipMaxSpinBox;

    // --- Color Range 控件 ---
    QCheckBox*       m_autoColorRangeCb;
    QDoubleSpinBox*  m_colorZMinSpinBox;
    QDoubleSpinBox*  m_colorZMaxSpinBox;
};
