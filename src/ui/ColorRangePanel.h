/**
 * @file ColorRangePanel.h
 * @brief 颜色范围设置面板头文件
 *
 * ColorRangePanel 是一个浮动面板，用于控制点云渲染的高程颜色范围。
 * 支持自动颜色范围（根据数据自动调整）和手动设置最小/最大 Z 值。
 *
 * 当 ViewportWidget 发射 cloudDataReady 信号时，
 * 面板根据实际点云数据的 Z 范围自动更新控件的取值范围。
 */

#pragma once

#include <QWidget>
#include <QCheckBox>
#include <QDoubleSpinBox>

class ViewportWidget;

/**
 * @brief 高程颜色范围浮动面板
 *
 * 提供控制：
 * - 自动颜色范围开关
 * - 最小 Z 值设置
 * - 最大 Z 值设置
 *
 * 当自动模式启用时，最小/最大 Z 值控件被禁用，
 * 颜色范围根据点云数据自动适配。
 */
class ColorRangePanel : public QWidget {
    Q_OBJECT

public:
    explicit ColorRangePanel(ViewportWidget* viewport, QWidget* parent = nullptr);

public slots:
    /**
     * @brief 点云数据就绪回调
     * @param dataZMin 点云数据 Z 最小值
     * @param dataZMax 点云数据 Z 最大值
     */
    void onCloudDataReady(float dataZMin, float dataZMax);

private:
    void setupUi();  ///< 初始化 UI 控件

    ViewportWidget* m_viewport;  ///< 关联的 3D 视口

    QCheckBox* m_autoColorRangeCb;   ///< 自动颜色范围复选框
    QDoubleSpinBox* m_colorZMinSpinBox;  ///< 颜色范围 Z 最小值旋钮
    QDoubleSpinBox* m_colorZMaxSpinBox;  ///< 颜色范围 Z 最大值旋钮
};
