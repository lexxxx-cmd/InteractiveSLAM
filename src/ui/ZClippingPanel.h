/**
 * @file ZClippingPanel.h
 * @brief Z 轴裁剪面板头文件
 *
 * ZClippingPanel 是一个浮动面板，用于控制 3D 视口中的 Z 轴裁剪。
 * 用户可以通过启用裁剪并设置最小/最大 Z 值来过滤掉特定高程范围外的点云，
 * 便于观察特定高度层的数据。
 *
 * 当 ViewportWidget 发射 cloudDataReady 信号时，
 * 面板根据实际点云数据的 Z 范围自动更新控件的取值范围。
 */

#pragma once

#include <QWidget>
#include <QLabel>
#include <QCheckBox>
#include <QDoubleSpinBox>

class ViewportWidget;

/**
 * @brief Z 轴裁剪浮动面板
 *
 * 提供控制：
 * - Z 裁剪启用/禁用开关
 * - 数据 Z 范围显示标签
 * - 最小裁剪 Z 值设置
 * - 最大裁剪 Z 值设置
 */
class ZClippingPanel : public QWidget {
    Q_OBJECT

public:
    explicit ZClippingPanel(ViewportWidget* viewport, QWidget* parent = nullptr);

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

    QCheckBox* m_zClipCb;              ///< 启用 Z 裁剪复选框
    QLabel* m_dataZRangeLabel;          ///< 数据 Z 范围标签（显示实际数据范围）
    QDoubleSpinBox* m_zClipMinSpinBox;  ///< Z 裁剪最小值旋钮
    QDoubleSpinBox* m_zClipMaxSpinBox;  ///< Z 裁剪最大值旋钮
};
