/**
 * @file RenderingAdvancedDialogs.h
 * @brief 「高级设置 → 渲染相关」三个独立小对话框
 *
 *   - LodSettingsDialog      多级渲染（LOD）模式/层级
 *   - ZClipSettingsDialog    Z 轴裁剪（启用 + 范围）
 *   - ColorRangeSettingsDialog 高程颜色范围（自动/手动）
 *
 * 控件变化即时作用于 ViewportWidget。
 */
#pragma once

#include <QDialog>

class ViewportWidget;
class QComboBox;
class QDoubleSpinBox;
class QCheckBox;
class QLabel;

// ============================================================================
// 多级渲染（LOD）
// ============================================================================

class LodSettingsDialog : public QDialog {
    Q_OBJECT
public:
    explicit LodSettingsDialog(ViewportWidget* viewport, QWidget* parent = nullptr);

private:
    void setupUi();

    ViewportWidget* m_viewport;
    QComboBox* m_lodModeCombo = nullptr;
    QComboBox* m_lodLevelCombo = nullptr;
    QLabel*    m_lodLevelLabel = nullptr;
};

// ============================================================================
// Z 轴裁剪
// ============================================================================

class ZClipSettingsDialog : public QDialog {
    Q_OBJECT
public:
    explicit ZClipSettingsDialog(ViewportWidget* viewport, QWidget* parent = nullptr);

private:
    void setupUi();
    void onCloudDataReady(float dataZMin, float dataZMax);

    ViewportWidget* m_viewport;
    QCheckBox*      m_zClipCb = nullptr;
    QDoubleSpinBox* m_zClipMinSpin = nullptr;
    QDoubleSpinBox* m_zClipMaxSpin = nullptr;
    QLabel*         m_dataZRangeLabel = nullptr;
};

// ============================================================================
// 高程颜色范围
// ============================================================================

class ColorRangeSettingsDialog : public QDialog {
    Q_OBJECT
public:
    explicit ColorRangeSettingsDialog(ViewportWidget* viewport, QWidget* parent = nullptr);

private:
    void setupUi();
    void onCloudDataReady(float dataZMin, float dataZMax);

    ViewportWidget* m_viewport;
    QCheckBox*      m_autoColorRangeCb = nullptr;
    QDoubleSpinBox* m_colorZMinSpin = nullptr;
    QDoubleSpinBox* m_colorZMaxSpin = nullptr;
};
