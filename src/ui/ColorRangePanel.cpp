/**
 * @file ColorRangePanel.cpp
 * @brief 颜色范围设置面板实现
 *
 * 实现高程颜色范围控制的 UI 和信号连接。
 * 支持自动/手动模式切换，手动模式下通过 spinbox 设置 Z 范围。
 * 当视口加载新点云数据时自动更新 spinbox 的取值范围。
 */

#include "ui/ColorRangePanel.h"
#include "ui/ViewportWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>

/**
 * @brief 构造函数
 *
 * 连接自动范围切换信号（启用/禁用 spinbox）和 Z 值变化信号，
 * 以及视口的 cloudDataReady 信号。
 */
ColorRangePanel::ColorRangePanel(ViewportWidget* viewport, QWidget* parent)
    : QWidget(parent), m_viewport(viewport) {
    setupUi();

    // 自动颜色范围切换：自动模式下禁用 spinbox
    connect(m_autoColorRangeCb, &QCheckBox::toggled, this, [this](bool checked) {
        m_colorZMinSpinBox->setEnabled(!checked);
        m_colorZMaxSpinBox->setEnabled(!checked);
        m_viewport->setAutoColorRange(checked);
    });

    // 颜色范围 spinbox → ViewportWidget
    connect(m_colorZMinSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setColorZMin);
    connect(m_colorZMaxSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setColorZMax);

    // 点云数据范围 → 更新 spinbox 范围和初始值
    connect(m_viewport, &ViewportWidget::cloudDataReady,
            this, &ColorRangePanel::onCloudDataReady);
}

/**
 * @brief 创建 UI 布局
 *
 * 包含自动颜色范围复选框、最小 Z 值、最大 Z 值控件。
 */
void ColorRangePanel::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(8, 8, 8, 8);

    auto* colorGroup = new QGroupBox(tr("Elevation Color Range"));
    auto* colorLayout = new QVBoxLayout(colorGroup);

    // 自动颜色范围复选框（默认开启）
    m_autoColorRangeCb = new QCheckBox(tr("Auto Color Range"));
    m_autoColorRangeCb->setChecked(true);
    colorLayout->addWidget(m_autoColorRangeCb);

    // 最小 Z 值
    auto* colorMinRow = new QHBoxLayout;
    colorMinRow->addWidget(new QLabel(tr("Min Z:")));
    m_colorZMinSpinBox = new QDoubleSpinBox;
    m_colorZMinSpinBox->setRange(-10000.0, 10000.0);
    m_colorZMinSpinBox->setDecimals(2);
    m_colorZMinSpinBox->setSingleStep(0.5);
    m_colorZMinSpinBox->setEnabled(false);  // 自动模式下禁用
    colorMinRow->addWidget(m_colorZMinSpinBox);
    colorLayout->addLayout(colorMinRow);

    // 最大 Z 值
    auto* colorMaxRow = new QHBoxLayout;
    colorMaxRow->addWidget(new QLabel(tr("Max Z:")));
    m_colorZMaxSpinBox = new QDoubleSpinBox;
    m_colorZMaxSpinBox->setRange(-10000.0, 10000.0);
    m_colorZMaxSpinBox->setDecimals(2);
    m_colorZMaxSpinBox->setSingleStep(0.5);
    m_colorZMaxSpinBox->setEnabled(false);  // 自动模式下禁用
    colorMaxRow->addWidget(m_colorZMaxSpinBox);
    colorLayout->addLayout(colorMaxRow);

    mainLayout->addWidget(colorGroup);
    mainLayout->addStretch();
}

/**
 * @brief 点云数据就绪回调
 *
 * 更新 spinbox 的取值范围和初始值。
 * 通过 blockSignals 防止初始化过程中触发值变化信号。
 */
void ColorRangePanel::onCloudDataReady(float dataZMin, float dataZMax) {
    // 阻止信号，避免初始化过程中触发更新
    m_colorZMinSpinBox->blockSignals(true);
    m_colorZMaxSpinBox->blockSignals(true);

    m_colorZMinSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_colorZMaxSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_colorZMinSpinBox->setValue(static_cast<double>(dataZMin));
    m_colorZMaxSpinBox->setValue(static_cast<double>(dataZMax));

    m_colorZMinSpinBox->blockSignals(false);
    m_colorZMaxSpinBox->blockSignals(false);
}
