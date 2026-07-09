/**
 * @file ZClippingPanel.cpp
 * @brief Z 轴裁剪面板实现
 *
 * 实现 Z 轴裁剪控制的 UI 和信号连接。
 * 当用户启用裁剪并设置 Z 范围后，ViewportWidget 会将超出范围的
 * 点云点剔除（不渲染）。
 * 当视口加载新点云数据时，自动更新标签和 spinbox 的取值范围。
 */

#include "ui/ZClippingPanel.h"
#include "ui/ViewportWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>

/**
 * @brief 构造函数
 *
 * 连接裁剪开关信号、Z 值变化信号以及视口的 cloudDataReady 信号。
 */
ZClippingPanel::ZClippingPanel(ViewportWidget* viewport, QWidget* parent)
    : QWidget(parent), m_viewport(viewport) {
    setupUi();

    // Z 裁剪开关 → ViewportWidget
    connect(m_zClipCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setZClipping);

    // Z 裁剪值变化 → ViewportWidget
    connect(m_zClipMinSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setZClipMin);
    connect(m_zClipMaxSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setZClipMax);

    // 点云数据范围 → 更新 UI 标签和 spinbox 范围
    connect(m_viewport, &ViewportWidget::cloudDataReady,
            this, &ZClippingPanel::onCloudDataReady);
}

/**
 * @brief 创建 UI 布局
 *
 * 包含启用复选框、数据 Z 范围标签、最小/最大 Z 值旋钮。
 */
void ZClippingPanel::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(8, 8, 8, 8);

    auto* clipGroup = new QGroupBox(tr("Z-Clipping"));
    auto* clipLayout = new QVBoxLayout(clipGroup);

    // 启用 Z 裁剪（默认关闭）
    m_zClipCb = new QCheckBox(tr("Enable Z-Clipping"));
    m_zClipCb->setChecked(false);
    clipLayout->addWidget(m_zClipCb);

    // 数据 Z 范围标签
    m_dataZRangeLabel = new QLabel(tr("Data Z: (no data)"));
    clipLayout->addWidget(m_dataZRangeLabel);

    // 最小 Z 值
    auto* clipMinRow = new QHBoxLayout;
    clipMinRow->addWidget(new QLabel(tr("Min Z:")));
    m_zClipMinSpinBox = new QDoubleSpinBox;
    m_zClipMinSpinBox->setRange(-10000.0, 10000.0);
    m_zClipMinSpinBox->setDecimals(2);
    m_zClipMinSpinBox->setSingleStep(0.5);
    m_zClipMinSpinBox->setValue(-1.5);
    clipMinRow->addWidget(m_zClipMinSpinBox);
    clipLayout->addLayout(clipMinRow);

    // 最大 Z 值
    auto* clipMaxRow = new QHBoxLayout;
    clipMaxRow->addWidget(new QLabel(tr("Max Z:")));
    m_zClipMaxSpinBox = new QDoubleSpinBox;
    m_zClipMaxSpinBox->setRange(-10000.0, 10000.0);
    m_zClipMaxSpinBox->setDecimals(2);
    m_zClipMaxSpinBox->setSingleStep(0.5);
    m_zClipMaxSpinBox->setValue(5.0);
    clipMaxRow->addWidget(m_zClipMaxSpinBox);
    clipLayout->addLayout(clipMaxRow);

    mainLayout->addWidget(clipGroup);
    mainLayout->addStretch();
}

/**
 * @brief 点云数据就绪回调
 *
 * 更新 Z 范围标签和 spinbox 的取值范围与初始值。
 * 通过 blockSignals 防止初始化过程中触发值变化信号。
 */
void ZClippingPanel::onCloudDataReady(float dataZMin, float dataZMax) {
    m_dataZRangeLabel->setText(
        tr("Data Z: %1 .. %2")
            .arg(dataZMin, 0, 'f', 2)
            .arg(dataZMax, 0, 'f', 2));

    // 阻止信号，避免初始化过程中触发更新
    m_zClipMinSpinBox->blockSignals(true);
    m_zClipMaxSpinBox->blockSignals(true);

    m_zClipMinSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_zClipMaxSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_zClipMinSpinBox->setValue(static_cast<double>(dataZMin));
    m_zClipMaxSpinBox->setValue(static_cast<double>(dataZMax));

    m_zClipMinSpinBox->blockSignals(false);
    m_zClipMaxSpinBox->blockSignals(false);
}
