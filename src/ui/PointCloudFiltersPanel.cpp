/**
 * @file PointCloudFiltersPanel.cpp
 * @brief 点云过滤面板实现
 *
 * 将 Z-Clipping 和 Elevation Color Range 合并到一个面板中，
 * 包含两个 QGroupBox 分组，所有信号槽连接与原始面板相同。
 */

#include "ui/PointCloudFiltersPanel.h"
#include "ui/ViewportWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>

PointCloudFiltersPanel::PointCloudFiltersPanel(ViewportWidget* viewport, QWidget* parent)
    : QWidget(parent), m_viewport(viewport) {
    setupUi();

    // === Z-Clipping 信号 ===
    connect(m_zClipCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setZClipping);

    connect(m_zClipMinSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setZClipMin);
    connect(m_zClipMaxSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setZClipMax);

    // === Color Range 信号 ===
    connect(m_autoColorRangeCb, &QCheckBox::toggled, this, [this](bool checked) {
        m_colorZMinSpinBox->setEnabled(!checked);
        m_colorZMaxSpinBox->setEnabled(!checked);
        m_viewport->setAutoColorRange(checked);
    });

    connect(m_colorZMinSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setColorZMin);
    connect(m_colorZMaxSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setColorZMax);

    // === 点云数据范围 ===
    connect(m_viewport, &ViewportWidget::cloudDataReady,
            this, &PointCloudFiltersPanel::onCloudDataReady);
}

void PointCloudFiltersPanel::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(8, 8, 8, 8);

    // ---- Z-Clipping 分组 ----
    auto* clipGroup = new QGroupBox(tr("Z-Clipping"));
    auto* clipLayout = new QVBoxLayout(clipGroup);

    m_zClipCb = new QCheckBox(tr("Enable Z-Clipping"));
    m_zClipCb->setChecked(false);
    clipLayout->addWidget(m_zClipCb);

    m_dataZRangeLabel = new QLabel(tr("Data Z: (no data)"));
    clipLayout->addWidget(m_dataZRangeLabel);

    auto* clipMinRow = new QHBoxLayout;
    clipMinRow->addWidget(new QLabel(tr("Min Z:")));
    m_zClipMinSpinBox = new QDoubleSpinBox;
    m_zClipMinSpinBox->setRange(-10000.0, 10000.0);
    m_zClipMinSpinBox->setDecimals(2);
    m_zClipMinSpinBox->setSingleStep(0.5);
    m_zClipMinSpinBox->setValue(-1.5);
    clipMinRow->addWidget(m_zClipMinSpinBox);
    clipLayout->addLayout(clipMinRow);

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

    // ---- Elevation Color Range 分组 ----
    auto* colorGroup = new QGroupBox(tr("Elevation Color Range"));
    auto* colorLayout = new QVBoxLayout(colorGroup);

    m_autoColorRangeCb = new QCheckBox(tr("Auto Color Range"));
    m_autoColorRangeCb->setChecked(true);
    colorLayout->addWidget(m_autoColorRangeCb);

    auto* colorMinRow = new QHBoxLayout;
    colorMinRow->addWidget(new QLabel(tr("Min Z:")));
    m_colorZMinSpinBox = new QDoubleSpinBox;
    m_colorZMinSpinBox->setRange(-10000.0, 10000.0);
    m_colorZMinSpinBox->setDecimals(2);
    m_colorZMinSpinBox->setSingleStep(0.5);
    m_colorZMinSpinBox->setEnabled(false);
    colorMinRow->addWidget(m_colorZMinSpinBox);
    colorLayout->addLayout(colorMinRow);

    auto* colorMaxRow = new QHBoxLayout;
    colorMaxRow->addWidget(new QLabel(tr("Max Z:")));
    m_colorZMaxSpinBox = new QDoubleSpinBox;
    m_colorZMaxSpinBox->setRange(-10000.0, 10000.0);
    m_colorZMaxSpinBox->setDecimals(2);
    m_colorZMaxSpinBox->setSingleStep(0.5);
    m_colorZMaxSpinBox->setEnabled(false);
    colorMaxRow->addWidget(m_colorZMaxSpinBox);
    colorLayout->addLayout(colorMaxRow);

    mainLayout->addWidget(colorGroup);
    mainLayout->addStretch();
}

void PointCloudFiltersPanel::onCloudDataReady(float dataZMin, float dataZMax) {
    // --- 更新 Z-Clipping 数据 ---
    m_dataZRangeLabel->setText(
        tr("Data Z: %1 .. %2")
            .arg(dataZMin, 0, 'f', 2)
            .arg(dataZMax, 0, 'f', 2));

    m_zClipMinSpinBox->blockSignals(true);
    m_zClipMaxSpinBox->blockSignals(true);
    m_zClipMinSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_zClipMaxSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_zClipMinSpinBox->setValue(static_cast<double>(dataZMin));
    m_zClipMaxSpinBox->setValue(static_cast<double>(dataZMax));
    m_zClipMinSpinBox->blockSignals(false);
    m_zClipMaxSpinBox->blockSignals(false);

    // --- 更新 Color Range 数据 ---
    m_colorZMinSpinBox->blockSignals(true);
    m_colorZMaxSpinBox->blockSignals(true);
    m_colorZMinSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_colorZMaxSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_colorZMinSpinBox->setValue(static_cast<double>(dataZMin));
    m_colorZMaxSpinBox->setValue(static_cast<double>(dataZMax));
    m_colorZMinSpinBox->blockSignals(false);
    m_colorZMaxSpinBox->blockSignals(false);
}
