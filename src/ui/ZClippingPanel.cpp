#include "ui/ZClippingPanel.h"
#include "ui/ViewportWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>

ZClippingPanel::ZClippingPanel(ViewportWidget* viewport, QWidget* parent)
    : QWidget(parent), m_viewport(viewport) {
    setupUi();

    // Z-clip controls → ViewportWidget
    connect(m_zClipCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setZClipping);

    connect(m_zClipMinSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setZClipMin);
    connect(m_zClipMaxSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setZClipMax);

    // Cloud data range → update UI labels & spinbox ranges
    connect(m_viewport, &ViewportWidget::cloudDataReady,
            this, &ZClippingPanel::onCloudDataReady);
}

void ZClippingPanel::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(8, 8, 8, 8);

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
    mainLayout->addStretch();
}

void ZClippingPanel::onCloudDataReady(float dataZMin, float dataZMax) {
    m_dataZRangeLabel->setText(
        tr("Data Z: %1 .. %2")
            .arg(dataZMin, 0, 'f', 2)
            .arg(dataZMax, 0, 'f', 2));

    // Block signals to avoid triggering updates during initialization
    m_zClipMinSpinBox->blockSignals(true);
    m_zClipMaxSpinBox->blockSignals(true);

    m_zClipMinSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_zClipMaxSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_zClipMinSpinBox->setValue(static_cast<double>(dataZMin));
    m_zClipMaxSpinBox->setValue(static_cast<double>(dataZMax));

    m_zClipMinSpinBox->blockSignals(false);
    m_zClipMaxSpinBox->blockSignals(false);
}
