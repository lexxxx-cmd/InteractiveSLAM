#include "ui/ColorRangePanel.h"
#include "ui/ViewportWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QLabel>

ColorRangePanel::ColorRangePanel(ViewportWidget* viewport, QWidget* parent)
    : QWidget(parent), m_viewport(viewport) {
    setupUi();

    // Auto color range toggle
    connect(m_autoColorRangeCb, &QCheckBox::toggled, this, [this](bool checked) {
        m_colorZMinSpinBox->setEnabled(!checked);
        m_colorZMaxSpinBox->setEnabled(!checked);
        m_viewport->setAutoColorRange(checked);
    });

    // Color range spinboxes → ViewportWidget
    connect(m_colorZMinSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setColorZMin);
    connect(m_colorZMaxSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setColorZMax);

    // Cloud data range → update spinbox ranges
    connect(m_viewport, &ViewportWidget::cloudDataReady,
            this, &ColorRangePanel::onCloudDataReady);
}

void ColorRangePanel::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(8, 8, 8, 8);

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

void ColorRangePanel::onCloudDataReady(float dataZMin, float dataZMax) {
    // Block signals to avoid triggering updates during initialization
    m_colorZMinSpinBox->blockSignals(true);
    m_colorZMaxSpinBox->blockSignals(true);

    m_colorZMinSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_colorZMaxSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_colorZMinSpinBox->setValue(static_cast<double>(dataZMin));
    m_colorZMaxSpinBox->setValue(static_cast<double>(dataZMax));

    m_colorZMinSpinBox->blockSignals(false);
    m_colorZMaxSpinBox->blockSignals(false);
}
