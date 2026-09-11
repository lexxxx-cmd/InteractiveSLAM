/**
 * @file RenderingAdvancedDialogs.cpp
 * @brief 「高级设置 → 渲染相关」三个独立小对话框实现
 */
#include "ui/RenderingAdvancedDialogs.h"
#include "ui/ViewportWidget.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QLabel>
#include <QFormLayout>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QDialogButtonBox>

// ============================================================================
// LodSettingsDialog —— 多级渲染（LOD）模式/层级
// ============================================================================

LodSettingsDialog::LodSettingsDialog(ViewportWidget* viewport, QWidget* parent)
    : QDialog(parent), m_viewport(viewport) {
    setWindowTitle(tr("Multi-level Rendering (LOD)"));
    setupUi();

    // LOD 层级状态：构建完成与切换时更新
    connect(m_viewport, &ViewportWidget::lodLevelChanged,
            this, [this](int level, int levelCount) {
        m_lodLevelCombo->blockSignals(true);
        m_lodLevelCombo->clear();
        for (int i = 0; i < levelCount; ++i) {
            m_lodLevelCombo->addItem(i == 0 ? tr("Level 0 (full)")
                                            : tr("Level 1 (decimated)"));
        }
        m_lodLevelCombo->setCurrentIndex(qBound(0, level, levelCount - 1));
        m_lodLevelCombo->blockSignals(false);

        m_lodLevelLabel->setText(
            levelCount > 1 ? tr("%1 / %2").arg(level).arg(levelCount)
                           : tr("off"));

        m_lodLevelLabel->setText(
            levelCount > 1 ? tr("%1 / %2").arg(level).arg(levelCount)
                           : tr("off"));
    });
}

void LodSettingsDialog::setupUi() {
    auto* layout = new QVBoxLayout(this);

    auto* group = new QGroupBox(tr("Multi-level Rendering (LOD)"));
    auto* form = new QFormLayout(group);

    m_lodModeCombo = new QComboBox;
    m_lodModeCombo->addItem(tr("Auto (by distance)"));
    m_lodModeCombo->addItem(tr("Manual"));
    connect(m_lodModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int idx) { m_viewport->setLodMode(idx == 1); });
    form->addRow(tr("Mode:"), m_lodModeCombo);

    m_lodLevelCombo = new QComboBox;
    m_lodLevelCombo->addItem(tr("Level 0 (full)"));
    m_lodLevelCombo->addItem(tr("Level 1 (decimated)"));
    connect(m_lodLevelCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int idx) { m_viewport->setLodManualLevel(idx); });
    form->addRow(tr("Level:"), m_lodLevelCombo);

    m_lodLevelLabel = new QLabel(tr("—"));
    form->addRow(tr("Status:"), m_lodLevelLabel);

    layout->addWidget(group);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

// ============================================================================
// ZClipSettingsDialog —— Z 轴裁剪
// ============================================================================

ZClipSettingsDialog::ZClipSettingsDialog(ViewportWidget* viewport, QWidget* parent)
    : QDialog(parent), m_viewport(viewport) {
    setWindowTitle(tr("Z-Clipping"));
    setupUi();

    connect(m_viewport, &ViewportWidget::cloudDataReady,
            this, &ZClipSettingsDialog::onCloudDataReady);
}

void ZClipSettingsDialog::setupUi() {
    auto* layout = new QVBoxLayout(this);

    auto* group = new QGroupBox(tr("Z-Clipping"));
    auto* form = new QFormLayout(group);

    m_zClipCb = new QCheckBox(tr("Enable Z-Clipping"));
    connect(m_zClipCb, &QCheckBox::toggled, m_viewport, &ViewportWidget::setZClipping);
    form->addRow(m_zClipCb);

    m_dataZRangeLabel = new QLabel(tr("Data Z: (no data)"));
    form->addRow(m_dataZRangeLabel);

    m_zClipMinSpin = new QDoubleSpinBox;
    m_zClipMinSpin->setRange(-10000.0, 10000.0);
    m_zClipMinSpin->setDecimals(2);
    m_zClipMinSpin->setSingleStep(0.5);
    connect(m_zClipMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setZClipMin);
    form->addRow(tr("Min Z:"), m_zClipMinSpin);

    m_zClipMaxSpin = new QDoubleSpinBox;
    m_zClipMaxSpin->setRange(-10000.0, 10000.0);
    m_zClipMaxSpin->setDecimals(2);
    m_zClipMaxSpin->setSingleStep(0.5);
    connect(m_zClipMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setZClipMax);
    form->addRow(tr("Max Z:"), m_zClipMaxSpin);

    layout->addWidget(group);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

void ZClipSettingsDialog::onCloudDataReady(float dataZMin, float dataZMax) {
    m_dataZRangeLabel->setText(
        tr("Data Z: %1 .. %2").arg(dataZMin, 0, 'f', 2).arg(dataZMax, 0, 'f', 2));

    m_zClipMinSpin->blockSignals(true);
    m_zClipMaxSpin->blockSignals(true);
    m_zClipMinSpin->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_zClipMaxSpin->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_zClipMinSpin->setValue(static_cast<double>(dataZMin));
    m_zClipMaxSpin->setValue(static_cast<double>(dataZMax));
    m_zClipMinSpin->blockSignals(false);
    m_zClipMaxSpin->blockSignals(false);
}

// ============================================================================
// ColorRangeSettingsDialog —— 高程颜色范围
// ============================================================================

ColorRangeSettingsDialog::ColorRangeSettingsDialog(ViewportWidget* viewport, QWidget* parent)
    : QDialog(parent), m_viewport(viewport) {
    setWindowTitle(tr("Elevation Color Range"));
    setupUi();

    connect(m_viewport, &ViewportWidget::cloudDataReady,
            this, &ColorRangeSettingsDialog::onCloudDataReady);
}

void ColorRangeSettingsDialog::setupUi() {
    auto* layout = new QVBoxLayout(this);

    auto* group = new QGroupBox(tr("Elevation Color Range"));
    auto* form = new QFormLayout(group);

    m_autoColorRangeCb = new QCheckBox(tr("Auto Color Range"));
    m_autoColorRangeCb->setChecked(true);
    connect(m_autoColorRangeCb, &QCheckBox::toggled, this, [this](bool checked) {
        m_colorZMinSpin->setEnabled(!checked);
        m_colorZMaxSpin->setEnabled(!checked);
        m_viewport->setAutoColorRange(checked);
    });
    form->addRow(m_autoColorRangeCb);

    m_colorZMinSpin = new QDoubleSpinBox;
    m_colorZMinSpin->setRange(-10000.0, 10000.0);
    m_colorZMinSpin->setDecimals(2);
    m_colorZMinSpin->setEnabled(false);
    connect(m_colorZMinSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setColorZMin);
    form->addRow(tr("Min Z:"), m_colorZMinSpin);

    m_colorZMaxSpin = new QDoubleSpinBox;
    m_colorZMaxSpin->setRange(-10000.0, 10000.0);
    m_colorZMaxSpin->setDecimals(2);
    m_colorZMaxSpin->setEnabled(false);
    connect(m_colorZMaxSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setColorZMax);
    form->addRow(tr("Max Z:"), m_colorZMaxSpin);

    layout->addWidget(group);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

void ColorRangeSettingsDialog::onCloudDataReady(float dataZMin, float dataZMax) {
    m_colorZMinSpin->blockSignals(true);
    m_colorZMaxSpin->blockSignals(true);
    m_colorZMinSpin->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_colorZMaxSpin->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_colorZMinSpin->setValue(static_cast<double>(dataZMin));
    m_colorZMaxSpin->setValue(static_cast<double>(dataZMax));
    m_colorZMinSpin->blockSignals(false);
    m_colorZMaxSpin->blockSignals(false);
}
