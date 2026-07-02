#include "ui/GraphInfoPanel.h"
#include "ui/ViewportWidget.h"
#include "backend/graph_manager.hpp"

#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>

GraphInfoPanel::GraphInfoPanel(GraphManager* manager, ViewportWidget* viewport,
                               QWidget* parent)
    : QWidget(parent), m_manager(manager), m_viewport(viewport) {
    setupUi();

    // --- Statistics ---
    connect(m_manager, &GraphManager::statsChanged,
            this, &GraphInfoPanel::onStatsChanged);
    connect(m_manager, &GraphManager::isLoadingChanged,
            this, &GraphInfoPanel::onLoadingStateChanged);

    // --- Rendering toggles → ViewportWidget ---
    connect(m_drawVerticesCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setDrawVertices);
    connect(m_drawEdgesCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setDrawEdges);
    connect(m_drawCloudsCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setDrawKeyframeClouds);
    connect(m_drawSE3EdgesCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setDrawSE3Edges);

    // --- Edge width slider ---
    connect(m_edgeWidthSlider, &QSlider::valueChanged, this, [this](int val) {
        m_edgeWidthLabel->setText(QString::number(val));
        m_viewport->setEdgeWidth(val);
    });

    // --- Sphere radius slider ---
    connect(m_sphereRadiusSlider, &QSlider::valueChanged, this, [this](int val) {
        float r = val / 100.0f;
        m_sphereRadiusLabel->setText(QString::number(r, 'f', 2));
        m_viewport->setSphereRadius(r);
    });

    // --- Point size slider ---
    connect(m_pointSizeSlider, &QSlider::valueChanged, this, [this](int val) {
        m_pointSizeLabel->setText(QString::number(val));
        m_viewport->setPointSize(val);
    });

    // --- Point opacity slider ---
    connect(m_pointOpacitySlider, &QSlider::valueChanged, this, [this](int val) {
        m_pointOpacityLabel->setText(QString::number(val) + "%");
        m_viewport->setPointOpacity(val);
    });

    // --- FPS from viewport ---
    connect(m_viewport, &ViewportWidget::fpsUpdated, this, [this](float fps) {
        m_fpsLabel->setText(QString::number(fps, 'f', 1));
    });

    // --- Cloud data range → update UI labels & spinbox ranges ---
    connect(m_viewport, &ViewportWidget::cloudDataReady,
            this, &GraphInfoPanel::onCloudDataReady);

    // --- Z-clip controls → ViewportWidget ---
    connect(m_zClipCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setZClipping);

    connect(m_zClipMinSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setZClipMin);
    connect(m_zClipMaxSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setZClipMax);

    // --- Color range controls ---
    connect(m_autoColorRangeCb, &QCheckBox::toggled, this, [this](bool checked) {
        m_colorZMinSpinBox->setEnabled(!checked);
        m_colorZMaxSpinBox->setEnabled(!checked);
        m_viewport->setAutoColorRange(checked);
    });

    connect(m_colorZMinSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setColorZMin);
    connect(m_colorZMaxSpinBox, QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            m_viewport, &ViewportWidget::setColorZMax);

    // Initial state
    onStatsChanged();
}

void GraphInfoPanel::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(8, 8, 8, 8);

    // --- Statistics group ---
    auto* statsGroup = new QGroupBox(tr("Graph Statistics"));
    auto* statsForm = new QFormLayout(statsGroup);

    m_vertexCountLabel = new QLabel("0");
    m_edgeCountLabel = new QLabel("0");
    m_keyframeCountLabel = new QLabel("0");
    m_fpsLabel = new QLabel("0.0");

    statsForm->addRow(tr("Vertices:"), m_vertexCountLabel);
    statsForm->addRow(tr("Edges:"), m_edgeCountLabel);
    statsForm->addRow(tr("Keyframes:"), m_keyframeCountLabel);
    statsForm->addRow(tr("FPS:"), m_fpsLabel);

    mainLayout->addWidget(statsGroup);

    // --- Rendering controls group ---
    auto* renderGroup = new QGroupBox(tr("Rendering"));
    auto* renderLayout = new QVBoxLayout(renderGroup);

    m_drawVerticesCb = new QCheckBox(tr("Show Vertices"));
    m_drawVerticesCb->setChecked(true);
    m_drawEdgesCb = new QCheckBox(tr("Show Edges"));
    m_drawEdgesCb->setChecked(true);
    m_drawCloudsCb = new QCheckBox(tr("Show Keyframe Clouds"));
    m_drawCloudsCb->setChecked(true);
    m_drawSE3EdgesCb = new QCheckBox(tr("Show SE3 Edges"));
    m_drawSE3EdgesCb->setChecked(true);

    renderLayout->addWidget(m_drawVerticesCb);
    renderLayout->addWidget(m_drawEdgesCb);
    renderLayout->addWidget(m_drawCloudsCb);
    renderLayout->addWidget(m_drawSE3EdgesCb);

    // Sphere radius
    renderLayout->addWidget(new QLabel(tr("Sphere Radius:")));
    m_sphereRadiusSlider = new QSlider(Qt::Horizontal);
    m_sphereRadiusSlider->setRange(1, 100);
    m_sphereRadiusSlider->setValue(50);
    m_sphereRadiusLabel = new QLabel("0.50");
    auto* sphereRow = new QHBoxLayout;
    sphereRow->addWidget(m_sphereRadiusSlider);
    sphereRow->addWidget(m_sphereRadiusLabel);
    renderLayout->addLayout(sphereRow);

    // Edge width
    renderLayout->addWidget(new QLabel(tr("Edge Width:")));
    m_edgeWidthSlider = new QSlider(Qt::Horizontal);
    m_edgeWidthSlider->setRange(1, 10);
    m_edgeWidthSlider->setValue(2);
    m_edgeWidthLabel = new QLabel("2");
    auto* edgeRow = new QHBoxLayout;
    edgeRow->addWidget(m_edgeWidthSlider);
    edgeRow->addWidget(m_edgeWidthLabel);
    renderLayout->addLayout(edgeRow);

    // Point size
    renderLayout->addWidget(new QLabel(tr("Point Size:")));
    m_pointSizeSlider = new QSlider(Qt::Horizontal);
    m_pointSizeSlider->setRange(1, 10);
    m_pointSizeSlider->setValue(3);
    m_pointSizeLabel = new QLabel("3");
    auto* sizeRow = new QHBoxLayout;
    sizeRow->addWidget(m_pointSizeSlider);
    sizeRow->addWidget(m_pointSizeLabel);
    renderLayout->addLayout(sizeRow);

    // Point opacity
    renderLayout->addWidget(new QLabel(tr("Point Opacity:")));
    m_pointOpacitySlider = new QSlider(Qt::Horizontal);
    m_pointOpacitySlider->setRange(10, 100);
    m_pointOpacitySlider->setValue(100);
    m_pointOpacityLabel = new QLabel("100%");
    auto* opacityRow = new QHBoxLayout;
    opacityRow->addWidget(m_pointOpacitySlider);
    opacityRow->addWidget(m_pointOpacityLabel);
    renderLayout->addLayout(opacityRow);

    // Reset camera button (inside Rendering group)
    auto* resetBtn = new QPushButton(tr("Reset Camera"));
    connect(resetBtn, &QPushButton::clicked, m_viewport, &ViewportWidget::resetCamera);
    renderLayout->addWidget(resetBtn);

    mainLayout->addWidget(renderGroup);

    // --- Z-Clipping group ---
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

    // --- Elevation Color Range group ---
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

    // --- Loading indicator ---
    m_loadingLabel = new QLabel;
    m_loadingLabel->setVisible(false);
    m_loadingLabel->setStyleSheet("color: #ffaa00; font-weight: bold;");
    mainLayout->addWidget(m_loadingLabel);

    mainLayout->addStretch();
}

void GraphInfoPanel::onStatsChanged() {
    m_vertexCountLabel->setText(QString::number(m_manager->vertexCount()));
    m_edgeCountLabel->setText(QString::number(m_manager->edgeCount()));
    m_keyframeCountLabel->setText(QString::number(m_manager->keyframeCount()));
}

void GraphInfoPanel::onLoadingStateChanged() {
    bool loading = m_manager->isLoading();
    m_loadingLabel->setVisible(loading);
    if (loading) {
        m_loadingLabel->setText(tr("Loading..."));
    }
}

void GraphInfoPanel::onCloudDataReady(float dataZMin, float dataZMax) {
    m_dataZRangeLabel->setText(
        tr("Data Z: %1 .. %2")
            .arg(dataZMin, 0, 'f', 2)
            .arg(dataZMax, 0, 'f', 2));

    // Block signals to avoid triggering updates during initialization
    m_zClipMinSpinBox->blockSignals(true);
    m_zClipMaxSpinBox->blockSignals(true);
    m_colorZMinSpinBox->blockSignals(true);
    m_colorZMaxSpinBox->blockSignals(true);

    m_zClipMinSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_zClipMaxSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_zClipMinSpinBox->setValue(static_cast<double>(dataZMin));
    m_zClipMaxSpinBox->setValue(static_cast<double>(dataZMax));

    m_colorZMinSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_colorZMaxSpinBox->setRange(dataZMin - 100.0, dataZMax + 100.0);
    m_colorZMinSpinBox->setValue(static_cast<double>(dataZMin));
    m_colorZMaxSpinBox->setValue(static_cast<double>(dataZMax));

    m_zClipMinSpinBox->blockSignals(false);
    m_zClipMaxSpinBox->blockSignals(false);
    m_colorZMinSpinBox->blockSignals(false);
    m_colorZMaxSpinBox->blockSignals(false);
}
