#include "ui/RenderingPanel.h"
#include "ui/ViewportWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>

RenderingPanel::RenderingPanel(ViewportWidget* viewport, QWidget* parent)
    : QWidget(parent), m_viewport(viewport) {
    setupUi();

    // Rendering toggles → ViewportWidget
    connect(m_drawVerticesCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setDrawVertices);
    connect(m_drawEdgesCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setDrawEdges);
    connect(m_drawCloudsCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setDrawKeyframeClouds);
    connect(m_drawSE3EdgesCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setDrawSE3Edges);

    // Sphere radius slider
    connect(m_sphereRadiusSlider, &QSlider::valueChanged, this, [this](int val) {
        float r = val / 100.0f;
        m_sphereRadiusLabel->setText(QString::number(r, 'f', 2));
        m_viewport->setSphereRadius(r);
    });

    // Edge width slider
    connect(m_edgeWidthSlider, &QSlider::valueChanged, this, [this](int val) {
        m_edgeWidthLabel->setText(QString::number(val));
        m_viewport->setEdgeWidth(val);
    });

    // Point size slider
    connect(m_pointSizeSlider, &QSlider::valueChanged, this, [this](int val) {
        m_pointSizeLabel->setText(QString::number(val));
        m_viewport->setPointSize(val);
    });

    // Point opacity slider
    connect(m_pointOpacitySlider, &QSlider::valueChanged, this, [this](int val) {
        m_pointOpacityLabel->setText(QString::number(val) + "%");
        m_viewport->setPointOpacity(val);
    });
}

void RenderingPanel::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(8, 8, 8, 8);

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

    mainLayout->addWidget(renderGroup);
    mainLayout->addStretch();
}
