/**
 * @file RenderingPanel.cpp
 * @brief 渲染设置面板实现
 *
 * 实现渲染控制面板的 UI 搭建和信号连接。
 * 所有控件变化实时通过信号槽传递到 ViewportWidget 的对应接口。
 */

#include "ui/RenderingPanel.h"
#include "ui/ViewportWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>

/**
 * @brief 构造函数
 *
 * 创建 UI 控件并连接所有信号槽：
 * - 复选框 → ViewportWidget::setDrawXxx
 * - 滑块值改变 → 对应的数值标签更新 + ViewportWidget::setXxx
 *
 * @param viewport 关联的视口部件（接收设置命令）
 * @param parent   父级部件
 */
RenderingPanel::RenderingPanel(ViewportWidget* viewport, QWidget* parent)
    : QWidget(parent), m_viewport(viewport) {
    setupUi();

    // 渲染开关 → ViewportWidget
    connect(m_drawVerticesCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setDrawVertices);
    connect(m_drawEdgesCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setDrawEdges);
    connect(m_drawCloudsCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setDrawKeyframeClouds);
    connect(m_drawSE3EdgesCb, &QCheckBox::toggled,
            m_viewport, &ViewportWidget::setDrawSE3Edges);

    // 球体半径滑块（值范围 1-100，对应实际半径 0.01-1.00）
    connect(m_sphereRadiusSlider, &QSlider::valueChanged, this, [this](int val) {
        float r = val / 100.0f;
        m_sphereRadiusLabel->setText(QString::number(r, 'f', 2));
        m_viewport->setSphereRadius(r);
    });

    // 边线宽度滑块
    connect(m_edgeWidthSlider, &QSlider::valueChanged, this, [this](int val) {
        m_edgeWidthLabel->setText(QString::number(val));
        m_viewport->setEdgeWidth(val);
    });

    // 点大小滑块
    connect(m_pointSizeSlider, &QSlider::valueChanged, this, [this](int val) {
        m_pointSizeLabel->setText(QString::number(val));
        m_viewport->setPointSize(val);
    });

    // 点不透明度滑块
    connect(m_pointOpacitySlider, &QSlider::valueChanged, this, [this](int val) {
        m_pointOpacityLabel->setText(QString::number(val) + "%");
        m_viewport->setPointOpacity(val);
    });

    // 顶点标记不透明度滑块
    connect(m_vertexOpacitySlider, &QSlider::valueChanged, this, [this](int val) {
        m_vertexOpacityLabel->setText(QString::number(val) + "%");
        m_viewport->setVertexOpacity(val);
    });

    // 采样步长 SpinBox
    connect(m_sampleStrideSpin, QOverload<int>::of(&QSpinBox::valueChanged),
            this, [this](int val) {
        m_sampleStrideLabel->setText(QString::number(val));
        m_viewport->setSampleStride(val);
    });
}

/**
 * @brief 初始化 UI 布局
 *
 * 在 QGroupBox 中放置 4 个复选框和 4 个带标签的滑块。
 */
void RenderingPanel::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(8, 8, 8, 8);

    auto* renderGroup = new QGroupBox(tr("Rendering"));
    auto* renderLayout = new QVBoxLayout(renderGroup);

    // 显示开关复选框（默认全部启用）
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

    // 球体半径滑块（1-100，默认 50 → 0.50）
    renderLayout->addWidget(new QLabel(tr("Sphere Radius:")));
    m_sphereRadiusSlider = new QSlider(Qt::Horizontal);
    m_sphereRadiusSlider->setRange(1, 100);
    m_sphereRadiusSlider->setValue(50);
    m_sphereRadiusLabel = new QLabel("0.50");
    auto* sphereRow = new QHBoxLayout;
    sphereRow->addWidget(m_sphereRadiusSlider);
    sphereRow->addWidget(m_sphereRadiusLabel);
    renderLayout->addLayout(sphereRow);

    // 边线宽度滑块（1-10，默认 2）
    renderLayout->addWidget(new QLabel(tr("Edge Width:")));
    m_edgeWidthSlider = new QSlider(Qt::Horizontal);
    m_edgeWidthSlider->setRange(1, 10);
    m_edgeWidthSlider->setValue(2);
    m_edgeWidthLabel = new QLabel("2");
    auto* edgeRow = new QHBoxLayout;
    edgeRow->addWidget(m_edgeWidthSlider);
    edgeRow->addWidget(m_edgeWidthLabel);
    renderLayout->addLayout(edgeRow);

    // 点大小滑块（1-10，默认 3）
    renderLayout->addWidget(new QLabel(tr("Point Size:")));
    m_pointSizeSlider = new QSlider(Qt::Horizontal);
    m_pointSizeSlider->setRange(1, 10);
    m_pointSizeSlider->setValue(3);
    m_pointSizeLabel = new QLabel("3");
    auto* sizeRow = new QHBoxLayout;
    sizeRow->addWidget(m_pointSizeSlider);
    sizeRow->addWidget(m_pointSizeLabel);
    renderLayout->addLayout(sizeRow);

    // 点不透明度滑块（10-100%，默认 100%）
    renderLayout->addWidget(new QLabel(tr("Point Opacity:")));
    m_pointOpacitySlider = new QSlider(Qt::Horizontal);
    m_pointOpacitySlider->setRange(10, 100);
    m_pointOpacitySlider->setValue(100);
    m_pointOpacityLabel = new QLabel("100%");
    auto* opacityRow = new QHBoxLayout;
    opacityRow->addWidget(m_pointOpacitySlider);
    opacityRow->addWidget(m_pointOpacityLabel);
    renderLayout->addLayout(opacityRow);

    // 顶点标记不透明度滑块（10-100%，默认 100%）
    renderLayout->addWidget(new QLabel(tr("Vertex Opacity:")));
    m_vertexOpacitySlider = new QSlider(Qt::Horizontal);
    m_vertexOpacitySlider->setRange(10, 100);
    m_vertexOpacitySlider->setValue(100);
    m_vertexOpacityLabel = new QLabel("100%");
    auto* vertexOpacityRow = new QHBoxLayout;
    vertexOpacityRow->addWidget(m_vertexOpacitySlider);
    vertexOpacityRow->addWidget(m_vertexOpacityLabel);
    renderLayout->addLayout(vertexOpacityRow);

    // 采样步长 SpinBox（1-100，默认 1）
    renderLayout->addWidget(new QLabel(tr("Sample Stride:")));
    m_sampleStrideSpin = new QSpinBox;
    m_sampleStrideSpin->setRange(1, 100);
    m_sampleStrideSpin->setValue(1);
    m_sampleStrideSpin->setToolTip(tr("Render every Nth keyframe (1 = all)"));
    m_sampleStrideLabel = new QLabel("1");
    auto* strideRow = new QHBoxLayout;
    strideRow->addWidget(m_sampleStrideSpin);
    strideRow->addWidget(m_sampleStrideLabel);
    renderLayout->addLayout(strideRow);

    mainLayout->addWidget(renderGroup);
    mainLayout->addStretch();
}
