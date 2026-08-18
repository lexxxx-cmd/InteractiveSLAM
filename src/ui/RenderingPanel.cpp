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

    // 采样步长 SpinBox
    connect(m_sampleStrideSpin, QOverload<int>::of(&QSpinBox::valueChanged),
            this, [this](int val) {
        m_sampleStrideLabel->setText(QString::number(val));
        m_viewport->setSampleStride(val);
    });

    // 点预算下拉框：点数上限档位（-1 = 全量），超过上限自动体素降采样
    connect(m_pointBudgetCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int) {
        m_viewport->setPointBudget(m_pointBudgetCombo->currentData().toInt());
        updateLodAvailability();
    });

    // LOD 多级渲染开关（仅全量模式启用；预算模式自动取消勾选并灰显）
    connect(m_lodCb, &QCheckBox::toggled, this, [this](bool checked) {
        m_viewport->setLodEnabled(checked);
        if (!checked) {
            m_lodLevelLabel->setText(tr("LOD: off"));
        } else {
            m_lodLevelLabel->setText(tr("LOD: building..."));
        }
        updateLodAvailability();
    });

    // LOD 切换模式：Auto（距离驱动）/ Manual（固定层级）
    connect(m_lodModeCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int index) {
        m_viewport->setLodMode(index == 1);
        m_lodLevelCombo->setEnabled(index == 1 && m_lodCb->isChecked());
    });

    // LOD 手动层级选择（仅 Manual 模式生效）
    connect(m_lodLevelCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int index) {
        m_viewport->setLodManualLevel(index);
    });

    // LOD 层级状态：构建完成（多级就绪）与距离切换时更新
    connect(m_viewport, &ViewportWidget::lodLevelChanged,
            this, &RenderingPanel::onLodLevelChanged);

    // 渲染统计显示：已渲染点数 / 全量点数
    connect(m_viewport, &ViewportWidget::pointCloudStatsChanged,
            this, [this](qint64 rendered, qint64 total) {
        m_pointBudgetCountLabel->setText(
            tr("Rendered: %1 / %2 points").arg(rendered).arg(total));
    });

    // 初始化 LOD 可用性（默认预算 500 万 → LOD 禁用）
    updateLodAvailability();
}

/**
 * @brief 根据点预算档位/LOD 开关联动控件可用性
 *
 * LOD 仅在"全量"模式下有意义（预算模式已固定点数上限）。
 * 预算模式（>0）下禁用并自动取消勾选 LOD；
 * LOD 未启用时禁用模式/层级下拉，Manual 模式才启用层级下拉。
 */
void RenderingPanel::updateLodAvailability() {
    bool fullMode = m_pointBudgetCombo->currentData().toInt() <= 0;
    m_lodCb->setEnabled(fullMode);
    if (!fullMode) {
        m_lodCb->setChecked(false);
    }
    bool lodActive = fullMode && m_lodCb->isChecked();
    m_lodModeCombo->setEnabled(lodActive);
    m_lodLevelCombo->setEnabled(lodActive &&
                                m_lodModeCombo->currentIndex() == 1);
}

/**
 * @brief LOD 状态更新：刷新层级下拉（构建完成后填充 0..N-1）与状态标签
 */
void RenderingPanel::onLodLevelChanged(int level, int levelCount) {
    // 重建层级下拉（保持当前选择，clamp 到有效范围）
    int prev = m_lodLevelCombo->currentIndex();
    m_lodLevelCombo->blockSignals(true);
    m_lodLevelCombo->clear();
    for (int i = 0; i < levelCount; ++i) {
        m_lodLevelCombo->addItem(
            i == 0 ? tr("Level 0 (full)") : tr("Level %1").arg(i));
    }
    m_lodLevelCombo->setCurrentIndex(qBound(0, prev, levelCount - 1));
    m_lodLevelCombo->blockSignals(false);

    updateLodLabel(level, levelCount);
}

/**
 * @brief 更新 LOD 状态标签（含切换模式）
 */
void RenderingPanel::updateLodLabel(int level, int levelCount) {
    if (levelCount <= 1 || !m_lodCb->isChecked()) {
        m_lodLevelLabel->setText(tr("LOD: off"));
        return;
    }
    bool manual = m_lodModeCombo->currentIndex() == 1;
    m_lodLevelLabel->setText(
        tr("LOD: L%1/%2 (%3)")
            .arg(level)
            .arg(levelCount)
            .arg(manual ? tr("manual") : tr("auto")));
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

    // 点预算下拉框（点数上限档位，默认 500 万）
    // 全量点数超过预算时，点云自动体素降采样，使渲染点数收敛到预算以内
    renderLayout->addWidget(new QLabel(tr("Point Budget:")));
    m_pointBudgetCombo = new QComboBox;
    m_pointBudgetCombo->addItem(tr("Full (all points)"), -1);
    m_pointBudgetCombo->addItem(tr("10,000,000"), 10000000);
    m_pointBudgetCombo->addItem(tr("5,000,000"), 5000000);
    m_pointBudgetCombo->addItem(tr("2,000,000"), 2000000);
    m_pointBudgetCombo->addItem(tr("1,000,000"), 1000000);
    m_pointBudgetCombo->addItem(tr("500,000"), 500000);
    m_pointBudgetCombo->setToolTip(
        tr("Cap rendered points. Larger maps are voxel-downsampled to fit the budget."));
    int budgetIdx = m_pointBudgetCombo->findData(5000000);
    m_pointBudgetCombo->setCurrentIndex(budgetIdx >= 0 ? budgetIdx : 0);
    m_pointBudgetCountLabel = new QLabel(tr("Rendered: -"));
    renderLayout->addWidget(m_pointBudgetCombo);
    renderLayout->addWidget(m_pointBudgetCountLabel);

    // LOD 多级渲染（仅全量模式可用，预算模式自动灰显）
    m_lodCb = new QCheckBox(tr("LOD (full mode)"));
    m_lodCb->setToolTip(
        tr("Distance-based level switching: full detail when close, fewer points when far. Only available in Full mode."));
    renderLayout->addWidget(m_lodCb);
    m_lodLevelLabel = new QLabel(tr("LOD: off"));
    renderLayout->addWidget(m_lodLevelLabel);

    // LOD 切换模式与手动层级
    auto* lodModeRow = new QHBoxLayout;
    lodModeRow->addWidget(new QLabel(tr("LOD Mode:")));
    m_lodModeCombo = new QComboBox;
    m_lodModeCombo->addItem(tr("Auto (distance)"));
    m_lodModeCombo->addItem(tr("Manual"));
    m_lodModeCombo->setToolTip(
        tr("Auto: switch level by camera distance. Manual: fix a level of your choice."));
    lodModeRow->addWidget(m_lodModeCombo);
    renderLayout->addLayout(lodModeRow);

    auto* lodLevelRow = new QHBoxLayout;
    lodLevelRow->addWidget(new QLabel(tr("LOD Level:")));
    m_lodLevelCombo = new QComboBox;
    m_lodLevelCombo->addItem(tr("Level 0 (full)"));
    m_lodLevelCombo->setToolTip(tr("Select the LOD level in Manual mode."));
    lodLevelRow->addWidget(m_lodLevelCombo);
    renderLayout->addLayout(lodLevelRow);

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
