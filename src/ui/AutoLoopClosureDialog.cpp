/**
 * @file AutoLoopClosureDialog.cpp
 * @brief 「高级设置 → 优化相关 → 自动回环检测」对话框实现
 */
#include "ui/AutoLoopClosureDialog.h"
#include "ui/AutoLoopClosurePanel.h"
#include "backend/graph_manager.hpp"

#include <QComboBox>
#include <QSpinBox>
#include <QDoubleSpinBox>
#include <QCheckBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QVBoxLayout>
#include <QScrollArea>
#include <QDialogButtonBox>

// 配准方法名（与 RegistrationMethods 索引一致）
static const char* kRegMethodNames[] = {
    "ICP", "GICP", "NDT", "GICP_OMP", "NDT_OMP", "VGICP"
};

// 鲁棒核名称（与 g2o RobustKernelFactory 顺序一致）
static const char* kKernelNames[] = {
    "NONE", "Huber", "Cauchy", "DCS", "Fair",
    "GemanMcClure", "PseudoHuber", "Saturated", "Tukey", "Welsch"
};

AutoLoopClosureDialog::AutoLoopClosureDialog(GraphManager* manager, QWidget* parent)
    : QDialog(parent), m_manager(manager),
      m_autoLoopPanel(new AutoLoopClosurePanel(manager, this)) {
    setWindowTitle(tr("Auto Loop Closure"));
    setupUi();
    loadParams();

    // 转发内嵌自动回环面板的信号给 MainWindow
    connect(m_autoLoopPanel, &AutoLoopClosurePanel::loopEdgeInserted,
            this, &AutoLoopClosureDialog::loopEdgeInserted);
    connect(m_autoLoopPanel, &AutoLoopClosurePanel::loopDetectionStatus,
            this, &AutoLoopClosureDialog::loopDetectionStatus);
}

void AutoLoopClosureDialog::stopAutoLoop() {
    if (m_autoLoopPanel) m_autoLoopPanel->stopDetection();
}

void AutoLoopClosureDialog::setSampleStride(int stride) {
    if (m_autoLoopPanel) m_autoLoopPanel->setSampleStride(stride);
}

bool AutoLoopClosureDialog::optimizeAfterInsert() const {
    if (!m_autoLoopPanel) return true;  // 保守默认：视为会优化
    return m_autoLoopPanel->getParams().optimizeAfterInsert;
}

void AutoLoopClosureDialog::setupUi() {
    // 内容放入滚动区域，避免内容过多时撑满整个屏幕
    auto* mainLayout = new QVBoxLayout(this);

    auto* scrollArea = new QScrollArea;
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    auto* content = new QWidget;
    auto* layout = new QVBoxLayout(content);
    layout->setContentsMargins(8, 8, 8, 8);

    auto* loopGroup = new QGroupBox(tr("Automatic Loop Closure"));
    auto* loopLayout = new QVBoxLayout(loopGroup);
    loopLayout->addWidget(m_autoLoopPanel);  // 开始/停止 + 状态

    auto* paramForm = new QFormLayout;

    m_searchMethodCombo = new QComboBox;
    m_searchMethodCombo->addItem(tr("SEQUENTIAL"), 0);
    m_searchMethodCombo->addItem(tr("RANDOM"), 1);
    paramForm->addRow(tr("Search method:"), m_searchMethodCombo);

    m_distanceThreshSpin = new QDoubleSpinBox;
    m_distanceThreshSpin->setRange(0.5, 100.0);
    m_distanceThreshSpin->setDecimals(1);
    m_distanceThreshSpin->setSingleStep(0.5);
    paramForm->addRow(tr("Distance thresh:"), m_distanceThreshSpin);

    m_accumDistThreshSpin = new QDoubleSpinBox;
    m_accumDistThreshSpin->setRange(0.0, 200.0);
    m_accumDistThreshSpin->setDecimals(1);
    m_accumDistThreshSpin->setSingleStep(0.5);
    paramForm->addRow(tr("Accum. distance thresh:"), m_accumDistThreshSpin);

    m_methodCombo = new QComboBox;
    for (const char* name : kRegMethodNames) m_methodCombo->addItem(tr(name));
    paramForm->addRow(tr("Registration method:"), m_methodCombo);

    m_maxIterSpin = new QSpinBox;
    m_maxIterSpin->setRange(1, 1000);
    paramForm->addRow(tr("Max iterations:"), m_maxIterSpin);

    m_epsSpin = new QDoubleSpinBox;
    m_epsSpin->setDecimals(6);
    m_epsSpin->setRange(1e-8, 1.0);
    m_epsSpin->setSingleStep(1e-4);
    paramForm->addRow(tr("Convergence epsilon:"), m_epsSpin);

    m_resolutionSpin = new QDoubleSpinBox;
    m_resolutionSpin->setDecimals(1);
    m_resolutionSpin->setRange(0.1, 20.0);
    m_resolutionSpin->setSingleStep(0.1);
    paramForm->addRow(tr("NDT resolution:"), m_resolutionSpin);

    m_kernelCombo = new QComboBox;
    for (const char* name : kKernelNames) m_kernelCombo->addItem(tr(name));
    paramForm->addRow(tr("Robust kernel:"), m_kernelCombo);

    m_kernelDeltaSpin = new QDoubleSpinBox;
    m_kernelDeltaSpin->setDecimals(3);
    m_kernelDeltaSpin->setRange(0.001, 10.0);
    paramForm->addRow(tr("Kernel delta:"), m_kernelDeltaSpin);

    m_fitnessThreshSpin = new QDoubleSpinBox;
    m_fitnessThreshSpin->setDecimals(2);
    m_fitnessThreshSpin->setRange(0.0, 10.0);
    m_fitnessThreshSpin->setSingleStep(0.01);
    paramForm->addRow(tr("Fitness threshold:"), m_fitnessThreshSpin);

    m_fitnessMaxRangeSpin = new QDoubleSpinBox;
    m_fitnessMaxRangeSpin->setDecimals(2);
    m_fitnessMaxRangeSpin->setRange(0.1, 100.0);
    paramForm->addRow(tr("Fitness max range:"), m_fitnessMaxRangeSpin);

    m_optimizeCb = new QCheckBox(tr("Optimize after edge insert"));
    paramForm->addRow(m_optimizeCb);

    loopLayout->addLayout(paramForm);
    layout->addWidget(loopGroup);

    // 所有参数控件变化 → 即时写回面板
    const auto apply = [this]() { applyParams(); };
    connect(m_searchMethodCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, apply);
    connect(m_distanceThreshSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, apply);
    connect(m_accumDistThreshSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, apply);
    connect(m_methodCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, apply);
    connect(m_maxIterSpin, QOverload<int>::of(&QSpinBox::valueChanged), this, apply);
    connect(m_epsSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, apply);
    connect(m_resolutionSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, apply);
    connect(m_kernelCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, apply);
    connect(m_kernelDeltaSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, apply);
    connect(m_fitnessThreshSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, apply);
    connect(m_fitnessMaxRangeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, apply);
    connect(m_optimizeCb, &QCheckBox::toggled, this, apply);

    // 挂载滚动区域与底部关闭按钮
    scrollArea->setWidget(content);
    mainLayout->addWidget(scrollArea, 1);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Close);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    mainLayout->addWidget(buttons);
}

void AutoLoopClosureDialog::loadParams() {
    if (!m_autoLoopPanel) return;
    LoopClosureParams p = m_autoLoopPanel->getParams();

    m_searchMethodCombo->setCurrentIndex(m_searchMethodCombo->findData(p.searchMethod));
    m_distanceThreshSpin->setValue(p.distanceThresh);
    m_accumDistThreshSpin->setValue(p.accumDistanceThresh);
    m_methodCombo->setCurrentIndex(p.registrationMethod);
    m_maxIterSpin->setValue(p.maxIterations);
    m_epsSpin->setValue(p.epsilon);
    m_resolutionSpin->setValue(p.resolution);
    m_kernelCombo->setCurrentIndex(p.robustKernel);
    m_kernelDeltaSpin->setValue(p.kernelDelta);
    m_fitnessThreshSpin->setValue(p.fitnessThresh);
    m_fitnessMaxRangeSpin->setValue(p.fitnessMaxRange);
    m_optimizeCb->setChecked(p.optimizeAfterInsert);
}

void AutoLoopClosureDialog::applyParams() {
    if (!m_autoLoopPanel) return;
    LoopClosureParams p;
    p.searchMethod = m_searchMethodCombo->currentData().toInt();
    p.distanceThresh = m_distanceThreshSpin->value();
    p.accumDistanceThresh = m_accumDistThreshSpin->value();
    p.registrationMethod = m_methodCombo->currentIndex();
    p.maxIterations = m_maxIterSpin->value();
    p.epsilon = m_epsSpin->value();
    p.resolution = m_resolutionSpin->value();
    p.robustKernel = m_kernelCombo->currentIndex();
    p.kernelDelta = m_kernelDeltaSpin->value();
    p.fitnessThresh = m_fitnessThreshSpin->value();
    p.fitnessMaxRange = m_fitnessMaxRangeSpin->value();
    p.optimizeAfterInsert = m_optimizeCb->isChecked();
    m_autoLoopPanel->setParams(p);
}
