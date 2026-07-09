/**
 * @file AutoLoopClosurePanel.cpp
 * @brief 自动闭环检测控制面板实现
 *
 * 实现自动闭环检测的 UI 控制面板，包括：
 * - 搜索参数（方法、距离阈值）的 UI 控件
 * - 点云配准参数（方法、迭代次数、精度、NDT分辨率）
 * - 鲁棒核函数选择
 * - 适应度分数阈值设置
 * - 启动/停止控制
 * - 定时轮询状态快照（10Hz）
 * - 信号发射：闭环边插入通知、顶点高亮通知
 */

#include "ui/AutoLoopClosurePanel.h"
#include "backend/graph_manager.hpp"
#include "data/hdl_graph_slam/automatic_loop_closure.hpp"
#include "data/hdl_graph_slam/registration_methods.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QScrollArea>
#include <QMessageBox>

// ---- 核函数名称表（必须与 g2o RobustKernelFactory 顺序一致） ----
static const char* kKernelUiNames[] = {
    "NONE", "Huber", "Cauchy", "DCS", "Fair",
    "GemanMcClure", "PseudoHuber", "Saturated", "Tukey", "Welsch"
};
static constexpr int kNumUiKernels = sizeof(kKernelUiNames) / sizeof(kKernelUiNames[0]);

// ---- 构造 / 析构 ----

AutoLoopClosurePanel::AutoLoopClosurePanel(GraphManager* manager, QWidget* parent)
    : QWidget(parent), m_manager(manager) {
    setupUi();

    // 轮询定时器 10 Hz
    m_pollTimer = new QTimer(this);
    m_pollTimer->setInterval(100);
    connect(m_pollTimer, &QTimer::timeout, this, &AutoLoopClosurePanel::onPollStatus);

    // 开始/停止按钮
    connect(m_startStopBtn, &QPushButton::clicked, this, &AutoLoopClosurePanel::onStartStop);
}

AutoLoopClosurePanel::~AutoLoopClosurePanel() {
    stopDetection();
}

// ---- 公开接口 ----

/**
 * @brief 停止自动检测
 *
 * 如果检测正在运行，停止 AutomaticLoopClosure 线程和轮询定时器，
 * 并将 UI 恢复为"已停止"状态。
 */
void AutoLoopClosurePanel::stopDetection() {
    if (m_autoLoop && m_autoLoop->is_running()) {
        m_autoLoop->stop();
    }
    m_pollTimer->stop();

    if (m_statusLabel) {
        m_statusLabel->setText(tr("Status: Stopped"));
        m_startStopBtn->setText(tr("Start"));
    }
}

// ---- UI 设置 ----

/**
 * @brief 创建 UI 控件
 *
 * 按功能分组创建控件：
 * - 搜索参数组（方法、距离阈值）
 * - 配准参数组（方法、迭代次数、精度、分辨率）
 * - 鲁棒核函数组（类型、delta）
 * - 适应度分数组（阈值、最大范围）
 * - 选项（插入后优化）
 * - 开始/停止按钮
 * - 状态显示组
 */
void AutoLoopClosurePanel::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(0, 0, 0, 0);

    // 可滚动区域 — 面板内容过长时自动出现滚动条
    auto* scrollArea = new QScrollArea(this);
    scrollArea->setWidgetResizable(true);
    scrollArea->setFrameShape(QFrame::NoFrame);
    scrollArea->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scrollArea->setObjectName("AutoLoopScrollArea");

    auto* scrollContent = new QWidget;
    scrollContent->setObjectName("AutoLoopScrollContent");
    auto* contentLayout = new QVBoxLayout(scrollContent);
    contentLayout->setContentsMargins(8, 8, 8, 8);

    // ---- 搜索参数组 ----
    auto* searchGroup = new QGroupBox(tr("Search"));
    auto* searchForm  = new QFormLayout(searchGroup);

    m_searchMethodCombo = new QComboBox;
    m_searchMethodCombo->addItem(tr("SEQUENTIAL"), 0);
    m_searchMethodCombo->addItem(tr("RANDOM"), 1);
    m_searchMethodCombo->setCurrentIndex(1);  // 默认：RANDOM
    searchForm->addRow(tr("Method:"), m_searchMethodCombo);

    m_distanceThreshSpin = new QDoubleSpinBox;
    m_distanceThreshSpin->setRange(0.5, 100.0);
    m_distanceThreshSpin->setValue(10.0);
    m_distanceThreshSpin->setDecimals(1);
    m_distanceThreshSpin->setSingleStep(0.5);
    searchForm->addRow(tr("Distance thresh:"), m_distanceThreshSpin);

    m_accumDistThreshSpin = new QDoubleSpinBox;
    m_accumDistThreshSpin->setRange(0.5, 100.0);
    m_accumDistThreshSpin->setValue(15.0);
    m_accumDistThreshSpin->setDecimals(1);
    m_accumDistThreshSpin->setSingleStep(0.5);
    searchForm->addRow(tr("Accum. dist thresh:"), m_accumDistThreshSpin);

    contentLayout->addWidget(searchGroup);

    // ---- 配准参数组 ----
    auto* regGroup = new QGroupBox(tr("Registration"));
    auto* regForm  = new QFormLayout(regGroup);

    m_methodCombo = new QComboBox;
    // 从临时 RegistrationMethods 实例填充配准方法列表
    {
        hdl_graph_slam::RegistrationMethods tmp;
        for (const char* name : tmp.method_names()) {
            m_methodCombo->addItem(QString::fromUtf8(name));
        }
    }
    m_methodCombo->setCurrentIndex(1);  // 默认：GICP
    regForm->addRow(tr("Method:"), m_methodCombo);

    m_maxIterSpin = new QSpinBox;
    m_maxIterSpin->setRange(1, 512);
    m_maxIterSpin->setValue(64);
    regForm->addRow(tr("Max iterations:"), m_maxIterSpin);

    m_epsSpin = new QDoubleSpinBox;
    m_epsSpin->setRange(1e-6, 1e-1);
    m_epsSpin->setDecimals(6);
    m_epsSpin->setValue(1e-4);
    m_epsSpin->setSingleStep(1e-5);
    regForm->addRow(tr("Transformation epsilon:"), m_epsSpin);

    m_resolutionSpin = new QDoubleSpinBox;
    m_resolutionSpin->setRange(0.1, 20.0);
    m_resolutionSpin->setDecimals(1);
    m_resolutionSpin->setValue(2.0);
    m_resolutionSpin->setSingleStep(0.5);
    regForm->addRow(tr("Resolution (NDT):"), m_resolutionSpin);

    contentLayout->addWidget(regGroup);

    // ---- 鲁棒核函数组 ----
    auto* kernelGroup = new QGroupBox(tr("Robust Kernel"));
    auto* kernelForm  = new QFormLayout(kernelGroup);

    m_kernelCombo = new QComboBox;
    for (int i = 0; i < kNumUiKernels; ++i) {
        m_kernelCombo->addItem(QString::fromUtf8(kKernelUiNames[i]), i);
    }
    m_kernelCombo->setCurrentIndex(0);  // 默认：NONE
    kernelForm->addRow(tr("Type:"), m_kernelCombo);

    m_kernelDeltaSpin = new QDoubleSpinBox;
    m_kernelDeltaSpin->setRange(1e-4, 10.0);
    m_kernelDeltaSpin->setDecimals(4);
    m_kernelDeltaSpin->setValue(0.01);
    m_kernelDeltaSpin->setSingleStep(0.001);
    kernelForm->addRow(tr("Delta:"), m_kernelDeltaSpin);

    contentLayout->addWidget(kernelGroup);

    // ---- 适应度分数组 ----
    auto* fitGroup = new QGroupBox(tr("Fitness Score"));
    auto* fitForm  = new QFormLayout(fitGroup);

    m_fitnessThreshSpin = new QDoubleSpinBox;
    m_fitnessThreshSpin->setRange(0.01, 10.0);
    m_fitnessThreshSpin->setDecimals(2);
    m_fitnessThreshSpin->setValue(0.30);
    m_fitnessThreshSpin->setSingleStep(0.01);
    fitForm->addRow(tr("Threshold:"), m_fitnessThreshSpin);

    m_fitnessMaxRangeSpin = new QDoubleSpinBox;
    m_fitnessMaxRangeSpin->setRange(0.1, 20.0);
    m_fitnessMaxRangeSpin->setDecimals(2);
    m_fitnessMaxRangeSpin->setValue(2.00);
    m_fitnessMaxRangeSpin->setSingleStep(0.10);
    fitForm->addRow(tr("Max range:"), m_fitnessMaxRangeSpin);

    contentLayout->addWidget(fitGroup);

    // ---- 选项 ----
    m_optimizeCb = new QCheckBox(tr("Optimize after edge insert"));
    m_optimizeCb->setChecked(true);
    contentLayout->addWidget(m_optimizeCb);

    // ---- 开始/停止按钮 ----
    m_startStopBtn = new QPushButton(tr("Start"));
    m_startStopBtn->setMinimumHeight(32);
    m_startStopBtn->setObjectName("primaryButton");
    contentLayout->addWidget(m_startStopBtn);

    // ---- 状态显示组 ----
    auto* statusGroup = new QGroupBox(tr("Status"));
    auto* statusLayout = new QVBoxLayout(statusGroup);

    m_statusLabel = new QLabel(tr("Status: Stopped"));
    m_statusLabel->setObjectName("LoopStatusLabel");
    statusLayout->addWidget(m_statusLabel);

    m_sourceLabel = new QLabel(tr("Current source: —"));
    statusLayout->addWidget(m_sourceLabel);

    m_candidatesLabel = new QLabel(tr("Candidates: —"));
    statusLayout->addWidget(m_candidatesLabel);

    m_edgesInsertedLabel = new QLabel(tr("Edges inserted: 0"));
    statusLayout->addWidget(m_edgesInsertedLabel);

    m_lastMatchLabel = new QLabel(tr("Last match: —"));
    m_lastMatchLabel->setWordWrap(true);
    statusLayout->addWidget(m_lastMatchLabel);

    contentLayout->addWidget(statusGroup);

    contentLayout->addStretch();

    // 将可滚动内容放入 ScrollArea，ScrollArea 放入主布局
    scrollArea->setWidget(scrollContent);
    mainLayout->addWidget(scrollArea);
}

// ---- 参数同步 ----

/**
 * @brief 将 UI 控件的值推送到数据层 AutomaticLoopClosure
 */
void AutoLoopClosurePanel::syncParamsToLoop() {
    if (!m_autoLoop) return;

    m_autoLoop->set_search_method(m_searchMethodCombo->currentData().toInt());
    m_autoLoop->set_distance_thresh(m_distanceThreshSpin->value());
    m_autoLoop->set_accum_distance_thresh(m_accumDistThreshSpin->value());
    m_autoLoop->set_registration_method_index(m_methodCombo->currentIndex());
    m_autoLoop->set_registration_max_iterations(m_maxIterSpin->value());
    m_autoLoop->set_registration_epsilon(static_cast<float>(m_epsSpin->value()));
    m_autoLoop->set_registration_resolution(static_cast<float>(m_resolutionSpin->value()));
    m_autoLoop->set_robust_kernel_type(m_kernelCombo->currentData().toInt());
    m_autoLoop->set_robust_kernel_delta(static_cast<float>(m_kernelDeltaSpin->value()));
    m_autoLoop->set_fitness_score_thresh(static_cast<float>(m_fitnessThreshSpin->value()));
    m_autoLoop->set_fitness_score_max_range(static_cast<float>(m_fitnessMaxRangeSpin->value()));
    m_autoLoop->set_optimize_after_insert(m_optimizeCb->isChecked());
}

// ---- 私有辅助 ----

void AutoLoopClosurePanel::updateStatusStyle(bool running) {
    if (running) {
        m_statusLabel->setStyleSheet("color: #22c55e; font-weight: bold;");
    } else {
        m_statusLabel->setStyleSheet("color: #999999; font-weight: bold;");
    }
}

// ---- 槽函数 ----

/**
 * @brief 开始/停止按钮处理
 *
 * - 未运行时：延迟创建数据层，同步参数，检查配准方法可用性，启动检测
 * - 运行时：停止检测，停止轮询，恢复 UI 可编辑状态
 */
void AutoLoopClosurePanel::onStartStop() {
    auto* graph = m_manager->graph();
    if (!graph) {
        QMessageBox::information(this, tr("No Graph"),
                                 tr("Load a map before starting loop detection."));
        return;
    }

    // 延迟创建数据层
    if (!m_autoLoop) {
        m_autoLoop = std::make_unique<hdl_graph_slam::AutomaticLoopClosure>(graph);
    }

    if (m_autoLoop->is_running()) {
        // ---- 停止检测 ----
        m_autoLoop->stop();
        m_pollTimer->stop();
        m_statusLabel->setText(tr("Status: Stopped"));
        updateStatusStyle(false);
        m_startStopBtn->setText(tr("Start"));
        // 恢复参数编辑
        m_searchMethodCombo->setEnabled(true);
    } else {
        // ---- 启动检测 ----
        // 如果图谱已关闭并重新打开（指针改变），重新创建数据层
        if (m_autoLoop) {
            m_autoLoop = std::make_unique<hdl_graph_slam::AutomaticLoopClosure>(graph);
        }

        syncParamsToLoop();

        // 提前检查配准方法是否可用
        try {
            m_autoLoop->reg_methods().method();
        } catch (const std::runtime_error& e) {
            QMessageBox::warning(this, tr("Method Unavailable"),
                                 QString::fromUtf8(e.what()));
            return;
        }

        m_autoLoop->start();
        m_pollTimer->start();
        m_statusLabel->setText(tr("Status: Running"));
        updateStatusStyle(true);
        m_startStopBtn->setText(tr("Stop"));
        m_lastKnownEdgesInserted = 0;

        // 运行时禁用参数编辑
        m_searchMethodCombo->setEnabled(false);
    }
}

/**
 * @brief 定时轮询状态快照（10Hz）
 *
 * 从数据层获取线程安全的状态快照，更新 UI 标签，
 * 发射 loopDetectionStatus 信号和 loopEdgeInserted 信号。
 */
void AutoLoopClosurePanel::onPollStatus() {
    if (!m_autoLoop) return;

    auto status = m_autoLoop->snapshot();

    // 检测线程刚启动，状态尚未更新时跳过
    if (!status.running && m_autoLoop->is_running()) {
        return;
    }

    // 线程已退出 — 确保 UI 反映停止状态
    if (!status.running && !m_autoLoop->is_running()) {
        m_pollTimer->stop();
        m_statusLabel->setText(tr("Status: Stopped"));
        updateStatusStyle(false);
        m_startStopBtn->setText(tr("Start"));
        m_searchMethodCombo->setEnabled(true);
        return;
    }

    // ---- 更新状态标签 ----
    m_statusLabel->setText(tr("Status: Running"));
    updateStatusStyle(true);

    if (status.current_source_id >= 0) {
        m_sourceLabel->setText(
            tr("Current source: %1").arg(status.current_source_id));
    } else {
        m_sourceLabel->setText(tr("Current source: —"));
    }

    m_candidatesLabel->setText(
        tr("Candidates: %1").arg(status.candidate_ids.size()));

    m_edgesInsertedLabel->setText(
        tr("Edges inserted: %1").arg(status.edges_inserted));

    // 最后匹配信息（含适应度分数）
    if (status.last_begin_id >= 0) {
        QString fitnessStr;
        if (status.last_fitness_score >= 1e100) {
            fitnessStr = QString::fromUtf8("∞ (no overlap)");
        } else {
            fitnessStr = QString::number(status.last_fitness_score, 'f', 4);
        }
        m_lastMatchLabel->setText(
            tr("Last: %1 → %2  fitness=%3")
                .arg(status.last_begin_id)
                .arg(status.last_end_id)
                .arg(fitnessStr));
    } else {
        m_lastMatchLabel->setText(tr("Last: —"));
    }

    // ---- 球体颜色高亮（蓝色=源顶点，绿色=候选顶点） ----
    QVector<long> qCandidates;
    qCandidates.reserve(static_cast<int>(status.candidate_ids.size()));
    for (long id : status.candidate_ids) qCandidates.append(id);
    emit loopDetectionStatus(status.current_source_id, qCandidates);

    // ---- 检测新边插入 ----
    if (status.edges_inserted > m_lastKnownEdgesInserted) {
        m_lastKnownEdgesInserted = status.edges_inserted;
        emit loopEdgeInserted();
    }
}
