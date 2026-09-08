/**
 * @file AutoLoopClosurePanel.cpp
 * @brief 自动闭环检测控制面板实现（精简版）
 *
 * 界面简洁化后，面板只保留"开始/停止 + 实时状态"；
 * 全部参数收敛到 LoopClosureParams，由「高级设置 → 优化相关…」
 * 对话框读写（getParams/setParams），启动时同步到数据层。
 */

#include "ui/AutoLoopClosurePanel.h"
#include "backend/graph_manager.hpp"
#include "data/hdl_graph_slam/automatic_loop_closure.hpp"
#include "data/hdl_graph_slam/registration_methods.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QMessageBox>

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

LoopClosureParams AutoLoopClosurePanel::getParams() const {
    return m_params;
}

void AutoLoopClosurePanel::setParams(const LoopClosureParams& params) {
    m_params = params;
    // 未运行时立即同步到数据层；运行中由下次启动生效
    if (m_autoLoop && !m_autoLoop->is_running()) {
        syncParamsToLoop();
    }
}

void AutoLoopClosurePanel::setSampleStride(int stride) {
    if (stride < 1) stride = 1;
    m_sampleStride = stride;
    // 数据层存在即推送（运行中由检测线程在下一迭代感知并重建源帧池）
    if (m_autoLoop) {
        m_autoLoop->set_sample_stride(stride);
    }
}

// ---- UI 设置 ----

/**
 * @brief 创建 UI 控件（仅开始/停止 + 状态显示）
 */
void AutoLoopClosurePanel::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(8, 8, 8, 8);

    // 开始/停止按钮
    m_startStopBtn = new QPushButton(tr("Start"));
    m_startStopBtn->setObjectName("primaryButton");
    mainLayout->addWidget(m_startStopBtn);

    // 状态显示组
    auto* statusGroup = new QGroupBox(tr("Status"));
    auto* statusForm = new QFormLayout(statusGroup);

    m_statusLabel = new QLabel(tr("Status: Stopped"));
    m_statusLabel->setStyleSheet("color: #999999; font-weight: bold;");
    statusForm->addRow(tr("State:"), m_statusLabel);

    m_sourceLabel = new QLabel(tr("Current source: —"));
    statusForm->addRow(tr("Source:"), m_sourceLabel);

    m_candidatesLabel = new QLabel(tr("Candidates: 0"));
    statusForm->addRow(tr("Candidates:"), m_candidatesLabel);

    m_edgesInsertedLabel = new QLabel(tr("Edges inserted: 0"));
    statusForm->addRow(tr("Edges:"), m_edgesInsertedLabel);

    m_lastMatchLabel = new QLabel(tr("Last: —"));
    statusForm->addRow(tr("Last match:"), m_lastMatchLabel);

    mainLayout->addWidget(statusGroup);
    mainLayout->addStretch();
}

// ---- 参数同步 ----

/**
 * @brief 将 m_params 推送到数据层 AutomaticLoopClosure
 */
void AutoLoopClosurePanel::syncParamsToLoop() {
    if (!m_autoLoop) return;

    m_autoLoop->set_search_method(m_params.searchMethod);
    m_autoLoop->set_distance_thresh(m_params.distanceThresh);
    m_autoLoop->set_accum_distance_thresh(m_params.accumDistanceThresh);
    m_autoLoop->set_registration_method_index(m_params.registrationMethod);
    m_autoLoop->set_registration_max_iterations(m_params.maxIterations);
    m_autoLoop->set_registration_epsilon(static_cast<float>(m_params.epsilon));
    m_autoLoop->set_registration_resolution(static_cast<float>(m_params.resolution));
    m_autoLoop->set_robust_kernel_type(m_params.robustKernel);
    m_autoLoop->set_robust_kernel_delta(static_cast<float>(m_params.kernelDelta));
    m_autoLoop->set_fitness_score_thresh(static_cast<float>(m_params.fitnessThresh));
    m_autoLoop->set_fitness_score_max_range(static_cast<float>(m_params.fitnessMaxRange));
    m_autoLoop->set_optimize_after_insert(m_params.optimizeAfterInsert);
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
 * - 运行时：停止检测，停止轮询，恢复 UI 状态
 */
void AutoLoopClosurePanel::onStartStop() {
    auto* graph = m_manager->graph();
    if (!graph) {
        QMessageBox::information(this, tr("No Graph"),
                                 tr("Load a map before starting loop detection."));
        return;
    }

    // 延迟创建数据层（图指针变化时重建）
    if (!m_autoLoop || m_loopGraph != graph) {
        m_autoLoop = std::make_unique<hdl_graph_slam::AutomaticLoopClosure>(graph);
        m_loopGraph = graph;
        // 数据层重建后补同步缓存的采样步长
        m_autoLoop->set_sample_stride(m_sampleStride);
    }

    if (m_autoLoop->is_running()) {
        // ---- 停止检测 ----
        m_autoLoop->stop();
        m_pollTimer->stop();
        m_statusLabel->setText(tr("Status: Stopped"));
        updateStatusStyle(false);
        m_startStopBtn->setText(tr("Start"));
    } else {
        // ---- 启动检测 ----
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
