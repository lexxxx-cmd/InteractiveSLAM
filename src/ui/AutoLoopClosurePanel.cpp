#include "ui/AutoLoopClosurePanel.h"
#include "backend/graph_manager.hpp"
#include "data/hdl_graph_slam/automatic_loop_closure.hpp"
#include "data/hdl_graph_slam/registration_methods.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QMessageBox>

// ── Kernel name table (must match g2o RobustKernelFactory order) ─────────────
static const char* kKernelUiNames[] = {
    "NONE", "Huber", "Cauchy", "DCS", "Fair",
    "GemanMcClure", "PseudoHuber", "Saturated", "Tukey", "Welsch"
};
static constexpr int kNumUiKernels = sizeof(kKernelUiNames) / sizeof(kKernelUiNames[0]);

// ── Construction / Destruction ───────────────────────────────────────────────

AutoLoopClosurePanel::AutoLoopClosurePanel(GraphManager* manager, QWidget* parent)
    : QWidget(parent), m_manager(manager) {
    setupUi();

    // Poll timer at 10 Hz
    m_pollTimer = new QTimer(this);
    m_pollTimer->setInterval(100);
    connect(m_pollTimer, &QTimer::timeout, this, &AutoLoopClosurePanel::onPollStatus);

    // Start / Stop button
    connect(m_startStopBtn, &QPushButton::clicked, this, &AutoLoopClosurePanel::onStartStop);
}

AutoLoopClosurePanel::~AutoLoopClosurePanel() {
    stopDetection();
}

// ── Public ───────────────────────────────────────────────────────────────────

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

// ── UI Setup ─────────────────────────────────────────────────────────────────

void AutoLoopClosurePanel::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(8, 8, 8, 8);

    // ── Search group ─────────────────────────────────────────────
    auto* searchGroup = new QGroupBox(tr("Search"));
    auto* searchForm  = new QFormLayout(searchGroup);

    m_searchMethodCombo = new QComboBox;
    m_searchMethodCombo->addItem(tr("SEQUENTIAL"), 0);
    m_searchMethodCombo->addItem(tr("RANDOM"), 1);
    m_searchMethodCombo->setCurrentIndex(1);  // default: RANDOM
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

    mainLayout->addWidget(searchGroup);

    // ── Registration group ───────────────────────────────────────
    auto* regGroup = new QGroupBox(tr("Registration"));
    auto* regForm  = new QFormLayout(regGroup);

    m_methodCombo = new QComboBox;
    // Populate from a temporary RegistrationMethods instance
    {
        hdl_graph_slam::RegistrationMethods tmp;
        for (const char* name : tmp.method_names()) {
            m_methodCombo->addItem(QString::fromUtf8(name));
        }
    }
    m_methodCombo->setCurrentIndex(1);  // default: GICP
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

    mainLayout->addWidget(regGroup);

    // ── Robust kernel group ──────────────────────────────────────
    auto* kernelGroup = new QGroupBox(tr("Robust Kernel"));
    auto* kernelForm  = new QFormLayout(kernelGroup);

    m_kernelCombo = new QComboBox;
    for (int i = 0; i < kNumUiKernels; ++i) {
        m_kernelCombo->addItem(QString::fromUtf8(kKernelUiNames[i]), i);
    }
    m_kernelCombo->setCurrentIndex(0);  // default: NONE
    kernelForm->addRow(tr("Type:"), m_kernelCombo);

    m_kernelDeltaSpin = new QDoubleSpinBox;
    m_kernelDeltaSpin->setRange(1e-4, 10.0);
    m_kernelDeltaSpin->setDecimals(4);
    m_kernelDeltaSpin->setValue(0.01);
    m_kernelDeltaSpin->setSingleStep(0.001);
    kernelForm->addRow(tr("Delta:"), m_kernelDeltaSpin);

    mainLayout->addWidget(kernelGroup);

    // ── Fitness group ────────────────────────────────────────────
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

    mainLayout->addWidget(fitGroup);

    // ── Options ──────────────────────────────────────────────────
    m_optimizeCb = new QCheckBox(tr("Optimize after edge insert"));
    m_optimizeCb->setChecked(true);
    mainLayout->addWidget(m_optimizeCb);

    // ── Start / Stop button ──────────────────────────────────────
    m_startStopBtn = new QPushButton(tr("Start"));
    m_startStopBtn->setMinimumHeight(32);
    mainLayout->addWidget(m_startStopBtn);

    // ── Status display ───────────────────────────────────────────
    auto* statusGroup = new QGroupBox(tr("Status"));
    auto* statusLayout = new QVBoxLayout(statusGroup);

    m_statusLabel = new QLabel(tr("Status: Stopped"));
    m_statusLabel->setStyleSheet("font-weight: bold; color: #888;");
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

    mainLayout->addWidget(statusGroup);

    mainLayout->addStretch();
}

// ── Parameter Sync ───────────────────────────────────────────────────────────

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

// ── Slots ────────────────────────────────────────────────────────────────────

void AutoLoopClosurePanel::onStartStop() {
    auto* graph = m_manager->graph();
    if (!graph) {
        QMessageBox::information(this, tr("No Graph"),
                                 tr("Load a map before starting loop detection."));
        return;
    }

    // Lazy-create the data layer
    if (!m_autoLoop) {
        m_autoLoop = std::make_unique<hdl_graph_slam::AutomaticLoopClosure>(graph);
    }

    if (m_autoLoop->is_running()) {
        // ── Stop ─────────────────────────────────────────────────
        m_autoLoop->stop();
        m_pollTimer->stop();
        m_statusLabel->setText(tr("Status: Stopped"));
        m_statusLabel->setStyleSheet("font-weight: bold; color: #888;");
        m_startStopBtn->setText(tr("Start"));
        // Re-enable param editing
        m_searchMethodCombo->setEnabled(true);
    } else {
        // ── Start ────────────────────────────────────────────────
        // Re-create if the graph was closed and re-opened (pointer changed)
        if (m_autoLoop) {
            m_autoLoop = std::make_unique<hdl_graph_slam::AutomaticLoopClosure>(graph);
        }

        syncParamsToLoop();

        // Check for unavailable methods early
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
        m_statusLabel->setStyleSheet("font-weight: bold; color: #0a0;");
        m_startStopBtn->setText(tr("Stop"));
        m_lastKnownEdgesInserted = 0;

        // Disable param editing while running
        m_searchMethodCombo->setEnabled(false);
    }
}

void AutoLoopClosurePanel::onPollStatus() {
    if (!m_autoLoop) return;

    auto status = m_autoLoop->snapshot();

    if (!status.running && m_autoLoop->is_running()) {
        // Detection thread just started, status not yet updated
        return;
    }

    if (!status.running && !m_autoLoop->is_running()) {
        // Thread exited — ensure UI reflects stopped state
        m_pollTimer->stop();
        m_statusLabel->setText(tr("Status: Stopped"));
        m_statusLabel->setStyleSheet("font-weight: bold; color: #888;");
        m_startStopBtn->setText(tr("Start"));
        m_searchMethodCombo->setEnabled(true);
        return;
    }

    // ── Update status labels ────────────────────────────────────
    m_statusLabel->setText(tr("Status: Running"));
    m_statusLabel->setStyleSheet("font-weight: bold; color: #0a0;");

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

    if (status.last_begin_id >= 0) {
        m_lastMatchLabel->setText(
            tr("Last: %1 → %2  fitness=%3")
                .arg(status.last_begin_id)
                .arg(status.last_end_id)
                .arg(status.last_fitness_score, 0, 'f', 4));
    } else {
        m_lastMatchLabel->setText(tr("Last: —"));
    }

    // ── Detect new edge insertions ──────────────────────────────
    if (status.edges_inserted > m_lastKnownEdgesInserted) {
        m_lastKnownEdgesInserted = status.edges_inserted;
        emit loopEdgeInserted();
    }
}
