#include "ui/LoopClosureDialog.h"
#include "ui/MiniViewportWidget.h"
#include "backend/graph_manager.hpp"
#include "data/hdl_graph_slam/registration_methods.hpp"
#include "data/hdl_graph_slam/information_matrix_calculator.hpp"
#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "data/hdl_graph_slam/interactive_keyframe.hpp"
#include "data/hdl_graph_slam/keyframe.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGroupBox>
#include <QFormLayout>
#include <QComboBox>
#include <QSpinBox>
#include <QDialogButtonBox>
#include <QMessageBox>
#include <QTimer>
#include <QtConcurrent/QtConcurrent>

#include <algorithm>

#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/registration/registration.h>

#include <g2o/types/slam3d/types_slam3d.h>

// ---------------------------------------------------------------------------
// mergeAdjacentClouds — free function
// ---------------------------------------------------------------------------

pcl::PointCloud<pcl::PointXYZI>::Ptr mergeAdjacentClouds(
    const hdl_graph_slam::InteractiveGraph* graph, long centerId,
    int windowHalfSize) {

    using PointT = pcl::PointXYZI;
    auto merged = pcl::make_shared<pcl::PointCloud<PointT>>();

    // Find center keyframe (required)
    auto itCenter = graph->keyframes.find(centerId);
    if (itCenter == graph->keyframes.end()) return merged;
    auto& centerKf = itCenter->second;
    if (!centerKf->cloud || centerKf->cloud->empty()) return merged;
    Eigen::Isometry3d centerPose = centerKf->estimate();

    // Merge centerId-windowHalfSize ... centerId+windowHalfSize
    for (long id = centerId - windowHalfSize; id <= centerId + windowHalfSize; ++id) {
        auto it = graph->keyframes.find(id);
        if (it == graph->keyframes.end()) continue;
        auto& kf = it->second;
        if (!kf->cloud || kf->cloud->empty()) continue;

        Eigen::Isometry3d T_rel = centerPose.inverse() * kf->estimate();

        pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*kf->cloud, *transformed, T_rel.matrix());
        *merged += *transformed;
    }

    return merged;
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

LoopClosureDialog::LoopClosureDialog(long beginVertexId, long endVertexId,
                                     GraphManager* manager,
                                     CloudPtr beginCloud, CloudPtr endCloud,
                                     QWidget* parent)
    : QDialog(parent),
      m_manager(manager),
      m_beginVertexId(beginVertexId),
      m_endVertexId(endVertexId) {

    setWindowTitle(tr("Loop Closure"));
    setModal(true);
    setMinimumSize(560, 700);

    m_graph = m_manager->graph();
    if (!m_graph) {
        QMessageBox::warning(parent, tr("Error"), tr("No graph loaded"));
        return;
    }

    // Use pre-merged clouds from caller
    m_beginCloud = beginCloud;
    m_endCloud   = endCloud;
    if (!m_beginCloud || !m_endCloud || m_beginCloud->empty() || m_endCloud->empty()) {
        QMessageBox::warning(parent, tr("Error"),
                             tr("One or both keyframes have no point cloud data"));
        return;
    }

    // Look up keyframes for poses
    auto itBegin = m_graph->keyframes.find(m_beginVertexId);
    auto itEnd   = m_graph->keyframes.find(m_endVertexId);
    if (itBegin == m_graph->keyframes.end() || itEnd == m_graph->keyframes.end()) {
        QMessageBox::warning(parent, tr("Error"), tr("Keyframe not found"));
        return;
    }

    m_beginPose    = itBegin->second->estimate();
    m_endPoseInit  = itEnd->second->estimate();
    m_endPose      = m_endPoseInit;

    m_regMethods = std::make_unique<hdl_graph_slam::RegistrationMethods>();

    // Slider prev values start at 0
    for (int i = 0; i < 6; ++i) m_sliderPrevValues[i] = 0.0;

    setupUi();

    // Initial preview
    m_miniViewport->setClouds(m_beginCloud, m_beginPose, m_endCloud, m_endPose);
    updateFitnessScore();
}

LoopClosureDialog::~LoopClosureDialog() {
    // If watchers are running, wait for them (they hold shared_ptr refs, safe)
    if (m_fpfhWatcher && m_fpfhWatcher->isRunning()) {
        m_fpfhWatcher->waitForFinished();
    }
    if (m_scanMatchWatcher && m_scanMatchWatcher->isRunning()) {
        m_scanMatchWatcher->waitForFinished();
    }
}

// ---------------------------------------------------------------------------
// UI Setup
// ---------------------------------------------------------------------------

void LoopClosureDialog::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);

    // --- Mini viewport (512×512) ---
    m_miniViewport = new MiniViewportWidget(this);
    mainLayout->addWidget(m_miniViewport, 0, Qt::AlignHCenter);

    // --- Fitness score ---
    m_fitnessLabel = new QLabel(tr("fitness_score: —"));
    m_fitnessLabel->setStyleSheet("font-family: monospace; font-size: 13px;");
    mainLayout->addWidget(m_fitnessLabel);

    // --- Manual adjustment sliders ---
    auto* sliderGroup = new QGroupBox(tr("Manual Adjustment (local frame)"));
    auto* sliderLayout = new QVBoxLayout(sliderGroup);

    // Step-size gear selector
    auto* stepRow = new QHBoxLayout;
    stepRow->addWidget(new QLabel(tr("Step:")));
    m_stepCombo = new QComboBox;
    m_stepCombo->addItem(tr("Fine     — 0.01m /  0.6°"),  0);
    m_stepCombo->addItem(tr("Medium   — 0.10m /  2.9°"),  1);
    m_stepCombo->addItem(tr("Coarse   — 0.50m / 11.5°"),  2);
    m_stepCombo->addItem(tr("Large    — 1.00m / 45.0°"),  3);
    m_stepCombo->setCurrentIndex(1);  // default: Medium
    stepRow->addWidget(m_stepCombo);
    stepRow->addStretch();
    sliderLayout->addLayout(stepRow);

    // Translation row: PX  PY  PZ
    auto* transRow = new QHBoxLayout;
    const char* transLabels[] = {"PX", "PY", "PZ"};
    for (int i = 0; i < 3; ++i) {
        transRow->addWidget(new QLabel(tr(transLabels[i])));
        m_sliders[i] = new QDoubleSpinBox;
        m_sliders[i]->setRange(-100.0, 100.0);
        m_sliders[i]->setDecimals(2);
        m_sliders[i]->setSingleStep(0.10);   // default: Medium
        m_sliders[i]->setValue(0.0);
        m_sliders[i]->setKeyboardTracking(false);
        m_sliders[i]->setFixedWidth(100);
        transRow->addWidget(m_sliders[i]);
    }
    sliderLayout->addLayout(transRow);

    // Rotation row: RX  RY  RZ
    auto* rotRow = new QHBoxLayout;
    const char* rotLabels[] = {"RX", "RY", "RZ"};
    for (int i = 0; i < 3; ++i) {
        rotRow->addWidget(new QLabel(tr(rotLabels[i])));
        m_sliders[3 + i] = new QDoubleSpinBox;
        m_sliders[3 + i]->setRange(-100.0, 100.0);
        m_sliders[3 + i]->setDecimals(2);
        m_sliders[3 + i]->setSingleStep(0.05);  // default: Medium
        m_sliders[3 + i]->setValue(0.0);
        m_sliders[3 + i]->setKeyboardTracking(false);
        m_sliders[3 + i]->setFixedWidth(100);
        rotRow->addWidget(m_sliders[3 + i]);
    }
    sliderLayout->addLayout(rotRow);

    mainLayout->addWidget(sliderGroup);

    // Connect slider signals
    connect(m_sliders[0], QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &LoopClosureDialog::onSliderPXChanged);
    connect(m_sliders[1], QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &LoopClosureDialog::onSliderPYChanged);
    connect(m_sliders[2], QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &LoopClosureDialog::onSliderPZChanged);
    connect(m_sliders[3], QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &LoopClosureDialog::onSliderRXChanged);
    connect(m_sliders[4], QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &LoopClosureDialog::onSliderRYChanged);
    connect(m_sliders[5], QOverload<double>::of(&QDoubleSpinBox::valueChanged),
            this, &LoopClosureDialog::onSliderRZChanged);

    // Step gear selector
    connect(m_stepCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int index) {
        // Gear presets: {trans_step, rot_step}
        static const double presets[][2] = {
            {0.01, 0.01},   // Fine
            {0.10, 0.05},   // Medium
            {0.50, 0.20},   // Coarse
            {1.00, 0.785},  // Large (45° ≈ 0.785 rad)
        };
        int i = std::clamp(index, 0, 3);
        for (int j = 0; j < 3; ++j) {
            m_sliders[j]->setSingleStep(presets[i][0]);
            m_sliders[3 + j]->setSingleStep(presets[i][1]);
        }
    });

    // --- Action buttons ---
    auto* btnRow = new QHBoxLayout;
    m_autoAlignBtn = new QPushButton(tr("Auto Align"));
    m_scanMatchBtn = new QPushButton(tr("Scan Matching"));
    m_resetBtn     = new QPushButton(tr("Reset"));
    btnRow->addWidget(m_autoAlignBtn);
    btnRow->addWidget(m_scanMatchBtn);
    btnRow->addWidget(m_resetBtn);
    mainLayout->addLayout(btnRow);

    connect(m_autoAlignBtn, &QPushButton::clicked, this, &LoopClosureDialog::onAutoAlign);
    connect(m_scanMatchBtn, &QPushButton::clicked, this, &LoopClosureDialog::onScanMatching);
    connect(m_resetBtn, &QPushButton::clicked, this, &LoopClosureDialog::onReset);

    // --- Progress bar ---
    m_progressBar = new QProgressBar;
    m_progressBar->setVisible(false);
    m_progressBar->setRange(0, 100);
    mainLayout->addWidget(m_progressBar);

    // --- Status label ---
    m_statusLabel = new QLabel;
    m_statusLabel->setStyleSheet("color: #888;");
    mainLayout->addWidget(m_statusLabel);

    // --- Bottom buttons ---
    mainLayout->addStretch();
    auto* bottomRow = new QHBoxLayout;
    m_addEdgeBtn = new QPushButton(tr("Add Edge"));
    m_cancelBtn  = new QPushButton(tr("Cancel"));
    bottomRow->addStretch();
    bottomRow->addWidget(m_addEdgeBtn);
    bottomRow->addWidget(m_cancelBtn);
    mainLayout->addLayout(bottomRow);

    connect(m_addEdgeBtn, &QPushButton::clicked, this, &LoopClosureDialog::onAddEdge);
    connect(m_cancelBtn, &QPushButton::clicked, this, &QDialog::reject);
}

// ---------------------------------------------------------------------------
// Slider slots — delta application with auto-reset
// ---------------------------------------------------------------------------

void LoopClosureDialog::applySliderDelta(int axis, double delta, bool isRotation) {
    if (std::abs(delta) < 1e-9) return;

    if (isRotation) {
        // Post-multiply = rotate around LOCAL axis
        switch (axis) {
        case 0: m_endPose = m_endPose * Eigen::AngleAxisd(delta, Eigen::Vector3d::UnitX()); break;
        case 1: m_endPose = m_endPose * Eigen::AngleAxisd(delta, Eigen::Vector3d::UnitY()); break;
        case 2: m_endPose = m_endPose * Eigen::AngleAxisd(delta, Eigen::Vector3d::UnitZ()); break;
        }
    } else {
        // Translate along LOCAL axis
        m_endPose.translation() += m_endPose.linear().col(axis) * delta;
    }

    updateFitnessScore();
    updatePreview();
}

void LoopClosureDialog::onSliderPXChanged(double value) {
    double delta = value - m_sliderPrevValues[0];
    m_sliderPrevValues[0] = value;
    if (std::abs(delta) < 1e-9) return;
    // Reset spinbox to 0 for next drag
    m_sliders[0]->blockSignals(true);
    m_sliders[0]->setValue(0.0);
    m_sliderPrevValues[0] = 0.0;
    m_sliders[0]->blockSignals(false);
    applySliderDelta(0, delta, false);
}

void LoopClosureDialog::onSliderPYChanged(double value) {
    double delta = value - m_sliderPrevValues[1];
    m_sliderPrevValues[1] = value;
    if (std::abs(delta) < 1e-9) return;
    m_sliders[1]->blockSignals(true);
    m_sliders[1]->setValue(0.0);
    m_sliderPrevValues[1] = 0.0;
    m_sliders[1]->blockSignals(false);
    applySliderDelta(1, delta, false);
}

void LoopClosureDialog::onSliderPZChanged(double value) {
    double delta = value - m_sliderPrevValues[2];
    m_sliderPrevValues[2] = value;
    if (std::abs(delta) < 1e-9) return;
    m_sliders[2]->blockSignals(true);
    m_sliders[2]->setValue(0.0);
    m_sliderPrevValues[2] = 0.0;
    m_sliders[2]->blockSignals(false);
    applySliderDelta(2, delta, false);
}

void LoopClosureDialog::onSliderRXChanged(double value) {
    double delta = value - m_sliderPrevValues[3];
    m_sliderPrevValues[3] = value;
    if (std::abs(delta) < 1e-9) return;
    m_sliders[3]->blockSignals(true);
    m_sliders[3]->setValue(0.0);
    m_sliderPrevValues[3] = 0.0;
    m_sliders[3]->blockSignals(false);
    applySliderDelta(0, delta, true);
}

void LoopClosureDialog::onSliderRYChanged(double value) {
    double delta = value - m_sliderPrevValues[4];
    m_sliderPrevValues[4] = value;
    if (std::abs(delta) < 1e-9) return;
    m_sliders[4]->blockSignals(true);
    m_sliders[4]->setValue(0.0);
    m_sliderPrevValues[4] = 0.0;
    m_sliders[4]->blockSignals(false);
    applySliderDelta(1, delta, true);
}

void LoopClosureDialog::onSliderRZChanged(double value) {
    double delta = value - m_sliderPrevValues[5];
    m_sliderPrevValues[5] = value;
    if (std::abs(delta) < 1e-9) return;
    m_sliders[5]->blockSignals(true);
    m_sliders[5]->setValue(0.0);
    m_sliderPrevValues[5] = 0.0;
    m_sliders[5]->blockSignals(false);
    applySliderDelta(2, delta, true);
}

// ---------------------------------------------------------------------------
// Fitness score
// ---------------------------------------------------------------------------

void LoopClosureDialog::updateFitnessScore() {
    Eigen::Isometry3d relative = m_beginPose.inverse() * m_endPose;
    double score = hdl_graph_slam::InformationMatrixCalculator::calc_fitness_score(
        m_beginCloud, m_endCloud, relative, 1.0);
    score = std::min(1000000.0, score);
    m_fitnessLabel->setText(QString("fitness_score: %1").arg(score, 0, 'f', 4));
}

// ---------------------------------------------------------------------------
// Preview update
// ---------------------------------------------------------------------------

void LoopClosureDialog::updatePreview() {
    m_miniViewport->updateEndPose(m_endPose, m_beginPose);
}

// ---------------------------------------------------------------------------
// Auto Align (FPFH global registration)
// ---------------------------------------------------------------------------

void LoopClosureDialog::onAutoAlign() {
    if (m_fpfhRunning || m_scanMatchRunning) return;

    // Build FPFH parameter dialog inline
    QDialog dlg(this);
    dlg.setWindowTitle(tr("FPFH Auto Align"));
    dlg.setModal(true);

    auto* form = new QFormLayout(&dlg);

    auto* normalRadius = new QDoubleSpinBox;
    normalRadius->setRange(0.1, 10.0);
    normalRadius->setValue(0.5);
    normalRadius->setDecimals(2);
    normalRadius->setSingleStep(0.1);
    form->addRow(tr("Normal radius:"), normalRadius);

    auto* searchRadius = new QDoubleSpinBox;
    searchRadius->setRange(0.1, 10.0);
    searchRadius->setValue(1.0);
    searchRadius->setDecimals(2);
    searchRadius->setSingleStep(0.1);
    form->addRow(tr("FPFH search radius:"), searchRadius);

    auto* maxIter = new QSpinBox;
    maxIter->setRange(1, 100000);
    maxIter->setValue(50000);
    form->addRow(tr("Max iterations:"), maxIter);

    auto* numSamples = new QSpinBox;
    numSamples->setRange(1, 100);
    numSamples->setValue(5);
    form->addRow(tr("Num samples:"), numSamples);

    auto* corrRandom = new QSpinBox;
    corrRandom->setRange(1, 100);
    corrRandom->setValue(20);
    form->addRow(tr("Correspondence randomness:"), corrRandom);

    auto* simThresh = new QDoubleSpinBox;
    simThresh->setRange(0.1, 1.0);
    simThresh->setValue(0.9);
    simThresh->setDecimals(2);
    simThresh->setSingleStep(0.05);
    form->addRow(tr("Similarity threshold:"), simThresh);

    auto* maxCorrDist = new QDoubleSpinBox;
    maxCorrDist->setRange(0.1, 50.0);
    maxCorrDist->setValue(2.5);
    maxCorrDist->setDecimals(2);
    maxCorrDist->setSingleStep(0.1);
    form->addRow(tr("Max correspondence dist:"), maxCorrDist);

    auto* inlierFrac = new QDoubleSpinBox;
    inlierFrac->setRange(0.01, 1.0);
    inlierFrac->setValue(0.2);
    inlierFrac->setDecimals(2);
    inlierFrac->setSingleStep(0.05);
    form->addRow(tr("Inlier fraction:"), inlierFrac);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    form->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() != QDialog::Accepted) return;

    runFpfhAlign(normalRadius->value(), searchRadius->value(),
                 maxIter->value(), numSamples->value(), corrRandom->value(),
                 simThresh->value(), maxCorrDist->value(), inlierFrac->value());
}

void LoopClosureDialog::runFpfhAlign(double normalRadius, double searchRadius,
                                      int maxIter, int numSamples, int corrRandomness,
                                      double similarityThresh, double maxCorrDist,
                                      double inlierFrac) {
    m_fpfhRunning = true;
    m_fpfhProgress = 0;
    m_progressBar->setVisible(true);
    m_progressBar->setValue(0);
    m_autoAlignBtn->setEnabled(false);
    m_scanMatchBtn->setEnabled(false);
    m_statusLabel->setText(tr("FPFH alignment running..."));

    // Capture cloud shared_ptrs (safe for background thread)
    CloudPtr beginCloud = m_beginCloud;
    CloudPtr endCloud   = m_endCloud;
    Eigen::Isometry3d beginPose = m_beginPose;
    std::atomic_int* progress = &m_fpfhProgress;

    // Progress polling timer
    auto* pollTimer = new QTimer(this);
    connect(pollTimer, &QTimer::timeout, this, [this, progress]() {
        int p = progress->load();
        m_progressBar->setValue(p * 20);  // 5 stages → 0/20/40/60/80/100
        if (p >= 5) {
            m_progressBar->setValue(100);
        }
    });
    pollTimer->start(100);

    m_fpfhWatcher = new QFutureWatcher<Eigen::Isometry3d>(this);
    connect(m_fpfhWatcher, &QFutureWatcher<Eigen::Isometry3d>::finished,
            this, [this, pollTimer]() {
        pollTimer->stop();
        pollTimer->deleteLater();
        onFpfhAlignFinished();
    });

    auto future = QtConcurrent::run([=]() -> Eigen::Isometry3d {
        using FeatureT = pcl::FPFHSignature33;
        using PointN = pcl::PointNormal;

        // Stage 1: Copy point clouds
        progress->store(1);
        pcl::PointCloud<PointN>::Ptr src(new pcl::PointCloud<PointN>());
        pcl::PointCloud<PointN>::Ptr tgt(new pcl::PointCloud<PointN>());
        pcl::copyPointCloud(*endCloud, *src);
        pcl::copyPointCloud(*beginCloud, *tgt);

        // Stage 2: Normal estimation
        progress->store(2);
        pcl::NormalEstimationOMP<PointN, PointN> nest;
        nest.setRadiusSearch(normalRadius);
        nest.setInputCloud(src);
        nest.compute(*src);
        nest.setInputCloud(tgt);
        nest.compute(*tgt);

        // Stage 3: FPFH features
        progress->store(3);
        pcl::PointCloud<FeatureT>::Ptr srcFeat(new pcl::PointCloud<FeatureT>());
        pcl::PointCloud<FeatureT>::Ptr tgtFeat(new pcl::PointCloud<FeatureT>());
        pcl::FPFHEstimation<PointN, PointN, FeatureT> fest;
        fest.setRadiusSearch(searchRadius);
        fest.setInputCloud(src);
        fest.setInputNormals(src);
        fest.compute(*srcFeat);
        fest.setInputCloud(tgt);
        fest.setInputNormals(tgt);
        fest.compute(*tgtFeat);

        // Stage 4: SAC prerejective alignment
        progress->store(4);
        pcl::SampleConsensusPrerejective<PointN, PointN, FeatureT> align;
        align.setInputSource(src);
        align.setSourceFeatures(srcFeat);
        align.setInputTarget(tgt);
        align.setTargetFeatures(tgtFeat);
        align.setMaximumIterations(maxIter);
        align.setNumberOfSamples(numSamples);
        align.setCorrespondenceRandomness(corrRandomness);
        align.setSimilarityThreshold(similarityThresh);
        align.setMaxCorrespondenceDistance(maxCorrDist);
        align.setInlierFraction(inlierFrac);

        pcl::PointCloud<PointN>::Ptr aligned(new pcl::PointCloud<PointN>());
        align.align(*aligned);

        Eigen::Isometry3d rel;
        rel.matrix() = align.getFinalTransformation().cast<double>();

        progress->store(5);
        // Return new end pose in world frame: beginPose * relative
        return beginPose * rel;
    });

    m_fpfhWatcher->setFuture(future);
}

void LoopClosureDialog::onFpfhAlignFinished() {
    m_endPose = m_fpfhWatcher->result();
    m_progressBar->setVisible(false);
    m_autoAlignBtn->setEnabled(true);
    m_scanMatchBtn->setEnabled(true);
    m_fpfhRunning = false;
    m_statusLabel->setText(tr("FPFH alignment complete"));

    updateFitnessScore();
    updatePreview();

    m_fpfhWatcher->deleteLater();
    m_fpfhWatcher = nullptr;
}

// ---------------------------------------------------------------------------
// Scan Matching (ICP / GICP / NDT local registration)
// ---------------------------------------------------------------------------

void LoopClosureDialog::onScanMatching() {
    if (m_fpfhRunning || m_scanMatchRunning) return;

    // Build registration config dialog inline
    QDialog dlg(this);
    dlg.setWindowTitle(tr("Scan Matching"));
    dlg.setModal(true);

    auto* form = new QFormLayout(&dlg);

    // Method combo
    auto* methodCombo = new QComboBox;
    for (const char* name : m_regMethods->method_names()) {
        methodCombo->addItem(QString::fromUtf8(name));
    }
    methodCombo->setCurrentIndex(m_regMethods->get_method_index());
    form->addRow(tr("Method:"), methodCombo);

    // Max iterations
    auto* maxIterSpin = new QSpinBox;
    maxIterSpin->setRange(1, 512);
    maxIterSpin->setValue(m_regMethods->get_max_iterations());
    form->addRow(tr("Max iterations:"), maxIterSpin);

    // Transformation epsilon
    auto* epsSpin = new QDoubleSpinBox;
    epsSpin->setRange(1e-6, 1e-1);
    epsSpin->setDecimals(6);
    epsSpin->setValue(m_regMethods->get_transformation_epsilon());
    epsSpin->setSingleStep(1e-5);
    form->addRow(tr("Transformation epsilon:"), epsSpin);

    // Resolution (for NDT)
    auto* resSpin = new QDoubleSpinBox;
    resSpin->setRange(0.1, 20.0);
    resSpin->setDecimals(1);
    resSpin->setValue(m_regMethods->get_resolution());
    resSpin->setSingleStep(0.5);
    form->addRow(tr("Resolution (NDT):"), resSpin);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    form->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() != QDialog::Accepted) return;

    int methodIdx = methodCombo->currentIndex();

    // Check for unavailable methods
    if (methodIdx >= 3) {
        try {
            m_regMethods->set_method_index(methodIdx);
            m_regMethods->method();  // will throw if unavailable
        } catch (const std::runtime_error& e) {
            QMessageBox::warning(this, tr("Method Unavailable"), QString::fromUtf8(e.what()));
            return;
        }
    }

    runScanMatching(methodIdx, maxIterSpin->value(),
                    static_cast<float>(epsSpin->value()),
                    static_cast<float>(resSpin->value()));
}

void LoopClosureDialog::runScanMatching(int methodIndex, int maxIterations,
                                         float transEpsilon, float resolution) {
    m_scanMatchRunning = true;
    m_progressBar->setVisible(true);
    m_progressBar->setRange(0, 0);  // indeterminate
    m_autoAlignBtn->setEnabled(false);
    m_scanMatchBtn->setEnabled(false);
    m_statusLabel->setText(tr("Scan matching running..."));

    CloudPtr beginCloud = m_beginCloud;
    CloudPtr endCloud   = m_endCloud;
    Eigen::Isometry3d beginPose = m_beginPose;
    Eigen::Isometry3d endPose   = m_endPose;

    m_scanMatchWatcher = new QFutureWatcher<Eigen::Isometry3d>(this);
    connect(m_scanMatchWatcher, &QFutureWatcher<Eigen::Isometry3d>::finished,
            this, &LoopClosureDialog::onScanMatchFinished);

    auto future = QtConcurrent::run([=]() -> Eigen::Isometry3d {
        // Create a fresh registration instance on this thread
        hdl_graph_slam::RegistrationMethods reg;
        reg.set_method_index(methodIndex);
        reg.set_max_iterations(maxIterations);
        reg.set_transformation_epsilon(transEpsilon);
        reg.set_resolution(resolution);

        auto registration = reg.method();
        registration->setInputTarget(beginCloud);
        registration->setInputSource(endCloud);

        pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
        Eigen::Isometry3d relative = beginPose.inverse() * endPose;
        registration->align(*aligned, relative.matrix().cast<float>());

        if (!registration->hasConverged()) {
            // Still return the final transformation even if not converged
        }

        relative.matrix() = registration->getFinalTransformation().cast<double>();
        return beginPose * relative;  // new end pose in world frame
    });

    m_scanMatchWatcher->setFuture(future);
}

void LoopClosureDialog::onScanMatchFinished() {
    m_endPose = m_scanMatchWatcher->result();
    m_progressBar->setVisible(false);
    m_autoAlignBtn->setEnabled(true);
    m_scanMatchBtn->setEnabled(true);
    m_scanMatchRunning = false;
    m_statusLabel->setText(tr("Scan matching complete"));

    updateFitnessScore();
    updatePreview();

    m_scanMatchWatcher->deleteLater();
    m_scanMatchWatcher = nullptr;
}

// ---------------------------------------------------------------------------
// Reset
// ---------------------------------------------------------------------------

void LoopClosureDialog::onReset() {
    m_endPose = m_endPoseInit;
    updateFitnessScore();
    updatePreview();
    m_statusLabel->setText(tr("Pose reset to initial"));
}

// ---------------------------------------------------------------------------
// Add Edge — commit to graph
// ---------------------------------------------------------------------------

void LoopClosureDialog::onAddEdge() {
    auto itBegin = m_graph->keyframes.find(m_beginVertexId);
    auto itEnd   = m_graph->keyframes.find(m_endVertexId);
    if (itBegin == m_graph->keyframes.end() || itEnd == m_graph->keyframes.end()) {
        QMessageBox::warning(this, tr("Error"), tr("Keyframe no longer exists"));
        reject();
        return;
    }

    auto& beginKf = itBegin->second;
    auto& endKf   = itEnd->second;
    Eigen::Isometry3d relative = m_beginPose.inverse() * m_endPose;

    // Check for existing edge between these two vertices
    bool updated = false;
    if (beginKf->node && endKf->node) {
        for (auto* edge : beginKf->node->edges()) {
            const auto& verts = edge->vertices();
            bool hasBegin = false, hasEnd = false;
            for (size_t i = 0; i < verts.size(); ++i) {
                if (verts[i] == beginKf->node) hasBegin = true;
                if (verts[i] == endKf->node)   hasEnd   = true;
            }
            if (hasBegin && hasEnd) {
                // Update existing edge measurement
                auto* se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
                if (se3) {
                    // Check direction
                    if (se3->vertices()[0] == beginKf->node &&
                        se3->vertices()[1] == endKf->node) {
                        se3->setMeasurement(relative);
                    } else {
                        se3->setMeasurement(relative.inverse());
                    }
                }
                updated = true;
                break;
            }
        }
    }

    if (!updated) {
        m_graph->add_edge(beginKf, endKf, relative);
    }

    m_graph->optimize();
    accept();  // QDialog::Accepted
}
