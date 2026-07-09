/**
 * @file LoopClosureDialog.cpp
 * @brief 手动闭环确认对话框实现
 *
 * 实现手动闭环的完整工作流：
 * - mergeAdjacentClouds() 自由函数：合并相邻关键帧的点云
 * - 对话框 UI 搭建：迷你视口、滑块、按钮、进度条
 * - 滑块增量调节（自动归零 + 局部坐标系变换）
 * - FPFH 全局配准（后台线程执行，进度轮询）
 * - ICP/GICP/NDT 扫描匹配（后台线程执行）
 * - 适应度分数实时计算
 * - 闭环边提交到图谱（更新已有边或创建新边）
 */

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
// mergeAdjacentClouds — 自由函数
// ---------------------------------------------------------------------------

/**
 * @brief 合并与指定顶点相邻关键帧的点云到该顶点的局部坐标系
 *
 * 将 centerId ± windowHalfSize 范围内所有存在且非空点云的关键帧
 * 变换到 centerId 的局部坐标系后合并。
 *
 * @param graph          交互图谱
 * @param centerId       中心顶点 ID
 * @param windowHalfSize 每侧合并帧数
 * @return 合并后的点云（可能为空）
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr mergeAdjacentClouds(
    const hdl_graph_slam::InteractiveGraph* graph, long centerId,
    int windowHalfSize) {

    using PointT = pcl::PointXYZI;
    auto merged = pcl::make_shared<pcl::PointCloud<PointT>>();

    // 查找中心关键帧（必需）
    auto itCenter = graph->keyframes.find(centerId);
    if (itCenter == graph->keyframes.end()) return merged;
    auto& centerKf = itCenter->second;
    if (!centerKf->cloud || centerKf->cloud->empty()) return merged;
    Eigen::Isometry3d centerPose = centerKf->estimate();

    // 合并 centerId-windowHalfSize 到 centerId+windowHalfSize 的关键帧
    for (long id = centerId - windowHalfSize; id <= centerId + windowHalfSize; ++id) {
        auto it = graph->keyframes.find(id);
        if (it == graph->keyframes.end()) continue;
        auto& kf = it->second;
        if (!kf->cloud || kf->cloud->empty()) continue;

        // 计算相对位姿：kf 在 centerPose 坐标系下的位姿
        Eigen::Isometry3d T_rel = centerPose.inverse() * kf->estimate();

        // 变换点云并合并
        pcl::PointCloud<PointT>::Ptr transformed(new pcl::PointCloud<PointT>());
        pcl::transformPointCloud(*kf->cloud, *transformed, T_rel.matrix());
        *merged += *transformed;
    }

    return merged;
}

// ---------------------------------------------------------------------------
// 构造 / 析构
// ---------------------------------------------------------------------------

/**
 * @brief 构造函数
 *
 * 获取起点和终点的关键帧位姿，创建配准方法工厂，
 * 初始化滑块值，搭建 UI，并在迷你视口中显示初始预览。
 */
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
    setMinimumSize(600, 780);

    // 获取图谱
    m_graph = m_manager->graph();
    if (!m_graph) {
        QMessageBox::warning(parent, tr("Error"), tr("No graph loaded"));
        return;
    }

    // 使用调用者传入的合并点云
    m_beginCloud = beginCloud;
    m_endCloud   = endCloud;
    if (!m_beginCloud || !m_endCloud || m_beginCloud->empty() || m_endCloud->empty()) {
        QMessageBox::warning(parent, tr("Error"),
                             tr("One or both keyframes have no point cloud data"));
        return;
    }

    // 查找关键帧获取位姿
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

    setupUi();

    // 初始预览
    m_miniViewport->setClouds(m_beginCloud, m_beginPose, m_endCloud, m_endPose);
    updateFitnessScore();
}

/**
 * @brief 析构函数
 *
 * 等待后台配准线程完成（如果仍在运行）。
 */
LoopClosureDialog::~LoopClosureDialog() {
    // 如果监听器正在运行，等待完成（它们持有 shared_ptr，安全）
    if (m_fpfhWatcher && m_fpfhWatcher->isRunning()) {
        m_fpfhWatcher->waitForFinished();
    }
    if (m_scanMatchWatcher && m_scanMatchWatcher->isRunning()) {
        m_scanMatchWatcher->waitForFinished();
    }
}

// ---------------------------------------------------------------------------
// UI 设置
// ---------------------------------------------------------------------------

/**
 * @brief 创建 UI 布局
 *
 * 从上到下依次为：
 * 1. 迷你视口（512×512 预览）
 * 2. 适应度分数标签
 * 3. 手动调节滑块组（含步长档位选择器）
 * 4. 操作按钮行（FPFH、扫描匹配、重置）
 * 5. 进度条 + 状态标签
 * 6. 底部按钮（添加边 / 取消）
 */
void LoopClosureDialog::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);

    // --- 迷你视口（560×560） ---
    m_miniViewport = new MiniViewportWidget(this);
    m_miniViewport->setFixedSize(560, 560);
    mainLayout->addWidget(m_miniViewport, 0, Qt::AlignHCenter);

    // --- 适应度分数 ---
    m_fitnessLabel = new QLabel(tr("fitness_score: —"));
    m_fitnessLabel->setObjectName("FitnessLabel");
    mainLayout->addWidget(m_fitnessLabel);

    // --- 手动调节滑块组 ---
    auto* sliderGroup = new QGroupBox(tr("Manual Adjustment (local frame)"));
    auto* sliderLayout = new QVBoxLayout(sliderGroup);

    // 步长档位选择器（Fine / Medium / Coarse / Large）
    auto* stepRow = new QHBoxLayout;
    stepRow->addWidget(new QLabel(tr("Step:")));
    m_stepCombo = new QComboBox;
    m_stepCombo->addItem(tr("Fine     — 0.01m /  0.6°"),  0);
    m_stepCombo->addItem(tr("Medium   — 0.10m /  2.9°"),  1);
    m_stepCombo->addItem(tr("Coarse   — 0.50m / 11.5°"),  2);
    m_stepCombo->addItem(tr("Large    — 1.00m / 45.0°"),  3);
    m_stepCombo->setCurrentIndex(1);  // 默认：Medium
    stepRow->addWidget(m_stepCombo);
    stepRow->addStretch();
    sliderLayout->addLayout(stepRow);

    // 平移行：PX  PY  PZ（每轴 ◀ 减 / ▶ 加）
    auto* transRow = new QHBoxLayout;
    const char* transLabels[] = {"PX", "PY", "PZ"};
    for (int i = 0; i < 3; ++i) {
        auto* label = new QLabel(tr(transLabels[i]));
        label->setObjectName("transLabel");
        transRow->addWidget(label);

        m_stepBtns[i][0] = new QPushButton;
        m_stepBtns[i][0]->setObjectName("stepBtn");
        m_stepBtns[i][0]->setFixedSize(26, 26);
        m_stepBtns[i][0]->setText("◀");
        transRow->addWidget(m_stepBtns[i][0]);

        m_stepBtns[i][1] = new QPushButton;
        m_stepBtns[i][1]->setObjectName("stepBtn");
        m_stepBtns[i][1]->setFixedSize(26, 26);
        m_stepBtns[i][1]->setText("▶");
        transRow->addWidget(m_stepBtns[i][1]);

        transRow->addStretch();
    }
    sliderLayout->addLayout(transRow);

    // 旋转行：RX  RY  RZ（每轴 ◀ 减 / ▶ 加）
    auto* rotRow = new QHBoxLayout;
    const char* rotLabels[] = {"RX", "RY", "RZ"};
    for (int i = 0; i < 3; ++i) {
        auto* label = new QLabel(tr(rotLabels[i]));
        label->setObjectName("rotLabel");
        rotRow->addWidget(label);

        m_stepBtns[3 + i][0] = new QPushButton;
        m_stepBtns[3 + i][0]->setObjectName("stepBtn");
        m_stepBtns[3 + i][0]->setFixedSize(26, 26);
        m_stepBtns[3 + i][0]->setText("◀");
        rotRow->addWidget(m_stepBtns[3 + i][0]);

        m_stepBtns[3 + i][1] = new QPushButton;
        m_stepBtns[3 + i][1]->setObjectName("stepBtn");
        m_stepBtns[3 + i][1]->setFixedSize(26, 26);
        m_stepBtns[3 + i][1]->setText("▶");
        rotRow->addWidget(m_stepBtns[3 + i][1]);

        rotRow->addStretch();
    }
    sliderLayout->addLayout(rotRow);

    mainLayout->addWidget(sliderGroup);

    // 连接步进按钮信号（6 轴 × 2 方向 = 12 个按钮）
    // 平移轴（0,1,2）
    for (int axis = 0; axis < 3; ++axis) {
        connect(m_stepBtns[axis][0], &QPushButton::clicked, this, [this, axis]() {
            onStepButton(axis, false, -1);  // 负方向
        });
        connect(m_stepBtns[axis][1], &QPushButton::clicked, this, [this, axis]() {
            onStepButton(axis, false, +1);  // 正方向
        });
    }
    // 旋转轴（3,4,5）
    for (int axis = 0; axis < 3; ++axis) {
        connect(m_stepBtns[3 + axis][0], &QPushButton::clicked, this, [this, axis]() {
            onStepButton(axis, true, -1);
        });
        connect(m_stepBtns[3 + axis][1], &QPushButton::clicked, this, [this, axis]() {
            onStepButton(axis, true, +1);
        });
    }

    // --- 操作按钮行 ---
    auto* btnRow = new QHBoxLayout;
    m_autoAlignBtn = new QPushButton(tr("Auto Align"));
    m_scanMatchBtn = new QPushButton(tr("Scan Matching"));
    m_resetBtn     = new QPushButton(tr("Reset"));
    m_resetBtn->setObjectName("tertiaryButton");
    btnRow->addWidget(m_autoAlignBtn);
    btnRow->addWidget(m_scanMatchBtn);
    btnRow->addWidget(m_resetBtn);
    mainLayout->addLayout(btnRow);

    connect(m_autoAlignBtn, &QPushButton::clicked, this, &LoopClosureDialog::onAutoAlign);
    connect(m_scanMatchBtn, &QPushButton::clicked, this, &LoopClosureDialog::onScanMatching);
    connect(m_resetBtn, &QPushButton::clicked, this, &LoopClosureDialog::onReset);

    // --- 进度条 ---
    m_progressBar = new QProgressBar;
    m_progressBar->setVisible(false);
    m_progressBar->setRange(0, 100);
    mainLayout->addWidget(m_progressBar);

    // --- 状态标签 ---
    m_statusLabel = new QLabel;
    m_statusLabel->setStyleSheet("color: #999999;");
    mainLayout->addWidget(m_statusLabel);

    // --- 底部按钮 ---
    mainLayout->addStretch();
    auto* bottomRow = new QHBoxLayout;
    m_addEdgeBtn = new QPushButton(tr("Add Edge"));
    m_addEdgeBtn->setObjectName("primaryButton");
    m_cancelBtn  = new QPushButton(tr("Cancel"));
    m_cancelBtn->setObjectName("tertiaryButton");
    bottomRow->addStretch();
    bottomRow->addWidget(m_addEdgeBtn);
    bottomRow->addWidget(m_cancelBtn);
    mainLayout->addLayout(bottomRow);

    connect(m_addEdgeBtn, &QPushButton::clicked, this, &LoopClosureDialog::onAddEdge);
    connect(m_cancelBtn, &QPushButton::clicked, this, &QDialog::reject);
}

// ---------------------------------------------------------------------------
// 滑块槽函数 — 增量应用 + 自动归零
// ---------------------------------------------------------------------------

/**
 * @brief 应用滑块增量到终点位姿
 *
 * 平移：沿终点位姿的局部坐标轴（m_endPose.linear().col(axis)）
 * 旋转：后乘（post-multiply）对应局部轴的 AngleAxis
 *
 * @param axis       轴索引（0=X, 1=Y, 2=Z）
 * @param delta      增量值
 * @param isRotation true=旋转, false=平移
 */
void LoopClosureDialog::applySliderDelta(int axis, double delta, bool isRotation) {
    if (std::abs(delta) < 1e-9) return;

    if (isRotation) {
        // 后乘 = 绕局部坐标轴旋转
        switch (axis) {
        case 0: m_endPose = m_endPose * Eigen::AngleAxisd(delta, Eigen::Vector3d::UnitX()); break;
        case 1: m_endPose = m_endPose * Eigen::AngleAxisd(delta, Eigen::Vector3d::UnitY()); break;
        case 2: m_endPose = m_endPose * Eigen::AngleAxisd(delta, Eigen::Vector3d::UnitZ()); break;
        }
    } else {
        // 沿局部坐标轴平移
        m_endPose.translation() += m_endPose.linear().col(axis) * delta;
    }

    updateFitnessScore();
    updatePreview();
}

// ---------------------------------------------------------------------------
// 步进按钮槽函数 — 按当前步长档位应用增量
// ---------------------------------------------------------------------------

/**
 * @brief 步进按钮（◀/▶）点击处理
 *
 * 根据步长档位选择器的当前档位读取步长，按方向应用增量到终点位姿。
 *
 * @param axis       轴索引（0=X, 1=Y, 2=Z）
 * @param isRotation true=旋转, false=平移
 * @param direction  方向（-1=减, +1=加）
 */
void LoopClosureDialog::onStepButton(int axis, bool isRotation, int direction) {
    // 档位预设：{平移步长, 旋转步长}
    static const double presets[][2] = {
        {0.01, 0.01},   // Fine
        {0.10, 0.05},   // Medium
        {0.50, 0.20},   // Coarse
        {1.00, 0.785},  // Large（45° ≈ 0.785 rad）
    };
    int idx = std::clamp(m_stepCombo->currentIndex(), 0, 3);
    double step = presets[idx][isRotation ? 1 : 0];
    double delta = direction * step;

    applySliderDelta(axis, delta, isRotation);
}

// ---------------------------------------------------------------------------
// 适应度分数
// ---------------------------------------------------------------------------

/**
 * @brief 计算并更新适应度分数显示
 *
 * 使用 InformationMatrixCalculator 计算起点和终点点云在当前相对位姿下的
 * 配准适应度分数（值越小配准质量越好）。
 */
void LoopClosureDialog::updateFitnessScore() {
    Eigen::Isometry3d relative = m_beginPose.inverse() * m_endPose;
    double score = hdl_graph_slam::InformationMatrixCalculator::calc_fitness_score(
        m_beginCloud, m_endCloud, relative, 1.0);
    score = std::min(1000000.0, score);
    m_fitnessLabel->setText(QString("fitness_score: %1").arg(score, 0, 'f', 4));
}

// ---------------------------------------------------------------------------
// 预览更新
// ---------------------------------------------------------------------------

void LoopClosureDialog::updatePreview() {
    m_miniViewport->updateEndPose(m_endPose, m_beginPose);
}

// ---------------------------------------------------------------------------
// FPFH 全局配准
// ---------------------------------------------------------------------------

/**
 * @brief 自动对齐按钮处理
 *
 * 弹出 FPFH 参数配置对话框，参数配置完成后在后台线程执行配准。
 */
void LoopClosureDialog::onAutoAlign() {
    if (m_fpfhRunning || m_scanMatchRunning) return;

    // 内联创建 FPFH 参数对话框
    QDialog dlg(this);
    dlg.setWindowTitle(tr("FPFH Auto Align"));
    dlg.setModal(true);

    auto* form = new QFormLayout(&dlg);

    // 法线估计搜索半径
    auto* normalRadius = new QDoubleSpinBox;
    normalRadius->setRange(0.1, 10.0);
    normalRadius->setValue(0.5);
    normalRadius->setDecimals(2);
    normalRadius->setSingleStep(0.1);
    form->addRow(tr("Normal radius:"), normalRadius);

    // FPFH 特征搜索半径
    auto* searchRadius = new QDoubleSpinBox;
    searchRadius->setRange(0.1, 10.0);
    searchRadius->setValue(1.0);
    searchRadius->setDecimals(2);
    searchRadius->setSingleStep(0.1);
    form->addRow(tr("FPFH search radius:"), searchRadius);

    // SAC 最大迭代次数
    auto* maxIter = new QSpinBox;
    maxIter->setRange(1, 100000);
    maxIter->setValue(50000);
    form->addRow(tr("Max iterations:"), maxIter);

    // 采样点数
    auto* numSamples = new QSpinBox;
    numSamples->setRange(1, 100);
    numSamples->setValue(5);
    form->addRow(tr("Num samples:"), numSamples);

    // 对应点随机性
    auto* corrRandom = new QSpinBox;
    corrRandom->setRange(1, 100);
    corrRandom->setValue(20);
    form->addRow(tr("Correspondence randomness:"), corrRandom);

    // 特征相似度阈值
    auto* simThresh = new QDoubleSpinBox;
    simThresh->setRange(0.1, 1.0);
    simThresh->setValue(0.9);
    simThresh->setDecimals(2);
    simThresh->setSingleStep(0.05);
    form->addRow(tr("Similarity threshold:"), simThresh);

    // 最大对应点距离
    auto* maxCorrDist = new QDoubleSpinBox;
    maxCorrDist->setRange(0.1, 50.0);
    maxCorrDist->setValue(2.5);
    maxCorrDist->setDecimals(2);
    maxCorrDist->setSingleStep(0.1);
    form->addRow(tr("Max correspondence dist:"), maxCorrDist);

    // 内点比例
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

    // 启动 FPFH 配准
    runFpfhAlign(normalRadius->value(), searchRadius->value(),
                 maxIter->value(), numSamples->value(), corrRandom->value(),
                 simThresh->value(), maxCorrDist->value(), inlierFrac->value());
}

/**
 * @brief 在后台线程执行 FPFH 配准
 *
 * 步骤：
 * 1. 复制点云数据
 * 2. OMP 并行法线估计
 * 3. FPFH 特征计算
 * 4. SAC-Prerejective 全局配准
 * 5. 返回新的终点世界位姿 = beginPose * relative
 */
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

    // 捕获 shared_ptr（安全用于后台线程）
    CloudPtr beginCloud = m_beginCloud;
    CloudPtr endCloud   = m_endCloud;
    Eigen::Isometry3d beginPose = m_beginPose;
    std::atomic_int* progress = &m_fpfhProgress;

    // 进度轮询定时器
    auto* pollTimer = new QTimer(this);
    connect(pollTimer, &QTimer::timeout, this, [this, progress]() {
        int p = progress->load();
        m_progressBar->setValue(p * 20);  // 5 个阶段 → 0/20/40/60/80/100
        if (p >= 5) {
            m_progressBar->setValue(100);
        }
    });
    pollTimer->start(100);

    // 创建异步结果监听器
    m_fpfhWatcher = new QFutureWatcher<Eigen::Isometry3d>(this);
    connect(m_fpfhWatcher, &QFutureWatcher<Eigen::Isometry3d>::finished,
            this, [this, pollTimer]() {
        pollTimer->stop();
        pollTimer->deleteLater();
        onFpfhAlignFinished();
    });

    // 后台线程执行配准
    auto future = QtConcurrent::run([=]() -> Eigen::Isometry3d {
        using FeatureT = pcl::FPFHSignature33;
        using PointN = pcl::PointNormal;

        // Stage 1: 复制点云
        progress->store(1);
        pcl::PointCloud<PointN>::Ptr src(new pcl::PointCloud<PointN>());
        pcl::PointCloud<PointN>::Ptr tgt(new pcl::PointCloud<PointN>());
        pcl::copyPointCloud(*endCloud, *src);
        pcl::copyPointCloud(*beginCloud, *tgt);

        // Stage 2: 法线估计
        progress->store(2);
        pcl::NormalEstimationOMP<PointN, PointN> nest;
        nest.setRadiusSearch(normalRadius);
        nest.setInputCloud(src);
        nest.compute(*src);
        nest.setInputCloud(tgt);
        nest.compute(*tgt);

        // Stage 3: FPFH 特征
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

        // Stage 4: SAC-Prerejective 配准
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
        // 返回世界坐标系中的新终点位姿：beginPose * relative
        return beginPose * rel;
    });

    m_fpfhWatcher->setFuture(future);
}

/**
 * @brief FPFH 配准完成
 *
 * 获取配准结果，更新终点位姿，恢复 UI 状态。
 */
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
// 扫描匹配（ICP / GICP / NDT 局部配准）
// ---------------------------------------------------------------------------

/**
 * @brief 扫描匹配按钮处理
 *
 * 弹出配准参数配置对话框，检查配准方法可用性，
 * 然后在后台线程执行扫描匹配。
 */
void LoopClosureDialog::onScanMatching() {
    if (m_fpfhRunning || m_scanMatchRunning) return;

    // 内联创建配准参数对话框
    QDialog dlg(this);
    dlg.setWindowTitle(tr("Scan Matching"));
    dlg.setModal(true);

    auto* form = new QFormLayout(&dlg);

    // 配准方法选择
    auto* methodCombo = new QComboBox;
    for (const char* name : m_regMethods->method_names()) {
        methodCombo->addItem(QString::fromUtf8(name));
    }
    methodCombo->setCurrentIndex(m_regMethods->get_method_index());
    form->addRow(tr("Method:"), methodCombo);

    // 最大迭代次数
    auto* maxIterSpin = new QSpinBox;
    maxIterSpin->setRange(1, 512);
    maxIterSpin->setValue(m_regMethods->get_max_iterations());
    form->addRow(tr("Max iterations:"), maxIterSpin);

    // 变换精度
    auto* epsSpin = new QDoubleSpinBox;
    epsSpin->setRange(1e-6, 1e-1);
    epsSpin->setDecimals(6);
    epsSpin->setValue(m_regMethods->get_transformation_epsilon());
    epsSpin->setSingleStep(1e-5);
    form->addRow(tr("Transformation epsilon:"), epsSpin);

    // NDT 分辨率
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

    // 检查配准方法是否可用（方法索引 >= 3 时可能需要可选依赖）
    if (methodIdx >= 3) {
        try {
            m_regMethods->set_method_index(methodIdx);
            m_regMethods->method();  // 如果不可用将抛出异常
        } catch (const std::runtime_error& e) {
            QMessageBox::warning(this, tr("Method Unavailable"), QString::fromUtf8(e.what()));
            return;
        }
    }

    runScanMatching(methodIdx, maxIterSpin->value(),
                    static_cast<float>(epsSpin->value()),
                    static_cast<float>(resSpin->value()));
}

/**
 * @brief 在后台线程执行扫描匹配
 *
 * 创建独立的 RegistrationMethods 实例（线程安全），
 * 以当前相对位姿为初始猜测执行配准，返回新的终点世界位姿。
 */
void LoopClosureDialog::runScanMatching(int methodIndex, int maxIterations,
                                         float transEpsilon, float resolution) {
    m_scanMatchRunning = true;
    m_progressBar->setVisible(true);
    m_progressBar->setRange(0, 0);  // 不确定模式（滚动条）
    m_autoAlignBtn->setEnabled(false);
    m_scanMatchBtn->setEnabled(false);
    m_statusLabel->setText(tr("Scan matching running..."));

    CloudPtr beginCloud = m_beginCloud;
    CloudPtr endCloud   = m_endCloud;
    Eigen::Isometry3d beginPose = m_beginPose;
    Eigen::Isometry3d endPose   = m_endPose;

    // 创建异步结果监听器
    m_scanMatchWatcher = new QFutureWatcher<Eigen::Isometry3d>(this);
    connect(m_scanMatchWatcher, &QFutureWatcher<Eigen::Isometry3d>::finished,
            this, &LoopClosureDialog::onScanMatchFinished);

    // 后台线程执行配准
    auto future = QtConcurrent::run([=]() -> Eigen::Isometry3d {
        // 在此线程上创建全新的配准实例（线程安全）
        hdl_graph_slam::RegistrationMethods reg;
        reg.set_method_index(methodIndex);
        reg.set_max_iterations(maxIterations);
        reg.set_transformation_epsilon(transEpsilon);
        reg.set_resolution(resolution);

        auto registration = reg.method();
        registration->setInputTarget(beginCloud);  // 目标 = 起点云
        registration->setInputSource(endCloud);    // 源 = 终点云

        pcl::PointCloud<PointT>::Ptr aligned(new pcl::PointCloud<PointT>());
        Eigen::Isometry3d relative = beginPose.inverse() * endPose;
        registration->align(*aligned, relative.matrix().cast<float>());

        // 即使未收敛也返回最终变换矩阵
        relative.matrix() = registration->getFinalTransformation().cast<double>();
        return beginPose * relative;  // 世界坐标系中的新终点位姿
    });

    m_scanMatchWatcher->setFuture(future);
}

/**
 * @brief 扫描匹配完成
 *
 * 获取配准结果，更新终点位姿，恢复 UI 状态。
 */
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
// 重置
// ---------------------------------------------------------------------------

/**
 * @brief 重置终点位姿到初始值
 */
void LoopClosureDialog::onReset() {
    m_endPose = m_endPoseInit;
    updateFitnessScore();
    updatePreview();
    m_miniViewport->resetCamera();
    m_statusLabel->setText(tr("Pose and camera reset to initial"));
}

// ---------------------------------------------------------------------------
// 添加边 — 提交到图谱
// ---------------------------------------------------------------------------

/**
 * @brief 将闭环边提交到图优化系统
 *
 * 1. 查找两个关键帧
 * 2. 计算相对位姿 = beginPose⁻¹ * endPose
 * 3. 检查是否已存在边（更新测量值）或创建新边
 * 4. 执行优化
 * 5. 接受对话框
 */
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

    // 检查是否已存在边
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
                // 更新已有边的测量值
                auto* se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
                if (se3) {
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
