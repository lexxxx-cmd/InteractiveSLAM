/**
 * @file LoopClosureDialog.cpp
 * @brief 手动闭环确认对话框实现
 *
 * 实现手动闭环的完整工作流：
 * - mergeAdjacentClouds() 自由函数：合并相邻关键帧的点云
 * - 对话框 UI 搭建：迷你视口、滑块、按钮、进度条
 * - 滑块增量调节（自动归零 + 局部坐标系变换）
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
#include <QtConcurrent/QtConcurrent>

#include <algorithm>
#include <optional>
#include <utility>

#include <pcl/common/transforms.h>
#include <pcl/registration/registration.h>

#include <g2o/types/slam3d/types_slam3d.h>

// ---------------------------------------------------------------------------
// mergeAdjacentClouds — 自由函数
// ---------------------------------------------------------------------------

/**
 * @brief 查找两顶点间已有的 SE3 边，返回"终点在起点坐标系下"的测量位姿
 *
 * 边的测量值语义为 vertices()[1] 在 vertices()[0] 坐标系下的位姿：
 *   - 边方向与对话框一致（v0=起点, v1=终点）→ 直接返回测量值
 *   - 边方向相反（v0=终点, v1=起点）→ 返回测量值的逆
 *
 * @param beginNode 起点顶点（对话框参考系）
 * @param endNode   终点顶点
 * @return 测量位姿（end 在 begin 坐标系下）；两顶点间无 SE3 边时为空
 */
static std::optional<Eigen::Isometry3d> existingEdgeRelative(
    g2o::VertexSE3* beginNode, g2o::VertexSE3* endNode) {
    if (!beginNode || !endNode) return std::nullopt;
    for (auto* edge : beginNode->edges()) {
        auto* se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
        if (!se3) continue;
        const auto& verts = se3->vertices();
        if (verts.size() < 2 || !verts[0] || !verts[1]) continue;
        if (verts[0] == beginNode && verts[1] == endNode) {
            return se3->measurement();
        }
        if (verts[0] == endNode && verts[1] == beginNode) {
            return se3->measurement().inverse();
        }
    }
    return std::nullopt;
}

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

    // 起点云的 KD-Tree 只建一次：m_beginCloud 在对话框生命周期内是常量，
    // 而适应度分数是"固定 cloud1、只变 relpose"的反复求值，每次重建树纯属浪费
    // （默认子图半宽 7 → 约 19.5 万点，建树约 0.1 秒/次）。
    m_beginTree.reset(new pcl::search::KdTree<PointT>());
    m_beginTree->setInputCloud(m_beginCloud);

    // 查找关键帧获取位姿
    auto itBegin = m_graph->keyframes.find(m_beginVertexId);
    auto itEnd   = m_graph->keyframes.find(m_endVertexId);
    if (itBegin == m_graph->keyframes.end() || itEnd == m_graph->keyframes.end()) {
        QMessageBox::warning(parent, tr("Error"), tr("Keyframe not found"));
        return;
    }

    m_beginPose    = itBegin->second->estimate();
    m_endPoseInit  = itEnd->second->estimate();

    // 已有边连接时，初始预览直接采用该边的测量位姿（配准解/人工校准
    // 的结果），比两顶点当前估计的相对位姿更接近真实约束；无边时
    // 退回当前估计的相对位姿
    if (auto edgeRelative = existingEdgeRelative(itBegin->second->node,
                                                 itEnd->second->node)) {
        // 边测量值语义为 vertices()[1] 在 vertices()[0] 坐标系下的位姿；
        // 对话框以起点为参考系（relative = begin⁻¹ · end），方向不一致
        // 时求逆换算
        m_endPoseInit = m_beginPose * (*edgeRelative);
    }

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
 * ⚠️ 这里**刻意不等待**后台任务结束——这是"点取消回主界面卡一下"的根因之一。
 *
 * 两个后台任务都不持有本对象：
 *   - 扫描匹配：`[=]` 捕获的是 beginCloud / endCloud / 两个位姿的**副本**，函数体内
 *     一行成员都没用，因此连 `this` 都没有被捕获（任务里自己 new 一套配准实例）；
 *   - 适应度评分：只捕获 KD-Tree / 点云的 shared_ptr 与位姿值拷贝。
 * 两个 watcher 又都是本对象的子对象，销毁时会把自己在 future 上的 call-out 注册摘掉，
 * 所以任务算完只会把结果丢进一个已经没人收的 future。
 *
 * 反过来，如果在这里 waitForFinished()，用户点"取消"时就要**冻结 GUI 线程**：
 * 评分任务最长约 180 ms（19.5 万点实测 126~180 ms），而扫描匹配是 GICP，19.5 万点
 * 可能要好几秒——表现就是对话框关掉后主界面"卡住不动"。代价是取消后任务仍会在
 * 后台跑完（白烧几秒 CPU）并把结果丢弃，这是刻意的取舍：宁可浪费后台算力，
 * 也不阻塞界面。
 */
LoopClosureDialog::~LoopClosureDialog() {
}

// ---------------------------------------------------------------------------
// UI 设置
// ---------------------------------------------------------------------------

/**
 * @brief 创建 UI 布局
 *
 * 从上到下依次为：
 * 1. 迷你视口（512×512 预览）
 * 2. 匹配结果区域（适应度分数 + 结果日志）
 * 3. 手动调节滑块组（含步长档位选择器）
 * 4. 操作按钮行（扫描匹配、重置）
 * 5. 进度条
 * 6. 底部按钮（添加边 / 取消）
 */
void LoopClosureDialog::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);

    // 适应度分数的去抖计时器：200 ms 单次触发，重复 start() 会重新计时，
    // 因此连续点击微调按钮时只在"停手"后算一次分数（见 scheduleFitnessScore）
    m_fitnessTimer = new QTimer(this);
    m_fitnessTimer->setSingleShot(true);
    m_fitnessTimer->setInterval(200);
    connect(m_fitnessTimer, &QTimer::timeout, this,
            [this]() { updateFitnessScore(); });

    // --- 迷你视口（560×560） ---
    m_miniViewport = new MiniViewportWidget(this);
    m_miniViewport->setFixedSize(560, 560);
    mainLayout->addWidget(m_miniViewport, 0, Qt::AlignHCenter);

    // --- 匹配结果区域（适应度分数 + 结果日志合并） ---
    auto* resultGroup = new QGroupBox(tr("Match Result"));
    auto* resultLayout = new QVBoxLayout(resultGroup);

    // 适应度分数
    m_fitnessLabel = new QLabel(tr("fitness_score: —"));
    m_fitnessLabel->setObjectName("FitnessLabel");
    resultLayout->addWidget(m_fitnessLabel);

    // 结果日志（扫描匹配运行中/完成/重置提示）
    m_statusLabel = new QLabel;
    m_statusLabel->setStyleSheet("color: #999999;");
    resultLayout->addWidget(m_statusLabel);

    mainLayout->addWidget(resultGroup);

    // --- 手动调节滑块组 ---
    auto* sliderGroup = new QGroupBox(tr("Manual Adjustment (local frame)"));
    auto* sliderLayout = new QVBoxLayout(sliderGroup);

    // 步长档位选择器
    //
    // 设计要点：标签用**字面量 tr()**（lupdate 必须能提取到），而步长值随选项一起
    // 存进 item data（Qt::UserRole = 平移米，+1 = 旋转弧度）。⚠️ 不要把"下标 → 步长"
    // 写成别处的硬编码数组下标：旧实现只把下标 clamp 到 0..3，新增档位后会**静默退回
    // 1m**（界面显示选了 100m、实际按 1m 走），是这类改动最难发现的一种错。
    auto* stepRow = new QHBoxLayout;
    stepRow->addWidget(new QLabel(tr("Step:")));
    m_stepCombo = new QComboBox;
    const auto addStep = [this](const QString& label, double transStep, double rotStep) {
        const int row = m_stepCombo->count();
        m_stepCombo->addItem(label, row);
        m_stepCombo->setItemData(row, transStep, Qt::UserRole);
        m_stepCombo->setItemData(row, rotStep, Qt::UserRole + 1);
    };
    addStep(tr("Fine     — 0.01m /  0.6°"),   0.01, 0.01);
    addStep(tr("Medium   — 0.10m /  2.9°"),   0.10, 0.05);
    addStep(tr("Coarse   — 0.50m / 11.5°"),   0.50, 0.20);
    addStep(tr("Large    — 1.00m / 45.0°"),   1.00, 0.785);
    // 新增两档大步长：合并图里两条轨迹的初始相对位姿可能差几十上百米（各自的世界系
    // 原点独立），1m 一档根本够不着。用法是先用大档位把两片点云大致拉到一起，
    // 再逐级换小档位精调；旋转步长同步放大（28.6° / 57.3°）以便一次性纠掉大角度。
    addStep(tr("X-Large  — 10.0m / 28.6°"),  10.0, 0.50);
    addStep(tr("XX-Large — 100.0m / 57.3°"), 100.0, 1.00);
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
    m_scanMatchBtn = new QPushButton(tr("Scan Matching"));
    m_resetBtn     = new QPushButton(tr("Reset"));
    m_resetBtn->setObjectName("tertiaryButton");
    btnRow->addWidget(m_scanMatchBtn);
    btnRow->addWidget(m_resetBtn);
    mainLayout->addLayout(btnRow);

    connect(m_scanMatchBtn, &QPushButton::clicked, this, &LoopClosureDialog::onScanMatching);
    connect(m_resetBtn, &QPushButton::clicked, this, &LoopClosureDialog::onReset);

    // --- 进度条 ---
    m_progressBar = new QProgressBar;
    m_progressBar->setVisible(false);
    m_progressBar->setRange(0, 100);
    mainLayout->addWidget(m_progressBar);

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

    // 实时预览优先、评分去抖：预览重建是 O(N) 线性且必须立刻看到，
    // 评分是 O(N log N) 的最近邻搜索（默认约 19.5 万点，单次 0.2–0.3 秒），
    // 连续点击时不能每次都算，否则 GUI 线程被阻塞、迷你视口极卡
    updatePreview();
    scheduleFitnessScore();
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
    // 步长从选项自带的 item data 读取（Qt::UserRole = 平移米，+1 = 旋转弧度），
    // 因此档位的增删/重排都不会让"界面选了哪档"与"实际走多少"脱节。
    // 数据缺失时退回 Medium 档并保证非零——绝不静默按 0 走（那样按钮会"点了没反应"）。
    const int row = m_stepCombo->currentIndex();
    bool okTrans = false, okRot = false;
    double transStep = m_stepCombo->itemData(row, Qt::UserRole).toDouble(&okTrans);
    double rotStep   = m_stepCombo->itemData(row, Qt::UserRole + 1).toDouble(&okRot);
    if (!okTrans || !okRot || transStep <= 0.0 || rotStep <= 0.0) {
        transStep = 0.10;  // Medium 档兜底
        rotStep   = 0.05;
    }

    applySliderDelta(axis, direction * (isRotation ? rotStep : transStep), isRotation);
}

// ---------------------------------------------------------------------------
// 适应度分数
// ---------------------------------------------------------------------------

/**
 * @brief 触发一次适应度分数计算（后台线程 + 结果合并）
 *
 * 语义：算出"当前 relative 下的适应度分数"并写进标签。分数算法与自动回环
 * 判定共用同一套（逐点全量、不降采样），这里只改"在哪算"。
 *
 * 三段逻辑：
 *   1. 已有任务在跑 → 只把 m_fitnessPending 立起来后直接返回，
 *      **不排第二个任务**（否则连点 10 下排 10 个 130 ms 的任务，标签要 1.3 秒
 *      才追上，且算的都是被淘汰的旧位姿）；
 *   2. 空闲 → 用 QtConcurrent::run 起后台任务；
 *   3. 任务结束 → onFitnessScoreReady() 里判断"期间是否又调过"，是则丢弃旧结果
 *      并立刻用最新位姿再算一次（标签保持"…"），否则才更新标签。
 *
 * ⚠️ lambda 的捕获列表是这里最关键的一行：只捕获 m_beginTree / m_endCloud /
 * relative 的**拷贝**（KD-Tree 与点云都是 shared_ptr，拷贝即延长生命周期），
 * 绝不捕获 this。原因：对话框可能在计算中被销毁（用户直接关窗口），捕获 this
 * 会让后台线程读到已析构的成员——这是"看起来能跑、偶发崩溃"的典型来源。只抓
 * shared_ptr 后，任务的数据依赖与对话框的生死完全解耦，析构里那次
 * waitForFinished() 就只是让时序确定，而不是安全性的必要条件。
 */
void LoopClosureDialog::updateFitnessScore() {
    // (1) 已有计算在跑：只记下"期间又调整过"这一事实，合并掉这次请求。
    //     这里**不存**这次算出的 relative：重算时直接用那时的 m_endPose 现算，
    //     而 m_endPose 只可能比此刻更新（任何改动都会再走一次本函数），
    //     所以"现算"得到的一定不比"存下来"旧。
    //
    //     ⚠️ 用 m_fitnessWatcher 非空当"忙"标志，**不能**用 m_fitnessWatcher->
    //     isRunning()：future 在 worker 线程结束时 isRunning() 立刻变 false，而
    //     finished 回调要经 GUI 事件队列才送达。若某次调整/重置正好落在这条缝里，
    //     就会误判为"空闲"→ 排第二个任务并覆盖 m_fitnessWatcher，于是：旧 watcher
    //     的回调先去读**新** watcher 的 result()（在 GUI 线程阻塞），等新任务真正
    //     结束时 m_fitnessWatcher 已被置空 → 空指针解引用崩溃。改用"只在完成回调
    //     里才清空"的指针当标志，这条缝就不存在（与本文件 m_scanMatchRunning 同构）。
    if (m_fitnessWatcher) {
        m_fitnessPending = true;
        return;   // 标签维持"…"，由完成回调负责重算/更新
    }

    // (2) 起后台任务。捕获只按值/shared_ptr，不含 this
    // relative 语义与既有一致：终点在起点坐标系下的位姿
    const Eigen::Isometry3d relative = m_beginPose.inverse() * m_endPose;
    auto tree  = m_beginTree;
    auto cloud = m_endCloud;
    auto future = QtConcurrent::run([tree, cloud, relative]() -> double {
        return hdl_graph_slam::InformationMatrixCalculator::
            calc_fitness_score_with_tree(tree, cloud, relative, 1.0);
    });

    // watcher 以 this 为父：对话框销毁时它一起销毁。
    // 连接的对象是 watcher、上下文是 this，因此 this 生命周期内槽一定有效；
    // 而任务不在 GUI 线程、也不引用 this，两者不存在竞态。
    m_fitnessWatcher = new QFutureWatcher<double>(this);
    connect(m_fitnessWatcher, &QFutureWatcher<double>::finished,
            this, &LoopClosureDialog::onFitnessScoreReady);

    // 标签先置"…"（在 setFuture 之前！）：setFuture 对"已完成的 future"会
    // 立刻同步发出 finished，若把置位放在后面会把刚算出的分数又盖回"…"
    m_fitnessLabel->setText(tr("fitness_score: …"));
    m_fitnessWatcher->setFuture(future);
}

/**
 * @brief 适应度分数后台计算完成回调（GUI 线程）
 *
 * 合并策略的落点——保证"标签上的分数永远对应最后一次调整的位姿"：
 *   - 计算期间又调整过（m_fitnessPending）→ 这次结果对应的是**旧位姿**，
 *     直接丢弃，用当前位姿立刻再算一次，标签继续显示"…"；
 *   - 没再调整 → 更新标签。
 *
 * 丢弃比"先显示旧值再刷新"更好：旧值会因为看着像最终结果而被用户采信，
 * 而这个对话框的分数是给"要不要加这条边"做判断用的。
 */
void LoopClosureDialog::onFitnessScoreReady() {
    // 本槽是 m_fitnessWatcher 那条 finished 信号的唯一接收者，因此这里
    // m_fitnessWatcher 必然非空（它只在下面这两行被清空）
    const double score = m_fitnessWatcher->result();

    // 清空"忙"标志与 watcher（重排时 updateFitnessScore 会建新的）。
    // 这一步必须在重排之前：它同时解开了 updateFitnessScore() 的忙判断
    m_fitnessWatcher->deleteLater();
    m_fitnessWatcher = nullptr;

    if (m_fitnessPending) {
        // 结果对应旧位姿 → 丢弃，用当前位姿立刻重算（标签保持"…"）。
        // 重算走 updateFitnessScore() 现算 relative：pending 期间 m_endPose 若被
        // 改过，必然又走过一次 updateFitnessScore（本函数），因此此刻的 m_endPose
        // 就是最新位姿，不需要（也没有）额外存一份待算的相对位姿。
        m_fitnessPending = false;
        updateFitnessScore();
        return;
    }

    applyFitnessScore(score);
}

/**
 * @brief 把一次异步计算的结果写进分数标签
 *
 * clamp 到 1e6 与既有显示语义保持一致（无有效点对时算法返回 double 最大值，
 * 直接显示会是一个 1.79e308 的怪物数字）。
 */
void LoopClosureDialog::applyFitnessScore(double score) {
    score = std::min(1000000.0, score);
    m_fitnessLabel->setText(tr("fitness_score: %1").arg(score, 0, 'f', 4));
}

/**
 * @brief 适应度分数的去抖触发（见头文件说明）
 *
 * 连续点击微调按钮时，每次都要算分数的话，即使计算已经异步化，也会往线程池
 * 连排一串 130 ms 级的任务、标签长时间追不上位姿。这里改成：调整立即刷新预览
 * （廉价），分数等 200 ms 无新调整后再**触发一次**（异步，见 updateFitnessScore）。
 * 标签先置"…"提示正在重算，避免显示过期数值。
 */
void LoopClosureDialog::scheduleFitnessScore() {
    if (!m_fitnessTimer) {   // 兜底：计时器未创建时也走异步路径（不再同步阻塞 GUI）
        updateFitnessScore();
        return;
    }
    m_fitnessLabel->setText(tr("fitness_score: …"));
    m_fitnessTimer->start();  // 单次触发；重复调用会重新计时（去抖）
}

// ---------------------------------------------------------------------------
// 预览更新
// ---------------------------------------------------------------------------

void LoopClosureDialog::updatePreview() {
    m_miniViewport->updateEndPose(m_endPose, m_beginPose);
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
    if (m_scanMatchRunning) return;

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
    m_scanMatchBtn->setEnabled(true);
    m_scanMatchRunning = false;
    m_statusLabel->setText(tr("Scan matching complete"));

    // 配准结果把位姿整体换掉了：清掉"计算期间又调整过"的合并标记，
    // 避免标签被一次与配准后位姿无关的 pending 重算顶成"…"
    m_fitnessPending = false;
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
    // 同 onScanMatchFinished：位姿被整体复位，丢弃上一轮的 pending 合并标记
    m_fitnessPending = false;
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
