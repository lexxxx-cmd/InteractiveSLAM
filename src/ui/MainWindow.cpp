/**
 * @file MainWindow.cpp
 * @brief 主窗口实现文件
 *
 * 实现 MainWindow 类的所有功能，包括：
 * - 界面初始化（setupUi）：创建中央视口和所有浮动面板，连接内部信号
 * - 菜单设置（setupMenus）：文件/视图/图三级菜单，绑定快捷键和动作
 * - 文件操作：打开/关闭地图、保存位姿图(.g2o)、保存地图（含 LVBA 格式）
 * - 图操作：后台线程执行 g2o 优化、重置摄像机
 * - 右键上下文菜单：顶点信息展示、手动闭环起点/终点设置、边删除
 * - 加载状态响应：状态栏消息、视口刷新
 */

#include "ui/MainWindow.h"
#include "ui/ViewportWidget.h"
#include "ui/GraphStatsPanel.h"
#include "ui/RenderingPanel.h"
#include "ui/EdgeListPanel.h"
#include "ui/OverlayPanelWidget.h"
#include "ui/PlaybackPanel.h"
#include "ui/LoopClosureDialog.h"
#include "ui/BagOpenDialog.h"
#include "ui/AutoLoopClosureDialog.h"
#include "ui/RenderingAdvancedDialogs.h"
#include "backend/graph_manager.hpp"
#include "data/hdl_graph_slam/bag_importer.hpp"

#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QAbstractButton>
#include <QMenu>
#include <QDialog>
#include <QFormLayout>
#include <QSpinBox>
#include <QLabel>
#include <QDialogButtonBox>
#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QDateTime>
#include <vector>

// ---------------------------------------------------------------------------
// 构造 / 析构
// ---------------------------------------------------------------------------

/**
 * @brief 构造函数：初始化主窗口
 *
 * 设置窗口标题和初始尺寸，创建界面布局、菜单栏，
 * 并连接 GraphManager 的信号（加载状态）以及 ViewportWidget 的信号（选中、右键菜单）。
 */
MainWindow::MainWindow(GraphManager* manager, QWidget* parent)
    : QMainWindow(parent), m_manager(manager) {

    setWindowTitle("Interactive SLAM");
    resize(1280, 720);

    setupUi();     // 创建中央视口和叠加面板
    setupMenus();  // 创建文件/视图/图菜单

    // --- 连接 GraphManager 信号 ---
    // 监听地图加载状态变化，更新状态栏和视口
    connect(m_manager, &GraphManager::loadingStarted,
            this, &MainWindow::onLoadingStarted);
    connect(m_manager, &GraphManager::loadingSucceeded,
            this, &MainWindow::onLoadingSucceeded);
    connect(m_manager, &GraphManager::loadingFailed,
            this, &MainWindow::onLoadingFailed);
    connect(m_manager, &GraphManager::lastMessageChanged,
            this, &MainWindow::onLogMessage);

    // 选中顶点的反馈：在状态栏显示顶点 ID
    connect(m_viewport, &ViewportWidget::vertexSelected,
            this, [this](long vertexId) {
        if (vertexId >= 0)
            statusBar()->showMessage(tr("Selected vertex: %1").arg(vertexId));
        else
            statusBar()->clearMessage();
    });

    // 多级渲染层级状态提示：构建完成与相机距离切换时在状态栏临时显示
    connect(m_viewport, &ViewportWidget::lodLevelChanged,
            this, [this](int level, int levelCount) {
        if (levelCount > 1) {
            statusBar()->showMessage(
                tr("渲染层级: %1/%2").arg(level).arg(levelCount), 2500);
        }
    });

    // 右键上下文菜单：显示顶点/边信息，支持手动闭环操作和边删除
    connect(m_viewport, &ViewportWidget::contextMenuRequested,
            this, [this](long vertexId, long edgeId,
                         long edgeV1, long edgeV2,
                         double edgeDist, const QString& edgeKernel,
                         QPoint pos,
                         long vtxCloudSize,
                         double vtxPosX, double vtxPosY, double vtxPosZ,
                         double vtxAccumDist, int vtxDegree) {
        if (vertexId < 0 && edgeId < 0) return;
        QMenu menu;

        // === 顶点右键菜单 ===
        if (vertexId >= 0) {
            // 顶点信息展示（只读）
            menu.addAction(tr("Vertex ID: %1").arg(vertexId))->setEnabled(false);
            menu.addAction(tr("Position: (%1, %2, %3)")
                .arg(vtxPosX, 0, 'f', 2)
                .arg(vtxPosY, 0, 'f', 2)
                .arg(vtxPosZ, 0, 'f', 2))->setEnabled(false);
            menu.addAction(tr("Distance: %1 m").arg(vtxAccumDist, 0, 'f', 2))
                ->setEnabled(false);
            menu.addAction(tr("Degree: %1 edges").arg(vtxDegree))->setEnabled(false);
            menu.addAction(tr("Cloud: %1 points").arg(vtxCloudSize))->setEnabled(false);
            menu.addSeparator();

            // --- 设置闭环起点 ---
            // 记录当前顶点 ID，状态栏提示用户右键另一个顶点作为终点
            QAction* loopBeginAction = menu.addAction(tr("Loop Begin"));
            connect(loopBeginAction, &QAction::triggered, this, [this, vertexId]() {
                m_loopBeginVertexId = vertexId;
                statusBar()->showMessage(
                    tr("Loop begin set to vertex %1. Right-click another vertex → Loop End.")
                        .arg(vertexId), 5000);
            });

            // --- 设置闭环终点并打开确认对话框 ---
            // 如果尚未选择起点，禁用该选项
            QAction* loopEndAction = menu.addAction(tr("Loop End"));
            if (m_loopBeginVertexId < 0) {
                loopEndAction->setEnabled(false);
            }
            connect(loopEndAction, &QAction::triggered, this, [this, vertexId]() {
                if (m_loopBeginVertexId < 0) return;

                auto* graph = m_manager->graph();
                if (!graph) {
                    statusBar()->showMessage(tr("No graph loaded"), 3000);
                    m_loopBeginVertexId = -1;
                    return;
                }

                // 不能与自身形成闭环
                if (m_loopBeginVertexId == vertexId) {
                    statusBar()->showMessage(tr("Cannot loop to the same vertex"), 3000);
                    m_loopBeginVertexId = -1;
                    return;
                }

                // 合并相邻关键帧的点云（用于闭环匹配）
                auto mergedBegin = mergeAdjacentClouds(graph, m_loopBeginVertexId, m_submapWindowHalfSize);
                auto mergedEnd   = mergeAdjacentClouds(graph, vertexId, m_submapWindowHalfSize);
                if (!mergedBegin || !mergedEnd ||
                    mergedBegin->empty() || mergedEnd->empty()) {
                    statusBar()->showMessage(
                        tr("Vertex has no point cloud data"), 3000);
                    m_loopBeginVertexId = -1;
                    return;
                }

                long beginId = m_loopBeginVertexId;
                m_loopBeginVertexId = -1;

                // 打开闭环确认对话框，允许用户调整相对位姿后添加闭环边
                LoopClosureDialog dlg(beginId, vertexId, m_manager,
                                      mergedBegin, mergedEnd, this);
                if (dlg.exec() == QDialog::Accepted) {
                    m_viewport->refreshScene();
                    m_viewport->rebuildPointClouds();
                    statusBar()->showMessage(
                        tr("Loop edge added: %1 → %2").arg(beginId).arg(vertexId), 5000);
                } else {
                    statusBar()->showMessage(tr("Loop closure cancelled"), 3000);
                }
            });

            menu.addSeparator();
            menu.addAction(tr("Go to Vertex"))->setEnabled(false);
            menu.addAction(tr("Vertex Details..."))->setEnabled(false);

        // === 边右键菜单 ===
        } else if (edgeId >= 0) {
            // 边信息展示（只读）
            menu.addAction(tr("Edge ID: %1").arg(edgeId))->setEnabled(false);
            menu.addAction(tr("Vertices: %1 → %2").arg(edgeV1).arg(edgeV2))
                ->setEnabled(false);
            menu.addAction(tr("Length: %1 m").arg(edgeDist, 0, 'f', 2))
                ->setEnabled(false);
            menu.addAction(tr("Kernel: %1").arg(edgeKernel))->setEnabled(false);
            menu.addSeparator();
            menu.addAction(tr("Go to Edge"))->setEnabled(false);
            menu.addAction(tr("Edge Details..."))->setEnabled(false);
            menu.addSeparator();

            // 删除边操作：从图中移除选中边并刷新视口
            QAction* deleteAction = menu.addAction(tr("Delete Edge"));
            connect(deleteAction, &QAction::triggered, this, [this, edgeId]() {
                auto* graph = m_manager->graph();
                if (!graph) return;
                if (graph->removeEdge(edgeId)) {
                    m_viewport->refreshScene();
                    statusBar()->showMessage(
                        tr("Edge %1 deleted").arg(edgeId), 3000);
                } else {
                    statusBar()->showMessage(
                        tr("Cannot delete edge %1 — optimization running, retry shortly")
                            .arg(edgeId), 3000);
                }
            });
        }
        menu.exec(pos);
    });
}

// ---------------------------------------------------------------------------
// 界面设置
// ---------------------------------------------------------------------------

/**
 * @brief 初始化 UI 组件
 *
 * 创建中央 ViewportWidget 作为 3D 渲染区域，创建 6 个浮动叠加面板
 * （图统计、渲染、Z裁剪、颜色范围、自动闭环、闭环边列表），
 * 注册到视口并默认隐藏。同时连接面板关闭按钮与视图菜单的状态同步信号。
 */
void MainWindow::setupUi() {
    // 中央视口（3D 渲染区域）
    m_viewport = new ViewportWidget(this);
    setCentralWidget(m_viewport);

    // ---- 新建浮动叠加面板（默认隐藏） ----
    // 创建面板的内容部件
    m_statsPanel   = new GraphStatsPanel(m_manager, m_viewport, nullptr);
    m_renderPanel  = new RenderingPanel(m_viewport, nullptr);

    // 将内容部件包装到可拖动的悬浮面板中
    m_statsOverlay    = new OverlayPanelWidget(tr("Graph Statistics"), m_statsPanel);
    m_renderOverlay   = new OverlayPanelWidget(tr("Rendering"), m_renderPanel);

    // 注册到视口（重新设置父级、定位、显示）
    m_viewport->registerOverlay(m_statsOverlay);
    m_viewport->registerOverlay(m_renderOverlay);

    // 图统计和渲染面板默认可见
    m_statsOverlay->show();
    m_renderOverlay->show();
    m_viewport->updateOverlayPositions();

    // 面板关闭按钮 → 隐藏面板 + 同步菜单勾选状态
    connect(m_statsOverlay, &OverlayPanelWidget::closeRequested,
            this, [this]() {
        if (m_statsOverlay) {
            m_statsOverlay->hide();
            m_viewport->updateOverlayPositions();
            if (m_statsViewAction) m_statsViewAction->setChecked(false);
        }
    });
    connect(m_renderOverlay, &OverlayPanelWidget::closeRequested,
            this, [this]() {
        if (m_renderOverlay) {
            m_renderOverlay->hide();
            m_viewport->updateOverlayPositions();
            if (m_renderViewAction) m_renderViewAction->setChecked(false);
        }
    });

    // ---- 已有的浮动叠加面板 ----
    // （自动回环已内嵌到"优化相关"设置对话框，不再作为独立浮动面板）
    m_edgeListPanel = new EdgeListPanel(m_manager, nullptr);

    m_edgeListOverlay = new OverlayPanelWidget(tr("Loop Edges"), m_edgeListPanel);

    m_viewport->registerOverlay(m_edgeListOverlay);

    // 默认隐藏（显示开关在"优化相关"设置对话框）
    m_edgeListOverlay->hide();
    m_viewport->updateOverlayPositions();

    // 面板关闭按钮 → 隐藏面板（显示开关状态由"优化相关"对话框打开时同步）
    connect(m_edgeListOverlay, &OverlayPanelWidget::closeRequested,
            this, [this]() {
        if (m_edgeListOverlay) {
            m_edgeListOverlay->hide();
            m_viewport->updateOverlayPositions();
        }
    });

    // ---- 播放轴面板 ----
    m_playbackPanel = new PlaybackPanel(m_viewport, nullptr);
    m_playbackOverlay = new OverlayPanelWidget(tr("Playback"), m_playbackPanel);
    m_playbackOverlay->setMaximumHeight(200);
    m_viewport->registerOverlay(m_playbackOverlay);
    m_playbackOverlay->hide();
    m_viewport->updateOverlayPositions();

    // 面板关闭按钮 → 同步菜单状态
    connect(m_playbackOverlay, &OverlayPanelWidget::closeRequested,
            this, [this]() {
        if (m_playbackOverlay) {
            m_playbackOverlay->hide();
            m_viewport->updateOverlayPositions();
            if (m_playbackViewAction) m_playbackViewAction->setChecked(false);
        }
    });

    // 采样步长变化 → 通知 PlaybackPanel 重建帧列表
    connect(m_viewport, &ViewportWidget::sampleStrideChanged,
            m_playbackPanel, &PlaybackPanel::onSampleStrideChanged);

    // 隐藏边变化时刷新视口
    connect(m_edgeListPanel, &EdgeListPanel::hiddenEdgesChanged,
            this, [this]() {
        m_viewport->setHiddenEdges(m_edgeListPanel->hiddenEdgeIds());
        m_viewport->refreshScene();
    });

    // 状态栏初始消息
    statusBar()->showMessage(tr("Ready — open a map folder to begin"));
}

// ---------------------------------------------------------------------------
// 菜单设置
// ---------------------------------------------------------------------------

/**
 * @brief 设置菜单栏
 *
 * 创建"文件"、"视图"、"图"三级菜单。
 * 文件菜单：打开/关闭地图、保存位姿图、保存地图、退出
 * 视图菜单：重置摄像机、正交视图切换、6 个面板的显示切换
 * 图菜单：图优化、子图合并窗口大小配置
 */
void MainWindow::setupMenus() {
    // ---- 文件菜单 ----
    auto* fileMenu = menuBar()->addMenu(tr("&File"));

    // 打开地图目录
    auto* openAction = fileMenu->addAction(tr("&Open Map..."));
    openAction->setShortcut(QKeySequence::Open);
    connect(openAction, &QAction::triggered, this, &MainWindow::onOpenMap);

    // 打开 ROS1 bag 文件（解析为地图后自动加载）
    auto* openBagAction = fileMenu->addAction(tr("Open &Bag File..."));
    openBagAction->setShortcut(QKeySequence(Qt::CTRL | Qt::ALT | Qt::Key_O));
    connect(openBagAction, &QAction::triggered, this, &MainWindow::onOpenBag);

    // 关闭当前地图
    auto* closeAction = fileMenu->addAction(tr("&Close Map"));
    closeAction->setShortcut(QKeySequence::Close);
    connect(closeAction, &QAction::triggered, this, &MainWindow::onCloseMap);

    fileMenu->addSeparator();

    // 保存位姿图为 .g2o 文件（Ctrl+S）
    auto* savePoseAction = fileMenu->addAction(tr("Save Pose Graph..."));
    savePoseAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_S));
    connect(savePoseAction, &QAction::triggered, this, &MainWindow::onSavePoseGraph);

    // 保存地图（含 LVBA 格式输出）（Ctrl+Shift+S）
    auto* saveMapAction = fileMenu->addAction(tr("Save Map..."));
    saveMapAction->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_S));
    connect(saveMapAction, &QAction::triggered, this, &MainWindow::onSaveMap);

    fileMenu->addSeparator();

    // 退出应用程序
    auto* quitAction = fileMenu->addAction(tr("&Quit"));
    quitAction->setShortcut(QKeySequence::Quit);
    connect(quitAction, &QAction::triggered, qApp, &QApplication::quit);

    // ---- 视图菜单 ----
    auto* viewMenu = menuBar()->addMenu(tr("&View"));

    // 重置摄像机视角（按 R 键）
    auto* resetCamAction = viewMenu->addAction(tr("&Reset Camera"));
    resetCamAction->setShortcut(QKeySequence(Qt::Key_R));
    connect(resetCamAction, &QAction::triggered, this, &MainWindow::onResetCamera);

    viewMenu->addSeparator();

    // 图统计面板显示切换
    m_statsViewAction = viewMenu->addAction(tr("Graph Statistics"));
    m_statsViewAction->setCheckable(true);
    m_statsViewAction->setChecked(true);  // 默认可见
    connect(m_statsViewAction, &QAction::toggled, this, [this](bool checked) {
        if (m_statsOverlay) {
            m_statsOverlay->setVisible(checked);
            m_viewport->updateOverlayPositions();
        }
    });

    // 渲染面板显示切换
    m_renderViewAction = viewMenu->addAction(tr("Rendering"));
    m_renderViewAction->setCheckable(true);
    m_renderViewAction->setChecked(true);  // 默认可见
    connect(m_renderViewAction, &QAction::toggled, this, [this](bool checked) {
        if (m_renderOverlay) {
            m_renderOverlay->setVisible(checked);
            m_viewport->updateOverlayPositions();
        }
    });

    // 播放轴面板显示切换（回环起点搜索常用入口）
    m_playbackViewAction = viewMenu->addAction(tr("Playback"));
    m_playbackViewAction->setCheckable(true);
    m_playbackViewAction->setChecked(false);
    connect(m_playbackViewAction, &QAction::toggled, this, [this](bool checked) {
        if (m_playbackOverlay) {
            m_playbackOverlay->setVisible(checked);
            m_viewport->updateOverlayPositions();
        }
    });

    // ---- 高级设置菜单（收纳不常用功能） ----
    auto* settingsMenu = menuBar()->addMenu(tr("Ad&vanced"));

    // ==================== 优化相关（子菜单） ====================
    auto* optMenu = settingsMenu->addMenu(tr("Optimization"));

    // 图优化（Ctrl+Shift+O）
    auto* optimizeAction = optMenu->addAction(tr("&Optimize"));
    optimizeAction->setShortcut(QKeySequence(Qt::CTRL | Qt::SHIFT | Qt::Key_O));
    m_optimizeAction = optimizeAction;
    connect(optimizeAction, &QAction::triggered, this, &MainWindow::onOptimize);

    optMenu->addSeparator();

    // 子图窗口大小（小弹窗，即时应用）
    auto* submapAction = optMenu->addAction(tr("Submap Window Size..."));
    connect(submapAction, &QAction::triggered, this, [this]() {
        bool ok = false;
        int half = QInputDialog::getInt(
            this, tr("Submap Merge Window"),
            tr("Merge ±N adjacent keyframes around the selected vertex\n"
               "for loop-closure matching (N=1 merges 3 frames):"),
            m_submapWindowHalfSize, 0, 10, 1, &ok);
        if (ok) {
            m_submapWindowHalfSize = half;
            m_viewport->setHighlightWindowHalf(half);
        }
    });

    // 自动回环检测（对话框：内嵌面板 + 参数）
    auto* autoLoopAction = optMenu->addAction(tr("Auto Loop Closure..."));
    connect(autoLoopAction, &QAction::triggered, this, [this]() {
        if (!m_autoLoopDialog) {
            m_autoLoopDialog = new AutoLoopClosureDialog(m_manager, this);
            // 自动回环信号（内嵌面板转发）→ 刷新视口 / 高亮
            connect(m_autoLoopDialog,
                    &AutoLoopClosureDialog::loopEdgeInserted,
                    this, [this]() {
                m_viewport->refreshScene();
                // 仅在"插入边后优化"（位姿变化）时才重建点云；
                // 否则点云世界坐标不变，重建纯属浪费（全量 VBO 上传卡顿）
                if (m_autoLoopDialog->optimizeAfterInsert()) {
                    m_viewport->rebuildPointClouds();
                }
                m_edgeListPanel->refreshList();
                statusBar()->showMessage(tr("Loop edge inserted by auto detection"), 3000);
            });
            connect(m_autoLoopDialog,
                    &AutoLoopClosureDialog::loopDetectionStatus,
                    this, [this](long sourceId, QVector<long> candidateIds) {
                std::vector<long> vec(candidateIds.begin(), candidateIds.end());
                m_viewport->setLoopHighlight(sourceId, vec);
                m_viewport->refreshScene();
            });
        }
        m_autoLoopDialog->show();
        m_autoLoopDialog->raise();
        m_autoLoopDialog->activateWindow();
    });

    // 回环边列表面板显示开关
    auto* edgeListAction = optMenu->addAction(tr("Show Loop Edges Panel"));
    edgeListAction->setCheckable(true);
    edgeListAction->setChecked(false);
    connect(edgeListAction, &QAction::toggled, this, [this](bool checked) {
        if (m_edgeListOverlay) {
            m_edgeListOverlay->setVisible(checked);
            m_viewport->updateOverlayPositions();
        }
    });

    settingsMenu->addSeparator();

    // ==================== 渲染相关（子菜单） ====================
    auto* renderMenu = settingsMenu->addMenu(tr("Rendering"));

    // 多级渲染（LOD）模式/层级
    auto* lodAction = renderMenu->addAction(tr("Multi-level Rendering (LOD)..."));
    connect(lodAction, &QAction::triggered, this, [this]() {
        if (!m_lodDialog) m_lodDialog = new LodSettingsDialog(m_viewport, this);
        m_lodDialog->show();
        m_lodDialog->raise();
        m_lodDialog->activateWindow();
    });

    // Z 轴裁剪
    auto* zClipAction = renderMenu->addAction(tr("Z-Clipping..."));
    connect(zClipAction, &QAction::triggered, this, [this]() {
        if (!m_zClipDialog) m_zClipDialog = new ZClipSettingsDialog(m_viewport, this);
        m_zClipDialog->show();
        m_zClipDialog->raise();
        m_zClipDialog->activateWindow();
    });

    // 高程颜色范围
    auto* colorRangeAction = renderMenu->addAction(tr("Elevation Color Range..."));
    connect(colorRangeAction, &QAction::triggered, this, [this]() {
        if (!m_colorRangeDialog) m_colorRangeDialog = new ColorRangeSettingsDialog(m_viewport, this);
        m_colorRangeDialog->show();
        m_colorRangeDialog->raise();
        m_colorRangeDialog->activateWindow();
    });
}

// ---------------------------------------------------------------------------
// 槽函数 — 文件操作
// ---------------------------------------------------------------------------

/**
 * @brief 打开地图目录
 *
 * 弹出目录选择对话框，调用 GraphManager 加载地图数据（.g2o 及相关点云信息）。
 */
void MainWindow::onOpenMap() {
    QString dir = QFileDialog::getExistingDirectory(
        this, tr("Open Map Directory"), QString(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    if (dir.isEmpty()) return;

    m_manager->openMapData(QUrl::fromLocalFile(dir));
}

/**
 * @brief 打开 ROS1 bag 文件（按 SCPGO 数据流解析）
 *
 * 流程：选择 .bag → 读取 bag 索引列出 topic → 弹配置对话框（topic/外参/
 * 抽稀/输出，以 config/bag_import.yaml 为默认预填）→ GraphManager::
 * openBagFile() 用弹窗返回的完整配置后台解析 → 自动加载生成的地图。
 */
void MainWindow::onOpenBag() {
    QString bagPath = QFileDialog::getOpenFileName(
        this, tr("Open ROS Bag File"), QString(),
        tr("ROS Bag (*.bag);;All Files (*)"));
    if (bagPath.isEmpty()) return;

    // 定位导入配置文件：优先可执行文件旁，其次当前工作目录，再其次用默认参数
    QString yamlPath;
    const QStringList candidates = {
        QCoreApplication::applicationDirPath() + "/config/bag_import.yaml",
        QDir::currentPath() + "/config/bag_import.yaml",
    };
    for (const auto& p : candidates) {
        if (QFileInfo::exists(p)) {
            yamlPath = p;
            break;
        }
    }
    if (yamlPath.isEmpty()) {
        QMessageBox::information(
            this, tr("Open Bag"),
            tr("Config file config/bag_import.yaml not found — using default parameters.\n"
               "Place it next to the executable or in the working directory to customize."));
    }

    // 读取 bag 索引（只读头部，速度快），列出可用 topic
    auto topicsStd = hdl_graph_slam::BagImporter::listTopics(bagPath.toStdString());
    if (topicsStd.empty()) {
        QMessageBox::warning(this, tr("Open Bag"),
                             tr("Failed to read bag or no topics found:\n%1").arg(bagPath));
        return;
    }
    QStringList topics;
    for (auto& t : topicsStd) topics << QString::fromStdString(t);

    // 对话框：以 yaml 配置为默认值预填（topic + 外参 + 抽稀 + 输出），
    // 用户可编辑全部导入参数；确认后返回完整配置供后台导入
    auto cfg = hdl_graph_slam::BagImporter::loadConfig(yamlPath.toStdString());
    BagOpenDialog dlg(topics, cfg, this);
    if (dlg.exec() != QDialog::Accepted) return;
    cfg = dlg.config();

    // 输出目录非空：残留旧地图文件会与新数据混合（如 000003/ 等残留子目录
    // 不会自动清除），导入前让用户明确选择处理方式
    if (!cfg.outputDir.empty() &&
        hdl_graph_slam::BagImporter::isOutputDirNonEmpty(cfg.outputDir)) {
        const QString dir = QString::fromStdString(cfg.outputDir);
        QMessageBox box(this);
        box.setIcon(QMessageBox::Warning);
        box.setWindowTitle(tr("Output Directory Not Empty"));
        box.setText(tr("The output directory is not empty:\n%1\n\n"
                       "Old map files may mix with the new import. "
                       "What should be done?").arg(dir));
        auto* overwriteBtn = box.addButton(tr("Overwrite (mix)"), QMessageBox::AcceptRole);
        auto* clearBtn    = box.addButton(tr("Clear & Import"), QMessageBox::DestructiveRole);
        box.addButton(QMessageBox::Cancel);
        box.exec();
        QAbstractButton* clicked = box.clickedButton();
        if (clicked == box.button(QMessageBox::Cancel)) return;
        if (clicked == clearBtn) {
            // 清空目录下所有内容（保留目录本身），失败则中止导入
            if (!hdl_graph_slam::BagImporter::clearDirectory(cfg.outputDir)) {
                QMessageBox::warning(
                    this, tr("Open Bag"),
                    tr("Failed to clear output directory:\n%1").arg(dir));
                return;
            }
        }
        // Overwrite（混合写入）：不做额外处理，直接导入
        Q_UNUSED(overwriteBtn);
    }

    m_manager->openBagFile(QUrl::fromLocalFile(bagPath), cfg);
}

/**
 * @brief 关闭当前地图
 *
 * 停止自动闭环检测，关闭地图数据，清空视口和闭环高亮，重置闭环起点。
 */
void MainWindow::onCloseMap() {
    // 停止自动闭环检测（内嵌于"自动回环检测"对话框）
    if (m_autoLoopDialog) {
        m_autoLoopDialog->stopAutoLoop();
    }

    m_manager->closeMap();
    m_viewport->onGraphClosed();
    if (m_playbackPanel) {
        m_playbackPanel->onGraphClosed();
    }
    m_viewport->setLoopHighlight(-1, {});  // 清除闭环高亮
    if (m_edgeListPanel) {
        m_edgeListPanel->clearList();
    }
    m_loopBeginVertexId = -1;
    statusBar()->showMessage(tr("Map closed"));
}

/**
 * @brief 保存位姿图为 .g2o 文件
 */
void MainWindow::onSavePoseGraph() {
    if (!m_manager->isLoaded()) {
        statusBar()->showMessage(tr("No graph loaded"), 3000);
        return;
    }

    QString path = QFileDialog::getSaveFileName(
        this, tr("Save Pose Graph"), QString(),
        tr("Pose Graph Files (*.g2o);;All Files (*)"));
    if (path.isEmpty()) return;

    try {
        m_manager->graph()->save(path.toStdString());
        statusBar()->showMessage(
            tr("Pose graph saved: %1").arg(path), 5000);
    } catch (const std::exception& e) {
        statusBar()->showMessage(
            tr("Save failed: %1").arg(e.what()), 5000);
    }
}

/**
 * @brief 保存地图（含 LVBA 格式输出 + 全局全量点云地图）
 *
 * 调用 graph->dump() 保存标准格式地图数据，
 * 并依次尝试额外导出：
 *   - LVBA 格式（saveLVBA）
 *   - 全局全量拼接点云地图（save_pointcloud → accumulated_cloud.pcd）
 * 额外格式导出失败仅记录日志，不影响主保存操作。
 */
void MainWindow::onSaveMap() {
    if (!m_manager->isLoaded()) {
        statusBar()->showMessage(tr("No graph loaded"), 3000);
        return;
    }

    QString dir = QFileDialog::getExistingDirectory(
        this, tr("Save Map Directory"), QString(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);

    if (dir.isEmpty()) return;

    try {
        auto* graph = m_manager->graph();
        graph->dump(dir.toStdString(), *m_manager->progress());

        // 尽力尝试 LVBA 格式转换 —— 失败仅记录日志，不影响主保存操作
        try {
            graph->saveLVBA(dir.toStdString(), *m_manager->progress());
        } catch (const std::exception& e) {
            std::cerr << "[MainWindow] LVBA conversion failed: "
                      << e.what() << std::endl;
        }

        // 尽力尝试保存全局全量拼接点云地图 —— 失败仅记录日志
        try {
            std::string cloudPath = dir.toStdString() + "/accumulated_cloud.pcd";
            if (!graph->save_pointcloud(cloudPath, *m_manager->progress())) {
                std::cerr << "[MainWindow] save_pointcloud returned false"
                          << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "[MainWindow] save_pointcloud failed: "
                      << e.what() << std::endl;
        }

        statusBar()->showMessage(
            tr("Map saved: %1").arg(dir), 5000);
    } catch (const std::exception& e) {
        statusBar()->showMessage(
            tr("Save failed: %1").arg(e.what()), 5000);
    }
}

// ---------------------------------------------------------------------------
// 槽函数 — 图操作
// ---------------------------------------------------------------------------

/**
 * @brief 执行图优化
 *
 * 在后台线程中运行 g2o 优化，保持 UI 响应。
 * 优化完成后通过 Qt::QueuedConnection 在主线程中刷新视口和点云。
 * 使用 m_optimizePending 标志防止重复触发。
 */
void MainWindow::onOptimize() {
    if (!m_manager->isLoaded()) {
        statusBar()->showMessage(tr("No graph loaded"));
        return;
    }

    if (m_optimizePending) {
        statusBar()->showMessage(tr("Optimization already in progress"), 3000);
        return;
    }

    m_optimizePending = true;
    if (m_optimizeAction) m_optimizeAction->setEnabled(false);
    statusBar()->showMessage(tr("Optimizing (background)..."));

    auto* graph = m_manager->graph();
    if (!graph) {
        m_optimizePending = false;
        if (m_optimizeAction) m_optimizeAction->setEnabled(true);
        return;
    }

    // 在分离线程中运行 g2o 优化，完成后通过 invokeMethod 回到主线程刷新。
    // 必须持有 optimization_mutex：与自动回环检测线程的优化互斥，避免并发
    // 修改图数据导致的数据竞争（自动回环在 automatic_loop_closure.cpp 中
    // 同样以该锁保护 optimize）。
    std::thread([this, graph]() {
        std::lock_guard<std::mutex> lock(graph->optimization_mutex);
        graph->optimize();
        QMetaObject::invokeMethod(this, [this]() {
            m_optimizePending = false;
            if (m_optimizeAction) m_optimizeAction->setEnabled(true);
            m_viewport->refreshScene();
            m_viewport->rebuildPointClouds();
            statusBar()->showMessage(tr("Optimization complete"), 3000);
        }, Qt::QueuedConnection);
    }).detach();
}

/**
 * @brief 重置摄像机视角到初始位置
 *
 * 调用 ViewportWidget::resetCamera() 将视口摄像机回到初始位置。
 */
void MainWindow::onResetCamera() {
    m_viewport->resetCamera();
}

// ---------------------------------------------------------------------------
// 槽函数 — 加载状态
// ---------------------------------------------------------------------------

/**
 * @brief 加载开始时的回调
 *
 * 在状态栏显示"正在加载地图..."提示。
 */
void MainWindow::onLoadingStarted() {
    statusBar()->showMessage(tr("Loading map..."));
}

/**
 * @brief 加载成功时的回调
 *
 * 更新状态栏显示顶点/边/关键帧数量，通知视口加载图谱，
 * 设置子图高亮窗口半宽，刷新闭环边列表。
 */
void MainWindow::onLoadingSucceeded() {
    m_loopBeginVertexId = -1;
    statusBar()->showMessage(
        tr("Map loaded — %1 vertices, %2 edges, %3 keyframes")
            .arg(m_manager->vertexCount())
            .arg(m_manager->edgeCount())
            .arg(m_manager->keyframeCount()),
        5000);

    m_viewport->onGraphLoaded(m_manager->sharedGraph());
    m_viewport->setHighlightWindowHalf(m_submapWindowHalfSize);
    if (m_edgeListPanel) {
        m_edgeListPanel->refreshList();
    }

    // 向播放轴面板传递排序后的关键帧 ID 列表
    auto graph = m_manager->sharedGraph();
    if (graph && m_playbackPanel) {
        std::vector<long> ids;
        ids.reserve(graph->keyframes.size());
        for (auto& [id, _] : graph->keyframes) {
            ids.push_back(id);
        }
        std::sort(ids.begin(), ids.end());
        m_playbackPanel->setKeyframeIds(ids);
    }
}

/**
 * @brief 加载失败时的回调
 *
 * 在状态栏和消息框中显示错误信息。
 */
void MainWindow::onLoadingFailed(const QString& error) {
    statusBar()->showMessage(tr("Loading failed: %1").arg(error));
    QMessageBox::warning(this, tr("Load Error"), error);
}

/**
 * @brief 日志消息回调
 *
 * 在状态栏显示来自 GraphManager 的日志消息，5 秒后自动消失。
 */
void MainWindow::onLogMessage(const QString& message) {
    statusBar()->showMessage(message, 5000);
}
