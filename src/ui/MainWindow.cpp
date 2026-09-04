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
#include "ui/ProjectCenterDialog.h"
#include "ui/ViewportWidget.h"
#include "ui/GraphStatsPanel.h"
#include "ui/RenderingPanel.h"
#include "ui/EdgeListPanel.h"
#include "ui/OverlayPanelWidget.h"
#include "ui/PlaybackPanel.h"
#include "ui/LoopClosureDialog.h"
#include "ui/AutoLoopClosureDialog.h"
#include "ui/RenderingAdvancedDialogs.h"
#include "ui/SaveMapDialog.h"
#include "backend/graph_manager.hpp"
#include "data/hdl_graph_slam/bag_importer.hpp"

#include <QFileDialog>
#include <QInputDialog>
#include <QMessageBox>
#include <QSettings>
#include <QMenu>
#include <QDialog>
#include <QFormLayout>
#include <QSpinBox>
#include <QLabel>
#include <QTimer>
#include <QDialogButtonBox>
#include <QApplication>
#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QDateTime>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrentRun>
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

    // 点云全部渲染完成后停止加载动画（spinner 持续到 LOD 分块上传完毕，
    // 而非数据加载完成就消失）
    connect(m_viewport, &ViewportWidget::cloudRenderFinished,
            this, &MainWindow::stopLoadingSpinner);

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

    // 第一人称模式提示：操作方式在状态栏显示
    connect(m_viewport, &ViewportWidget::firstPersonModeChanged,
            this, [this](bool active) {
        statusBar()->showMessage(active
            ? tr("First-person mode: drag to look, W/A/S/D to walk, "
                 "Q up, E down, wheel adjusts speed, Shift to exit")
            : tr("First-person mode off"), 5000);
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

    // ---- 播放轴面板（默认显示） ----
    m_playbackPanel = new PlaybackPanel(m_viewport, nullptr);
    m_playbackOverlay = new OverlayPanelWidget(tr("Playback"), m_playbackPanel);
    m_playbackOverlay->setMaximumHeight(200);
    m_viewport->registerOverlay(m_playbackOverlay);
    m_playbackOverlay->show();
    m_viewport->updateOverlayPositions();

    // 面板关闭按钮 → 同步菜单状态
    connect(m_playbackOverlay, &OverlayPanelWidget::closeRequested,
            this, [this]() {
        if (m_playbackOverlay) {
            m_playbackOverlay->hide();
            m_viewport->updateOverlayPositions();
            if (m_playbackViewAction) m_playbackViewAction->setChecked(false);
            // 关闭面板 = 播放会话结束：停止播放并清理播放高亮
            if (m_playbackPanel) m_playbackPanel->pausePlayback();
            m_viewport->highlightPlaybackVertex(-1);
        }
    });

    // 采样步长变化 → 通知 PlaybackPanel 重建帧列表
    connect(m_viewport, &ViewportWidget::sampleStrideChanged,
            m_playbackPanel, &PlaybackPanel::onSampleStrideChanged);

    // 任何选中（Ctrl+Click / 程序化 selectVertex）→ 暂停播放轴并清理
    // 播放通道高亮：用户主动选择时播放让位，避免两个红色标记并存。
    // 面板自身的停下路径已先清理再选中，此处为幂等兜底。
    connect(m_viewport, &ViewportWidget::vertexSelected, this, [this](long) {
        if (m_playbackPanel) m_playbackPanel->pausePlayback();
        m_viewport->highlightPlaybackVertex(-1);
    });

    // 隐藏边变化时刷新视口
    connect(m_edgeListPanel, &EdgeListPanel::hiddenEdgesChanged,
            this, [this]() {
        m_viewport->setHiddenEdges(m_edgeListPanel->hiddenEdgeIds());
        m_viewport->refreshScene();
    });

    // 状态栏初始消息
    statusBar()->showMessage(tr("Ready — open a map folder to begin"));

    // 加载动画：状态栏右侧的旋转字符指示器（加载地图/bag 时显示）
    m_loadingSpinner = new QLabel(this);
    m_loadingSpinner->hide();
    statusBar()->addPermanentWidget(m_loadingSpinner);

    m_loadingTimer = new QTimer(this);
    m_loadingTimer->setInterval(120);  // ~8.3 fps，旋转平滑
    connect(m_loadingTimer, &QTimer::timeout, this, [this]() {
        // 旋转字符序列：| / - \ 循环
        static const char kFrames[] = {'|', '/', '-', '\\'};
        m_loadingSpinner->setText(
            QString(" %1 %2").arg(QChar(kFrames[m_loadingFrame]))
                             .arg(m_loadingText));
        m_loadingFrame = (m_loadingFrame + 1) % 4;
    });

    // 异步保存完成 → 停止加载动画并通知结果
    //（QFutureWatcher::finished 在启动 watcher 的线程回调，即 UI 线程）
    connect(&m_saveWatcher, &QFutureWatcher<QString>::finished, this, [this]() {
        stopLoadingSpinner();
        m_isSaving = false;
        const QString err = m_saveWatcher.result();
        if (err.isEmpty())
            statusBar()->showMessage(tr("Map saved: %1").arg(m_lastSaveDir), 5000);
        else
            statusBar()->showMessage(tr("Save failed: %1").arg(err), 5000);
    });
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

    // 打开项目中心（新建/切换项目；加载前会先关闭当前地图）
    auto* openProjectAction = fileMenu->addAction(tr("Open &Project..."));
    connect(openProjectAction, &QAction::triggered, this, &MainWindow::onOpenProjectCenter);

    // 打开地图目录
    // 注意：文件菜单的 Ctrl 系快捷键已全部移除（第一人称模式 Ctrl=下降，
    // 按住 Ctrl 行走时会误触发 Ctrl+W 关地图 / Ctrl+Q 退出程序等）
    auto* openAction = fileMenu->addAction(tr("&Open Map..."));
    connect(openAction, &QAction::triggered, this, &MainWindow::onOpenMap);

    // 关闭当前地图
    auto* closeAction = fileMenu->addAction(tr("&Close Map"));
    connect(closeAction, &QAction::triggered, this, &MainWindow::onCloseMap);

    fileMenu->addSeparator();

    // 快速保存：直接写入项目数据目录/地图来源目录（不弹窗）
    auto* saveAction = fileMenu->addAction(tr("&Save"));
    connect(saveAction, &QAction::triggered, this, &MainWindow::onSaveMap);

    // 另存为：弹窗选择保存内容与目标目录
    auto* saveMapAction = fileMenu->addAction(tr("Save Map &As..."));
    connect(saveMapAction, &QAction::triggered, this, &MainWindow::onSaveMapAs);

    fileMenu->addSeparator();

    // 退出应用程序
    auto* quitAction = fileMenu->addAction(tr("&Quit"));
    connect(quitAction, &QAction::triggered, qApp, &QApplication::quit);

    // ---- 视图菜单 ----
    auto* viewMenu = menuBar()->addMenu(tr("&View"));

    // 重置摄像机视角
    // 注意：不设单键 R 快捷键——R 紧邻 WASD，第一人称行走时易误触
    // （重置相机同时会退出第一人称模式）
    auto* resetCamAction = viewMenu->addAction(tr("&Reset Camera"));
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
    m_playbackViewAction->setChecked(true);  // 默认显示
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

    // 图优化（不设快捷键：历史上与"打开项目"快捷键重复，
    // 且需避免与 WASD 行走键相互干扰）
    auto* optimizeAction = optMenu->addAction(tr("&Optimize"));
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
 * @brief 默认保存目录
 *
 * 项目模式下为项目数据目录（与加载来源一致，保存 = 写回）；
 * 非项目模式为当前地图的来源目录。两者都为空（理论上不发生）返回空串。
 */
QString MainWindow::defaultSaveDir() const {
    if (!m_activeProjectDir.isEmpty()) {
        auto info = ProjectManager::read(m_activeProjectDir);
        if (info.valid) return info.resolvedDataDir();
    }
    return m_manager->mapSourceDir();
}

/**
 * @brief 执行保存（统一入口）
 *
 * 参数校验与"目录非空"防御确认在 UI 线程同步完成；位姿图/关键帧/
 * 全局点云的写盘重活放到后台线程（与 bag 导入同一 QtConcurrent 模式），
 * 完成后经 QFutureWatcher 回 UI 线程通知状态栏，期间不阻塞界面。
 *
 * 1) 位姿图 → graph.g2o；2) 每帧点云及 data → 标准地图目录；
 * 3) 全局点云 → accumulated_cloud.pcd。各项按开关独立执行。
 * （LVBA/all_pcd_body 导出已停用：无下游使用；saveLVBA 函数保留）
 *
 * 防御：目标目录既不是默认保存目录又非空时，弹窗确认"清空并保存"，
 * 防止与旧地图文件混存（同 bag 导入的清空语义）。
 *
 * @return 是否启动了保存（参数校验失败/已有保存进行中/用户取消为 false）
 */
bool MainWindow::performSave(const QString& dir,
                             bool savePoseGraph, bool saveKeyframes,
                             bool saveGlobalCloud) {
    if (dir.isEmpty()) {
        statusBar()->showMessage(tr("No output directory selected"), 3000);
        return false;
    }
    if (!savePoseGraph && !saveKeyframes && !saveGlobalCloud) {
        statusBar()->showMessage(tr("Nothing selected to save"), 3000);
        return false;
    }
    if (m_isSaving) {
        statusBar()->showMessage(tr("Saving already in progress"), 3000);
        return false;
    }

    // 防御确认：非默认目录且非空 → 清空确认（避免新旧地图文件混杂）
    if (dir != defaultSaveDir() &&
        hdl_graph_slam::BagImporter::isOutputDirNonEmpty(dir.toStdString())) {
        QMessageBox box(this);
        box.setIcon(QMessageBox::Warning);
        box.setWindowTitle(tr("Output Directory Not Empty"));
        box.setText(tr("The output directory is not empty:\n%1\n\n"
                       "Saving will permanently delete all existing content "
                       "in this directory (cannot be undone).").arg(dir));
        auto* clearBtn = box.addButton(tr("Clear & Save"), QMessageBox::DestructiveRole);
        box.addButton(QMessageBox::Cancel);
        box.exec();
        if (box.clickedButton() != clearBtn) return false;
        if (!hdl_graph_slam::BagImporter::clearDirectory(dir.toStdString())) {
            QMessageBox::warning(this, tr("Clear Failed"),
                                 tr("Failed to clear the output directory:\n%1").arg(dir));
            return false;
        }
    }

    // 记住本次内容勾选（下次快速保存/另存为复用）
    QSettings settings("DAFTECH", "InteractiveSLAM");
    settings.setValue("save_map/pose_graph", savePoseGraph);
    settings.setValue("save_map/keyframes", saveKeyframes);
    settings.setValue("save_map/global_cloud", saveGlobalCloud);

    m_isSaving = true;
    m_lastSaveDir = dir;
    startLoadingSpinner(tr("Saving..."));

    auto* graph = m_manager->graph();
    auto* progress = m_manager->progress();
    const std::string dirStd = dir.toStdString();
    m_saveWatcher.setFuture(QtConcurrent::run(
        [graph, progress, dirStd, savePoseGraph, saveKeyframes, saveGlobalCloud]()
            -> QString {
            try {
                // 1) 保存位姿图（graph.g2o，固定位于地图目录根，与单帧目录同层）
                if (savePoseGraph) {
                    graph->save(dirStd + "/graph.g2o");
                }

                // 2) 保存每帧点云及 data 文件（标准地图目录）
                //    （LVBA/all_pcd_body 导出已注释：无下游使用；saveLVBA 函数保留）
                // if (saveKeyframes) {
                //     graph->saveLVBA(dirStd, *progress);
                // }
                if (saveKeyframes) {
                    graph->dump(dirStd, *progress);
                }

                // 3) 保存全局全量拼接点云地图（accumulated_cloud.pcd）
                if (saveGlobalCloud &&
                    !graph->save_pointcloud(dirStd + "/accumulated_cloud.pcd",
                                            *progress)) {
                    std::cerr << "[MainWindow] save_pointcloud returned false"
                              << std::endl;
                }

                return {};  // 空串 = 成功
            } catch (const std::exception& e) {
                return QString::fromUtf8(e.what());
            }
        }));
    return true;
}

/**
 * @brief 快速保存（Ctrl+S）
 *
 * 不弹窗：直接保存到默认保存目录（项目数据目录 / 地图来源目录），
 * 内容勾选复用上次保存的配置（QSettings）。没有可用目录时退化为另存为。
 */
void MainWindow::onSaveMap() {
    if (!m_manager->isLoaded()) {
        statusBar()->showMessage(tr("No graph loaded"), 3000);
        return;
    }
    if (m_isSaving) {
        statusBar()->showMessage(tr("Saving already in progress"), 3000);
        return;
    }

    QString dir = defaultSaveDir();
    if (dir.isEmpty()) {
        onSaveMapAs();
        return;
    }

    QSettings settings("DAFTECH", "InteractiveSLAM");
    performSave(dir,
                settings.value("save_map/pose_graph", true).toBool(),
                settings.value("save_map/keyframes", true).toBool(),
                settings.value("save_map/global_cloud", true).toBool());
}

/**
 * @brief 另存为（弹窗选择保存内容与目标目录）
 *
 * SaveMapDialog 预填默认保存目录并恢复上次勾选；用户可改目录
 * （导出副本）或改勾选。
 */
void MainWindow::onSaveMapAs() {
    if (!m_manager->isLoaded()) {
        statusBar()->showMessage(tr("No graph loaded"), 3000);
        return;
    }
    if (m_isSaving) {
        statusBar()->showMessage(tr("Saving already in progress"), 3000);
        return;
    }

    SaveMapDialog dlg(this);
    dlg.setOutputDirectory(defaultSaveDir());
    if (dlg.exec() != QDialog::Accepted) return;
    performSave(dlg.outputDirectory(),
                dlg.savePoseGraph(), dlg.saveKeyframes(), dlg.saveGlobalCloud());
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
    // 通用加载文案（地图加载与 bag 导入共用）
    startLoadingSpinner(tr("Loading..."));
}

/**
 * @brief 启动状态栏加载动画（旋转字符 | / - \）
 *
 * 在状态栏右侧显示"字符 + 文案"，由 120ms 定时器驱动字符旋转，
 * 直到加载成功/失败回调停止。
 */
void MainWindow::startLoadingSpinner(const QString& text) {
    m_loadingText = text;
    m_loadingFrame = 0;
    m_loadingSpinner->setText(QString(" | %1").arg(text));
    m_loadingSpinner->show();
    m_loadingTimer->start();
}

/** @brief 停止并隐藏加载动画 */
void MainWindow::stopLoadingSpinner() {
    m_loadingTimer->stop();
    m_loadingSpinner->hide();
}

/**
 * @brief 加载成功时的回调
 *
 * 更新状态栏显示顶点/边/关键帧数量，通知视口加载图谱，
 * 设置子图高亮窗口半宽，刷新闭环边列表。
 */
void MainWindow::onLoadingSucceeded() {
    // 注意：此处不停止加载动画——点云（含 LOD 分块）仍在后台构建/渐进上传，
    // spinner 由 cloudRenderFinished 信号在点云全部渲染完成后停止
    m_loopBeginVertexId = -1;
    // Bag 导入随加载一并成功 → 回写项目状态 completed
    if (m_activeIsImport && !m_activeProjectDir.isEmpty()) {
        ProjectManager::setStatus(m_activeProjectDir, "completed");
        m_activeIsImport = false;
    }
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
    stopLoadingSpinner();
    // Bag 导入失败 → 回写项目状态 failed（保留 processing 前的状态信息）
    if (m_activeIsImport && !m_activeProjectDir.isEmpty()) {
        ProjectManager::setStatus(m_activeProjectDir, "failed");
        m_activeIsImport = false;
    }
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

// ---------------------------------------------------------------------------
// 项目集成
// ---------------------------------------------------------------------------

/**
 * @brief 从项目中心的启动任务引导主界面
 *
 * 由 main.cpp 在窗口 show() 之后调用一次。Action 分派：
 *   - LoadDirectory：openMapData() 加载项目数据目录；
 *   - ImportBag：    openBagFile() 后台导入（配置已在项目中心完成，
 *                    清空确认也已处理），状态回写挂在加载成功/失败回调；
 *   - None：         空白项目，仅更新标题。
 */
void MainWindow::launchFromProject(const ProjectTask& task) {
    m_activeProjectDir = task.projectDir;
    m_activeIsImport = (task.action == ProjectTask::Action::ImportBag);

    if (!task.projectName.isEmpty()) {
        setWindowTitle(tr("Interactive SLAM — %1").arg(task.projectName));
    }

    switch (task.action) {
    case ProjectTask::Action::LoadDirectory:
        statusBar()->showMessage(tr("Opening project \"%1\"...").arg(task.projectName));
        m_manager->openMapData(QUrl::fromLocalFile(task.dataDir));
        break;
    case ProjectTask::Action::ImportBag:
        statusBar()->showMessage(tr("Importing Bag into project \"%1\"...").arg(task.projectName));
        m_manager->openBagFile(QUrl::fromLocalFile(task.bagPath), task.importCfg);
        break;
    case ProjectTask::Action::None:
    default:
        statusBar()->showMessage(
            tr("Project \"%1\" opened (blank project, import data later)").arg(task.projectName),
            5000);
        break;
    }
}

/**
 * @brief 打开项目中心（文件菜单）
 *
 * 重新弹出项目中心；用户选定新项目后关闭当前地图并按新任务引导界面。
 * 取消则留在当前项目/状态。
 */
void MainWindow::onOpenProjectCenter() {
    ProjectCenterDialog dlg(this);
    if (dlg.exec() != QDialog::Accepted) return;

    if (m_manager->isLoaded()) {
        m_manager->closeMap();
    }
    setWindowTitle("Interactive SLAM");  // launchFromProject 会按需设置项目名
    m_activeProjectDir.clear();
    m_activeIsImport = false;
    launchFromProject(dlg.task());
}
