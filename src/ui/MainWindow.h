/**
 * @file MainWindow.h
 * @brief 主窗口头文件 — 应用程序的主界面容器
 *
 * MainWindow 继承自 QMainWindow，是整个 InterSLAM 应用程序的根窗口。
 * 它持有中央 3D 视口（ViewportWidget）以及多个浮动叠加面板（OverlayPanelWidget），
 * 涵盖渲染控制、图统计、自动闭环检测、边列表管理等功能。
 * 同时负责创建菜单栏、连接 GraphManager 信号、处理右键上下文菜单等。
 */

#pragma once

#include <QMainWindow>
#include <QStatusBar>
#include <QMenuBar>
#include <QFutureWatcher>

#include "backend/project_manager.h"

class QAction;
class QLabel;
class QTimer;
class GraphManager;
class ViewportWidget;
class GraphStatsPanel;
class RenderingPanel;
class AutoLoopClosurePanel;
class EdgeListPanel;
class OverlayPanelWidget;
class PlaybackPanel;
class LoadingOverlayWidget;
// 高级设置对话框（前向声明必须位于命名空间作用域，不能写在类体内，
// 否则会变成 MainWindow 的嵌套类声明，导致不完整类型报错）
class AutoLoopClosureDialog;
class LodSettingsDialog;
class ZClipSettingsDialog;
class ColorRangeSettingsDialog;

/**
 * @brief 主应用程序窗口
 *
 * 替换原有的 Main.qml。布局结构为：中央 ViewportWidget（3D 场景渲染）
 * + 浮动叠加面板（悬浮于视口之上的控制面板）。
 *
 * 核心职责：
 * - 创建和管理菜单栏（文件、视图、图操作）
 * - 初始化各类叠加面板并注册到视口
 * - 连接 GraphManager 的信号（加载状态、日志消息等）
 * - 处理右键上下文菜单（顶点选择、手动闭环操作）
 * - 响应菜单动作（打开/关闭地图、保存、优化等）
 */
class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param manager GraphManager 实例，用于管理图谱数据
     * @param parent  父级 Qt 组件
     */
    explicit MainWindow(GraphManager* manager, QWidget* parent = nullptr);

    /**
     * @brief 从项目中心的启动任务引导主界面
     *
     * 由 main.cpp 在窗口 show() 之后调用一次：按任务动作加载地图目录
     * 或后台导入 Bag；同时更新窗口标题并回写项目导入状态。
     * Action::None（空白项目）仅更新标题。
     */
    void launchFromProject(const ProjectTask& task);

private slots:
    void onOpenProjectCenter();    ///< 打开项目中心（文件菜单）
    void onCloseMap();             ///< 关闭当前地图（文件菜单）
    void onSaveMap();              ///< 快速保存（Ctrl+S，直接存到项目数据目录/来源目录）
    void onSaveMapAs();            ///< 另存为（弹窗选择保存内容与目标目录）
    void onOptimize();             ///< 执行图优化（图菜单）
    void onResetCamera();          ///< 重置摄像机视角（视图菜单）
    void onLoadingStarted();       ///< 加载开始时的回调
    void onLoadingSucceeded();     ///< 加载成功时的回调
    void onLoadingFailed(const QString& error);  ///< 加载失败时的回调
    void onLogMessage(const QString& message);   ///< 日志消息回调

private:
    void setupMenus();  ///< 初始化菜单栏（文件、视图、图菜单）
    void setupUi();     ///< 初始化界面组件（视口、叠加面板）
    void startLoadingSpinner(const QString& text);  ///< 启动状态栏加载动画
    void stopLoadingSpinner();                      ///< 停止并隐藏加载动画
    void hideLoadingUi();  ///< 隐藏加载遮罩并停止加载动画（关图路径清理用）

    QString defaultSaveDir() const;  ///< 默认保存目录（项目数据目录，否则地图来源目录）
    /**
     * @brief 执行保存（含目录非空防御确认）
     *
     * 重活（位姿图/关键帧/全局点云写盘）放到后台线程执行，
     * 结果经 QFutureWatcher 回到 UI 线程通知状态栏。
     *
     * @return 是否启动了保存（参数校验失败/已有保存进行中/用户取消
     *         为 false；true 仅表示后台保存已启动，结果异步通知）
     */
    bool performSave(const QString& dir,
                     bool savePoseGraph, bool saveKeyframes, bool saveGlobalCloud);

    /**
     * @brief 弹出「顶点」右键菜单（帧信息 + 手动闭环起点/终点）
     *
     * 右键视锥体与播放轴面板的「当前帧菜单」共用本方法，保证两条入口
     * 的操作项与启用条件完全一致。顶点信息在此处从 GraphManager 现场
     * 查询，调用方只需给出顶点 ID。
     *
     * @param vertexId  目标顶点 ID（<0 直接返回）
     * @param globalPos 菜单弹出的全局坐标
     */
    void showVertexContextMenu(long vertexId, const QPoint& globalPos);

    GraphManager* m_manager;    ///< 图数据管理器，非拥有指针
    ViewportWidget* m_viewport; ///< 中央 3D 视口部件

    // === 叠加面板（内容部件 + 包装器） ===
    GraphStatsPanel*    m_statsPanel    = nullptr;  ///< 图统计信息面板
    RenderingPanel*     m_renderPanel   = nullptr;  ///< 渲染控制面板

    EdgeListPanel* m_edgeListPanel = nullptr;         ///< 闭环边列表面板
    PlaybackPanel* m_playbackPanel = nullptr;         ///< 播放轴面板

    // === 视图菜单动作（用于与叠加面板的显示状态同步） ===
    QAction* m_statsViewAction  = nullptr;  ///< "图统计信息"视图切换动作
    QAction* m_renderViewAction = nullptr;  ///< "渲染"视图切换动作
    QAction* m_playbackViewAction = nullptr;  ///< "播放轴"视图切换动作

    // === 叠加面板包装器（浮动于视口之上） ===
    OverlayPanelWidget* m_statsOverlay  = nullptr;  ///< 图统计悬浮面板
    OverlayPanelWidget* m_renderOverlay = nullptr;  ///< 渲染悬浮面板
    OverlayPanelWidget* m_edgeListOverlay = nullptr;  ///< 闭环边悬浮面板
    OverlayPanelWidget* m_playbackOverlay = nullptr;  ///< 播放轴悬浮面板

    // === 高级设置对话框（懒创建） ===
    AutoLoopClosureDialog*    m_autoLoopDialog = nullptr;   ///< 自动回环检测对话框
    LodSettingsDialog*        m_lodDialog = nullptr;        ///< 多级渲染(LOD)对话框
    ZClipSettingsDialog*      m_zClipDialog = nullptr;      ///< Z 轴裁剪对话框
    ColorRangeSettingsDialog* m_colorRangeDialog = nullptr; ///< 高程颜色范围对话框

    long m_loopBeginVertexId = -1;   ///< 手动闭环起点顶点 ID，-1 表示未选择
    int m_submapWindowHalfSize = 7;  ///< 闭环匹配时合并的相邻关键帧数（±N 帧）
    QAction* m_optimizeAction = nullptr;  ///< 图优化动作（用于启用/禁用状态同步）
    bool m_optimizePending = false;  ///< 优化是否正在进行中（防止重复触发）

    // === 当前项目（项目中心启动时设置，空 = 未使用项目功能） ===
    QString m_activeProjectDir;      ///< 当前项目根目录
    bool m_activeIsImport = false;   ///< 当前加载是否为 Bag 导入（决定状态回写）

    // === 加载动画（状态栏旋转字符 -\|/ ） ===
    QLabel* m_loadingSpinner = nullptr;   ///< 旋转动画标签（位于状态栏）
    QTimer* m_loadingTimer = nullptr;     ///< 旋转动画定时器
    int     m_loadingFrame = 0;           ///< 当前动画帧（0..3）
    QString m_loadingText;                ///< 加载中的基础文案（如"Loading map..."）

    LoadingOverlayWidget* m_loadingOverlay = nullptr;  ///< 全屏加载遮罩进度覆盖层

    // === 加载会话代际（防过期完成信号误关新一轮遮罩） ===
    // onLoadingStarted 时记录当前 cloudBuildSeq；cloudRenderFinished
    // 处理器只在代际一致时隐藏遮罩。遮罩可能属于保存等非加载流程
    // （-1 = 无加载会话，完成信号一律忽略隐藏动作之外的处理）。
    int m_loadSessionSeq = -1;  ///< 遮罩所属加载会话的 cloudBuildSeq，-1 = 未跟踪

    // === 异步保存 ===
    QFutureWatcher<QString> m_saveWatcher; ///< 后台保存结果监视器（结果为空串 = 成功，否则为错误信息）
    bool    m_isSaving = false;            ///< 是否已有后台保存进行中（防重入）
    QString m_lastSaveDir;                 ///< 正在保存的目标目录（完成后状态栏显示）
};
