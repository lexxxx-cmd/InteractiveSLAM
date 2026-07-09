/**
 * @file AutoLoopClosurePanel.h
 * @brief 自动闭环检测控制面板头文件
 *
 * AutoLoopClosurePanel 是用于配置和控制自动闭环检测的浮动面板。
 * 它拥有一个 AutomaticLoopClosure 实例（数据层），提供 Qt 控件用于
 * 参数配置和状态监控。使用 100ms 定时器轮询数据层的状态快照。
 *
 * 功能：
 * - 闭环搜索参数配置（搜索方法、距离阈值）
 * - 点云配准参数配置（方法、迭代次数、精度等）
 * - 鲁棒核函数配置
 * - 匹配质量阈值（适应度分数）
 * - 启动/停止检测
 * - 实时状态显示（当前源顶点、候选顶点数、已插入边数、最后匹配）
 */

#pragma once

#include <QWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <memory>

class GraphManager;

namespace hdl_graph_slam {
class AutomaticLoopClosure;
}  // namespace hdl_graph_slam

/**
 * @brief 自动闭环检测控制面板
 *
 * 拥有一个 AutomaticLoopClosure 实例（数据层），提供 Qt 控件
 * 用于参数配置和状态监控。使用 100ms QTimer 定时从数据层轮询
 * 线程安全的状态快照。
 *
 * 信号：
 * - loopEdgeInserted: 新闭环边插入时触发，MainWindow 用于刷新视口
 * - loopDetectionStatus: 每次轮询（~10Hz），包含源和候选顶点 ID
 */
class AutoLoopClosurePanel : public QWidget {
    Q_OBJECT

public:
    explicit AutoLoopClosurePanel(GraphManager* manager, QWidget* parent = nullptr);
    ~AutoLoopClosurePanel() override;

    /// 停止检测线程（MainWindow 在关闭地图时调用）
    void stopDetection();

signals:
    /// 新闭环边插入时发出 — MainWindow 刷新视口
    void loopEdgeInserted();
    /// 每次轮询发出（~10Hz）— source=蓝色, candidates=绿色 在视口中高亮
    void loopDetectionStatus(long sourceId, QVector<long> candidateIds);

private slots:
    void onStartStop();   ///< 开始/停止按钮处理
    void onPollStatus();  ///< 定时轮询状态快照

private:
    void setupUi();                        ///< 创建 UI 控件
    void syncParamsToLoop();               ///< 将 UI 控件的值推送到数据层
    void syncParamsFromLoop();             ///< 将数据层默认值拉到 UI 控件（创建时）
    void updateStatusStyle(bool running);  ///< 根据运行状态更新状态标签样式

    GraphManager* m_manager;  ///< 图数据管理器（非拥有指针）

    // ---- 数据层（首次启动时延迟创建） ----
    std::unique_ptr<hdl_graph_slam::AutomaticLoopClosure> m_autoLoop;

    // ---- 搜索参数 ----
    QComboBox*      m_searchMethodCombo;      ///< 搜索方法（SEQUENTIAL / RANDOM）
    QDoubleSpinBox* m_distanceThreshSpin;      ///< 搜索距离阈值
    QDoubleSpinBox* m_accumDistThreshSpin;     ///< 累积距离阈值

    // ---- 配准参数 ----
    QComboBox*      m_methodCombo;             ///< 配准方法（GICP/NDT 等）
    QSpinBox*       m_maxIterSpin;             ///< 最大迭代次数
    QDoubleSpinBox* m_epsSpin;                 ///< 变换精度阈值
    QDoubleSpinBox* m_resolutionSpin;          ///< NDT 分辨率

    // ---- 鲁棒核函数 ----
    QComboBox*      m_kernelCombo;             ///< 鲁棒核类型
    QDoubleSpinBox* m_kernelDeltaSpin;          ///< 核函数 delta 参数

    // ---- 适应度阈值 ----
    QDoubleSpinBox* m_fitnessThreshSpin;        ///< 适应度分数阈值
    QDoubleSpinBox* m_fitnessMaxRangeSpin;      ///< 适应度计算最大范围

    // ---- 选项 ----
    QCheckBox* m_optimizeCb;                    ///< 插入边后是否执行优化

    // ---- 动作按钮 ----
    QPushButton* m_startStopBtn;                ///< 开始/停止按钮

    // ---- 状态显示 ----
    QLabel* m_statusLabel;           ///< 运行状态标签
    QLabel* m_sourceLabel;           ///< 当前源顶点标签
    QLabel* m_candidatesLabel;       ///< 候选顶点数标签
    QLabel* m_edgesInsertedLabel;    ///< 已插入边数标签
    QLabel* m_lastMatchLabel;        ///< 最后匹配信息标签

    // ---- 轮询 ----
    QTimer* m_pollTimer;                ///< 状态轮询定时器（100ms）
    int m_lastKnownEdgesInserted = 0;  ///< 上次已知的边插入数（用于检测新边）
};
