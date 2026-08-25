/**
 * @file AutoLoopClosurePanel.h
 * @brief 自动闭环检测控制面板（精简版）头文件
 *
 * 界面简洁化后，面板只保留"开始/停止 + 实时状态"，所有参数
 * （搜索/配准/鲁棒核/评分/优化选项）收敛到 LoopClosureParams 结构，
 * 由「高级设置 → 优化相关…」对话框读写（getParams/setParams）。
 *
 * 面板拥有一个 AutomaticLoopClosure 实例（数据层），使用 100ms
 * 定时器轮询数据层的线程安全状态快照。
 */

#pragma once

#include <QWidget>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <memory>

class GraphManager;

namespace hdl_graph_slam {
class AutomaticLoopClosure;
class InteractiveGraph;
}  // namespace hdl_graph_slam

/**
 * @brief 自动回环检测参数（UI 无关，供高级设置对话框读写）
 */
struct LoopClosureParams {
    int    searchMethod        = 0;      ///< 搜索方法（0=SEQUENTIAL, 1=RANDOM）
    double distanceThresh      = 10.0;   ///< 搜索距离阈值（米）
    double accumDistanceThresh = 15.0;   ///< 累计距离阈值（米）

    int    registrationMethod  = 1;      ///< 配准方法索引（0=ICP, 1=GICP, ...）
    int    maxIterations       = 64;     ///< 配准最大迭代次数
    double epsilon             = 1e-4;   ///< 变换收敛精度
    double resolution          = 2.0;    ///< NDT 分辨率

    int    robustKernel        = 0;      ///< 鲁棒核类型（0=NONE, ...）
    double kernelDelta         = 0.01;   ///< 核函数 delta

    double fitnessThresh       = 0.30;   ///< 配准评分阈值
    double fitnessMaxRange     = 2.00;   ///< 评分最大对应点距离

    bool   optimizeAfterInsert = true;   ///< 插入边后自动优化
};

/**
 * @brief 自动闭环检测控制面板（精简版：开始/停止 + 状态）
 */
class AutoLoopClosurePanel : public QWidget {
    Q_OBJECT

public:
    explicit AutoLoopClosurePanel(GraphManager* manager, QWidget* parent = nullptr);
    ~AutoLoopClosurePanel() override;

    /// 停止检测线程（MainWindow 在关闭地图时调用）
    void stopDetection();

    /** @brief 读取当前参数（高级设置对话框用） */
    LoopClosureParams getParams() const;
    /** @brief 写入参数（高级设置对话框确定时调用；未运行时生效） */
    void setParams(const LoopClosureParams& params);

signals:
    /// 新闭环边插入时发出 — MainWindow 刷新视口
    void loopEdgeInserted();
    /// 每次轮询发出（~10Hz）— source=蓝色, candidates=绿色 在视口中高亮
    void loopDetectionStatus(long sourceId, QVector<long> candidateIds);

private slots:
    void onStartStop();   ///< 开始/停止按钮处理
    void onPollStatus();  ///< 定时轮询状态快照

private:
    void setupUi();                        ///< 创建 UI 控件（仅开始/停止+状态）
    void syncParamsToLoop();               ///< 将 params 推送到数据层
    void updateStatusStyle(bool running);  ///< 根据运行状态更新状态标签样式

    GraphManager* m_manager;  ///< 图数据管理器（非拥有指针）

    // ---- 数据层（首次启动时延迟创建） ----
    std::unique_ptr<hdl_graph_slam::AutomaticLoopClosure> m_autoLoop;
    hdl_graph_slam::InteractiveGraph* m_loopGraph = nullptr;  ///< 数据层绑定的图指针（检测图变化）

    // ---- 参数（由高级设置对话框读写） ----
    LoopClosureParams m_params;

    // ---- 动作按钮 ----
    QPushButton* m_startStopBtn = nullptr;  ///< 开始/停止按钮

    // ---- 状态显示 ----
    QLabel* m_statusLabel = nullptr;       ///< 运行状态标签
    QLabel* m_sourceLabel = nullptr;       ///< 当前源顶点标签
    QLabel* m_candidatesLabel = nullptr;   ///< 候选顶点数标签
    QLabel* m_edgesInsertedLabel = nullptr;///< 已插入边数标签
    QLabel* m_lastMatchLabel = nullptr;    ///< 最后匹配信息标签

    // ---- 轮询 ----
    QTimer* m_pollTimer = nullptr;                ///< 状态轮询定时器（100ms）
    int m_lastKnownEdgesInserted = 0;             ///< 上次已知的边插入数（检测新边）
};
