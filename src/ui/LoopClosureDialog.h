/**
 * @file LoopClosureDialog.h
 * @brief 手动闭环确认对话框头文件
 *
 * LoopClosureDialog 是用于手动创建闭环的模态对话框。
 * 用户右键选择两个顶点后，该对话框允许预览相对位姿、调整位姿、
 * 执行自动对齐（FPFH）或扫描匹配（ICP/GICP/NDT），
 * 最后将闭环边提交到图优化系统中。
 *
 * 操作流程：
 *   1. 用户右键选择顶点 A → "Loop Begin"（MainWindow 存储顶点 ID）
 *   2. MainWindow 合并 A 和 B 周围相邻关键帧的点云
 *   3. 用户右键选择顶点 B → "Loop End"（MainWindow 打开此对话框）
 *   4. 对话框显示相对位姿预览、适应度分数、调整控件
 *   5. 用户可自动对齐（FPFH）、扫描匹配（ICP/GICP/NDT）或手动调节
 *   6. "添加边" 提交相对位姿到图谱；"取消" 放弃操作
 */

#pragma once

#include <QDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QFutureWatcher>
#include <atomic>
#include <memory>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class MiniViewportWidget;
class GraphManager;

namespace hdl_graph_slam {
class RegistrationMethods;
class InteractiveGraph;
}  // namespace hdl_graph_slam

/**
 * @brief 合并与 @p centerId 相邻关键帧的点云到 @p centerId 的局部坐标系
 *
 * @param graph          交互图谱
 * @param centerId       中心顶点 ID
 * @param windowHalfSize 每侧合并的关键帧数量。
 *                        N 将合并 centerId-N 到 centerId+N（最多 2N+1 帧）。
 *                        默认 1（后向兼容：最多 3 帧）。
 *                        不存在的关键帧或空点云被静默跳过。
 * @return 合并后的点云
 */
pcl::PointCloud<pcl::PointXYZI>::Ptr mergeAdjacentClouds(
    const hdl_graph_slam::InteractiveGraph* graph, long centerId,
    int windowHalfSize = 1);

/**
 * @brief 手动闭环确认对话框
 *
 * 工作流程：
 *   1. 用户右键选择顶点 A → "Loop Begin"（MainWindow 存储顶点 ID）
 *   2. MainWindow 合并 A 和 B 周围相邻关键帧的点云
 *   3. 用户右键选择顶点 B → "Loop End"（MainWindow 打开此对话框）
 *   4. 对话框显示相对位姿预览、适应度分数、调整控件
 *   5. 用户可自动对齐（FPFH）、扫描匹配（ICP/GICP/NDT）或手动调节
 *   6. "添加边" 提交相对位姿到图谱；"取消" 放弃操作
 *
 * @param beginCloud  起点顶点的合并点云（关键帧局部坐标）
 * @param endCloud    终点顶点的合并点云（关键帧局部坐标）
 */
class LoopClosureDialog : public QDialog {
    Q_OBJECT
public:
    using PointT = pcl::PointXYZI;
    using CloudPtr = pcl::PointCloud<PointT>::ConstPtr;

    /**
     * @brief 构造函数
     * @param beginVertexId   "闭环起点"顶点 ID
     * @param endVertexId     "闭环终点"顶点 ID
     * @param manager         GraphManager 用于访问图谱
     * @param beginCloud      起点顶点的合并点云（关键帧局部坐标）
     * @param endCloud        终点顶点的合并点云（关键帧局部坐标）
     * @param parent          父级部件
     */
    LoopClosureDialog(long beginVertexId, long endVertexId,
                      GraphManager* manager,
                      CloudPtr beginCloud, CloudPtr endCloud,
                      QWidget* parent = nullptr);
    ~LoopClosureDialog() override;

private slots:
    // --- 步进按钮增量应用 ---
    void onStepButton(int axis, bool isRotation, int direction);  ///< ◀/▶ 步进按钮

    // --- 按钮 ---
    void onAutoAlign();       ///< FPFH 全局配准
    void onScanMatching();    ///< ICP/GICP/NDT 局部配准
    void onReset();           ///< 重置位姿到初始值
    void onAddEdge();         ///< 添加闭环边到图谱

    // --- 配准线程完成 ---
    void onFpfhAlignFinished();     ///< FPFH 配准完成回调
    void onScanMatchFinished();     ///< 扫描匹配完成回调

private:
    void updateFitnessScore();  ///< 更新适应度分数显示
    void updatePreview();       ///< 更新迷你视口预览
    void setupUi();             ///< 初始化 UI 布局

    /**
     * @brief 应用滑块增量
     * @param axis       轴索引（0=X, 1=Y, 2=Z）
     * @param delta      增量值
     * @param isRotation true=绕局部轴旋转, false=沿局部轴平移
     */
    void applySliderDelta(int axis, double delta, bool isRotation);

    // FPFH 自动对齐（后台线程运行）
    void runFpfhAlign(double normalRadius, double searchRadius,
                      int maxIter, int numSamples, int corrRandomness,
                      double similarityThresh, double maxCorrDist, double inlierFrac);

    // 扫描匹配（后台线程运行）
    void runScanMatching(int methodIndex, int maxIterations,
                         float transEpsilon, float resolution);

    // === 数据 ===
    GraphManager* m_manager;            ///< 图管理器
    hdl_graph_slam::InteractiveGraph* m_graph;  ///< 交互图谱

    long m_beginVertexId;  ///< 闭环起点顶点 ID
    long m_endVertexId;    ///< 闭环终点顶点 ID

    CloudPtr m_beginCloud;  ///< 起点合并点云
    CloudPtr m_endCloud;    ///< 终点合并点云

    Eigen::Isometry3d m_beginPose;      ///< 起点位姿（对话框打开时固定）
    Eigen::Isometry3d m_endPoseInit;    ///< 终点初始位姿（对话框打开时固定）
    Eigen::Isometry3d m_endPose;        ///< 终点当前位姿（可被滑块/配准修改）

    std::unique_ptr<hdl_graph_slam::RegistrationMethods> m_regMethods;  ///< 配准方法工厂

    // === UI 控件 ===
    MiniViewportWidget* m_miniViewport;  ///< 迷你 3D 视口预览
    QLabel* m_fitnessLabel;              ///< 适应度分数标签
    QComboBox* m_stepCombo;              ///< 步长档位选择器
    // +/- 按钮数组：[axis][0]=减按钮, [axis][1]=加按钮
    QPushButton* m_stepBtns[6][2];       ///< 六轴步进按钮：PX, PY, PZ, RX, RY, RZ
    QPushButton* m_autoAlignBtn;         ///< 自动对齐按钮
    QPushButton* m_scanMatchBtn;         ///< 扫描匹配按钮
    QPushButton* m_resetBtn;             ///< 重置按钮
    QPushButton* m_addEdgeBtn;           ///< 添加边按钮
    QPushButton* m_cancelBtn;            ///< 取消按钮
    QProgressBar* m_progressBar;         ///< 进度条
    QLabel* m_statusLabel;               ///< 状态提示标签

    // === 线程相关 ===
    QFutureWatcher<Eigen::Isometry3d>* m_fpfhWatcher = nullptr;     ///< FPFH 异步结果监听器
    std::atomic_int m_fpfhProgress{0};  ///< FPFH 进度（0-5，用于进度条阶段）
    bool m_fpfhRunning = false;         ///< FPFH 是否正在运行

    QFutureWatcher<Eigen::Isometry3d>* m_scanMatchWatcher = nullptr; ///< 扫描匹配异步结果监听器
    bool m_scanMatchRunning = false;    ///< 扫描匹配是否正在运行
};
