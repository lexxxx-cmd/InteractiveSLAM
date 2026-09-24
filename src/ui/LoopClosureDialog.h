/**
 * @file LoopClosureDialog.h
 * @brief 手动闭环确认对话框头文件
 *
 * LoopClosureDialog 是用于手动创建闭环的模态对话框。
 * 用户右键选择两个顶点后，该对话框允许预览相对位姿、调整位姿、
 * 执行扫描匹配（ICP/GICP/NDT）或手动调节，
 * 最后将闭环边提交到图优化系统中。
 *
 * 操作流程：
 *   1. 用户右键选择顶点 A → "Loop Begin"（MainWindow 存储顶点 ID）
 *   2. MainWindow 合并 A 和 B 周围相邻关键帧的点云
 *   3. 用户右键选择顶点 B → "Loop End"（MainWindow 打开此对话框）
 *   4. 对话框显示相对位姿预览、适应度分数、调整控件
 *   5. 用户可扫描匹配（ICP/GICP/NDT）或手动调节
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
#include <QTimer>
#include <memory>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/search/kdtree.h>   // m_beginTree（缓存起点云的 KD-Tree）

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
 *   5. 用户可扫描匹配（ICP/GICP/NDT）或手动调节
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
    void onScanMatching();    ///< ICP/GICP/NDT 局部配准
    void onReset();           ///< 重置位姿到初始值
    void onAddEdge();         ///< 添加闭环边到图谱

    // --- 配准线程完成 ---
    void onScanMatchFinished();     ///< 扫描匹配完成回调

    /**
     * @brief 适应度分数后台计算完成回调
     *
     * 见 updateFitnessScore() 的说明。本槽只在 GUI 线程执行，
     * 任务本身（QtConcurrent::run 的 lambda）绝不回调这里。
     */
    void onFitnessScoreReady();

private:
    /**
     * @brief 触发一次适应度分数计算（异步 + 合并，见 .cpp）
     */
    void updateFitnessScore();

    /**
     * @brief 把一次异步计算的结果写进分数标签
     * @param score 已完成任务的返回值（未 clamp）
     */
    void applyFitnessScore(double score);

    void updatePreview();       ///< 更新迷你视口预览
    void setupUi();             ///< 初始化 UI 布局

    /**
     * @brief 适应度分数的去抖触发
     *
     * 微调按钮可能被连续点击，而每次评分是 O(N log N) 的最近邻搜索
     * （默认子图半宽 7 → 每个云约 19.5 万点，**实测单次 130–180 ms**，
     * 见 tools 里的一次性测量）。因此调整时只立即刷新预览（廉价、也是用户
     * 真正需要的视觉反馈），评分改为停顿 200 ms 后再算一次。
     *
     * 去抖只解决"连点时不重复算"，不解决"算的那一次会冻结 GUI"——后者
     * 由 updateFitnessScore() 的异步化解决。两者互补，缺一不可：
     * 只异步不去抖 → 连点会往线程池塞几十个任务，标签长时间追不上；
     * 只去抖不异步 → 每次停顿仍冻结 GUI 130–180 ms。
     */
    void scheduleFitnessScore();

    /**
     * @brief 应用滑块增量
     * @param axis       轴索引（0=X, 1=Y, 2=Z）
     * @param delta      增量值
     * @param isRotation true=绕局部轴旋转, false=沿局部轴平移
     */
    void applySliderDelta(int axis, double delta, bool isRotation);

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

    /**
     * @brief 起点云的 KD-Tree（只建一次）
     *
     * 适应度分数是"固定 cloud1、变化 relpose"的反复求值，而 m_beginCloud
     * 在整个对话框生命周期内是常量（构造时传入的 ConstPtr，之后不再赋值），
     * 因此树可以缓存复用。原先每次评分都重建树，对 19.5 万点约多花 36 ms（实测）。
     *
     * ⚠️ 线程安全契约：本树在构造时建好后**只读**。后台的分数计算会把
     * shared_ptr 拷一份进任务 lambda，随后仅调用 nearestKSearch()（PCL 的
     * KdTree 在 setInputCloud 之后不再改内部状态，并发只读搜索是安全的）。
     * 同一时刻最多只有一个分数任务在跑（见 m_fitnessWatcher），因此这里
     * 连"多个任务并发读同一棵树"都不会发生。m_endCloud 同理（ConstPtr，
     * 仅被 transformPointCloud 读）。
     *
     * 将来若允许在对话框内换云（换子图 / 重新合并），必须改成：
     *   1. 换云前先把在跑的分数任务 waitForFinished() 并丢弃结果；
     *   2. 建新树后替换 m_beginTree（shared_ptr 赋值本身要加锁或用 GUI 线程
     *      信号投递，因为任务里持有的是旧 shared_ptr 的拷贝——旧树只要还有
     *      引用就不会被释放，所以"拷贝语义"已经保证了不会 use-after-free，
     *      真正的风险是标签上出现用旧云算出来的分数，需要靠 pending/版本号丢弃）。
     */
    pcl::search::KdTree<PointT>::Ptr m_beginTree;

    /// 适应度分数去抖计时器（单次触发，见 scheduleFitnessScore()）
    QTimer* m_fitnessTimer = nullptr;

    // === 适应度分数异步计算 ===
    //
    // 为什么不是"直接在 GUI 线程算"：calc_fitness_score_with_tree 是 O(N log N)
    // 的全量最近邻搜索（19.5 万点实测 130–180 ms），在 GUI 线程同步算就是
    // "每调整一下卡一下"。改成后台任务后，GUI 线程只做 setText。
    //
    // 为什么是"一个任务在跑 + 一个 pending 标记"而不是"每次调整排一个任务"：
    // 20 万点单次 130 ms 以上，连点 10 下会排 10 个任务，标签要 1.3 秒后才追上，
    // 而且算出来的都是被后来的调整淘汰的旧位姿。这里保证同时最多一个任务，
    // 期间到来的调整只把 m_fitnessPending 立起来——**不存那份 relative**：重算时
    // 直接用当时的 m_endPose 现算，而 m_endPose 只会比立标记那一刻更新，见 .cpp。
    //
    // !! 这两个字段只在 GUI 线程读写（updateFitnessScore / onFitnessScoreReady）。
    //    后台任务只持有 m_beginTree / m_endCloud / relative 的 shared_ptr 拷贝，
    //    **绝不触碰 this**（原因见 updateFitnessScore() 的长注释）。
    QFutureWatcher<double>* m_fitnessWatcher = nullptr;  ///< 当前在跑的分数计算（为空=空闲）
    bool m_fitnessPending = false;              ///< 计算期间又调整过（旧结果需丢弃重算）

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
    QPushButton* m_scanMatchBtn;         ///< 扫描匹配按钮
    QPushButton* m_resetBtn;             ///< 重置按钮
    QPushButton* m_addEdgeBtn;           ///< 添加边按钮
    QPushButton* m_cancelBtn;            ///< 取消按钮
    QProgressBar* m_progressBar;         ///< 进度条
    QLabel* m_statusLabel;               ///< 状态提示标签

    // === 线程相关 ===
    QFutureWatcher<Eigen::Isometry3d>* m_scanMatchWatcher = nullptr; ///< 扫描匹配异步结果监听器
    bool m_scanMatchRunning = false;    ///< 扫描匹配是否正在运行
};
