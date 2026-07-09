/**
 * @file automatic_loop_closure.hpp
 * @brief 自动闭环检测模块定义
 *
 * AutomaticLoopClosure 是一个纯算法层的自动闭环检测模块，
 * 在后台线程中运行，通过扫描匹配验证闭环候选并插入闭环边。
 *
 * 核心功能：
 *   1. 在后台线程中遍历关键帧
 *   2. 通过 BFS（广度优先搜索）沿 g2o 边查找闭环候选
 *   3. 使用点云配准（ICP/GICP/NDT 等）验证候选
 *   4. 成功匹配后插入闭环边
 *   5. 可选在插入后执行全局图优化
 *
 * 设计特点：
 *   - 无 Qt / ImGui 依赖，纯 C++ 算法层
 *   - 通过 Status 结构体向 UI 层提供线程安全的状态快照
 *   - 配置参数可通过 setter 方法在运行前/运行时设置
 */
#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include <vector>
#include <unordered_map>
#include <unordered_set>
#include <deque>

#include <Eigen/Dense>

#include "data/hdl_graph_slam/registration_methods.hpp"

namespace hdl_graph_slam {

class InteractiveGraph;      ///< 交互式图（前向声明）
class InteractiveKeyFrame;   ///< 交互式关键帧（前向声明）

/**
 * @brief 自动闭环检测 —— 纯算法层实现
 *
 * 在后台 std::thread 中运行，执行以下流程：
 *   1. 遍历关键帧（顺序模式或随机模式）
 *   2. 对每个源关键帧，沿 g2o 边进行 BFS 搜索，计算图距离
 *   3. 过滤出距离足够远且空间上接近的候选者
 *   4. 使用点云配准验证候选者的匹配质量
 *   5. 对匹配成功的候选插入闭环边
 *   6. 可选执行全局图优化
 *
 * 线程安全设计：
 *   - m_running：std::atomic_bool 原子布尔值控制线程启停
 *   - m_status_mutex：保护 Status 结构体，供 Qt UI 轮询读取
 *   - graph->optimization_mutex：在 optimize() 调用时加锁
 *   - graph->add_edge()：唯一写入者（仅本线程写入）
 */
class AutomaticLoopClosure {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  ///< Eigen 内存对齐支持

    /**
     * @brief 线程安全的状态快照，供 Qt UI 轮询定时器消费
     */
    struct Status {
        bool running = false;               ///< 检测器是否正在运行
        long  current_source_id = -1;        ///< 当前处理的源关键帧 ID
        std::vector<long> candidate_ids;     ///< 当前源找到的候选者 ID 列表
        long  last_begin_id      = -1;       ///< 最近一次匹配的源 ID
        long  last_end_id        = -1;       ///< 最近一次匹配的目标 ID
        double last_fitness_score = 0.0;     ///< 最近一次匹配的适应度分数
        int   edges_inserted     = 0;        ///< 已插入的闭环边总数
    };

    /**
     * @brief 构造函数
     * @param graph 交互式图对象的非拥有指针。
     *              调用者必须保证 graph 的生命周期长于此对象，
     *              或者在 graph 销毁前调用 stop()。
     */
    explicit AutomaticLoopClosure(InteractiveGraph* graph);
    ~AutomaticLoopClosure();

    // ── 生命周期管理 ──────────────────────────────────────────────────
    void start();   ///< 启动后台检测线程
    void stop();    ///< 停止后台检测线程
    bool is_running() const { return m_running.load(std::memory_order_acquire); }

    // ── 线程安全状态查询 ───────────────────────────────────────────────
    Status snapshot() const;

    // ── 运行时参数配置（可在运行前/运行时从 UI 线程设置） ─────────────
    enum SearchMethod { SEQUENTIAL = 0, RANDOM = 1 };  ///< 关键帧选择策略

    void set_search_method(int m)                  { m_search_method = m; }
    void set_distance_thresh(double v)             { m_distance_thresh = v; }
    void set_accum_distance_thresh(double v)       { m_accum_distance_thresh = v; }
    void set_fitness_score_thresh(float v)         { m_fitness_score_thresh = v; }
    void set_fitness_score_max_range(float v)      { m_fitness_score_max_range = v; }
    void set_registration_method_index(int idx)    { m_reg_methods.set_method_index(idx); }
    void set_registration_max_iterations(int n)    { m_reg_methods.set_max_iterations(n); }
    void set_registration_epsilon(float eps)       { m_reg_methods.set_transformation_epsilon(eps); }
    void set_registration_resolution(float r)      { m_reg_methods.set_resolution(r); }
    void set_robust_kernel_type(int t)             { m_kernel_type = t; }
    void set_robust_kernel_delta(float d)          { m_kernel_delta = d; }
    void set_optimize_after_insert(bool v)         { m_optimize = v; }

    // ── 供 UI 初始化的参数访问器 ──────────────────────────────────────
    const RegistrationMethods& reg_methods() const { return m_reg_methods; }
    int    search_method()          const { return m_search_method; }
    double distance_thresh()        const { return m_distance_thresh; }
    double accum_distance_thresh()  const { return m_accum_distance_thresh; }
    float  fitness_score_thresh()   const { return m_fitness_score_thresh; }
    float  fitness_score_max_range() const { return m_fitness_score_max_range; }
    int    robust_kernel_type()     const { return m_kernel_type; }
    float  robust_kernel_delta()    const { return m_kernel_delta; }
    bool   optimize_after_insert()  const { return m_optimize; }

private:
    // ── 后台线程主循环 ────────────────────────────────────────────────
    void loop_detection();                                              ///< 闭环检测主循环
    std::vector<long> find_loop_candidates(long source_id);             ///< BFS 查找闭环候选

    // ── 辅助函数 ───────────────────────────────────────────────────────
    void refresh_sorted_ids();                                          ///< 刷新排序后的 ID 列表缓存
    static const char* kernel_name(int type);                           ///< 根据索引获取鲁棒核函数名称
    void update_status(long source_id, const std::vector<long>& candidates);  ///< 更新状态（带锁）
    void update_last_match(long begin_id, long end_id, double fitness);        ///< 更新最近一次匹配状态

    // ── 成员变量 ───────────────────────────────────────────────────────
    InteractiveGraph* m_graph;          ///< 交互式图对象的非拥有指针

    // 线程管理
    std::thread  m_thread;              ///< 后台检测线程
    std::atomic_bool m_running{false};  ///< 线程运行标志（原子操作）

    // 状态快照
    mutable std::mutex m_status_mutex;  ///< 状态互斥锁
    Status m_status;                    ///< 当前状态快照

    // 运行时参数
    RegistrationMethods m_reg_methods;  ///< 点云配准方法管理器

    int    m_search_method         = 1;       ///< 搜索方法：0=顺序, 1=随机（默认随机）
    double m_distance_thresh       = 10.0;    ///< 空间距离阈值（米），候选与源的空间距离上限
    double m_accum_distance_thresh = 15.0;    ///< 图累计距离阈值（米），候选沿图路径距源的最小距离
    float  m_fitness_score_thresh  = 0.3f;    ///< 适应度分数阈值，低于此值视为成功匹配
    float  m_fitness_score_max_range = 2.0f;  ///< 适应度计算的最大距离范围（米）
    int    m_kernel_type           = 0;       ///< 鲁棒核函数类型索引（0=NONE）
    float  m_kernel_delta          = 0.01f;   ///< 鲁棒核函数 delta 参数
    bool   m_optimize              = true;    ///< 插入闭环边后是否自动执行全局优化

    // 排序后的关键帧 ID 缓存（用于顺序索引）
    std::vector<long> m_sorted_ids;      ///< 按 ID 排序的关键帧列表
    int  m_current_index = 0;            ///< 当前处理的索引位置
    size_t m_last_keyframe_count = 0;    ///< 上次缓存时的关键帧数量，用于检测变化
};

}  // namespace hdl_graph_slam
