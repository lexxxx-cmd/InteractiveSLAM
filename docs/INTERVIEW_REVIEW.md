# InteractiveSLAM 面试回顾 — 关键问题整理

> 基于代码审查 + Git 提交历史的深度分析，覆盖 Qt 开发、多线程、渲染管线、软件架构等方面。

---

## 1. 架构与生命周期管理

### 1.1 GraphManager 的栈构造 + 裸指针传递

**位置**: `main.cpp:33-35`

```cpp
GraphManager graphManager;                    // 栈上构造
MainWindow mainWindow(&graphManager);         // 裸指针传递
```

程序生命周期 = GraphManager 生命周期，栈构造没有问题。析构顺序：main 返回时 MainWindow 先析构，然后 GraphManager 析构，顺序正确。

### 1.2 `std::thread` 的生命周期管理

**位置**: `interactive_graph.cpp:61-65`

```cpp
InteractiveGraph::~InteractiveGraph() {
    if (optimization_thread.joinable()) {
        optimization_thread.join();
    }
}
```

**关键知识点**: 如果 joinable 的 `std::thread` 析构时既没有 `join()` 也没有 `detach()`，C++ 标准规定调用 `std::terminate()` —— 程序直接 crash，不是 UB，是标准强制行为。

**潜在问题**: `join()` 会阻塞直到优化线程结束。没有取消机制时，用户关闭程序可能需要等很久。

---

## 2. 三层架构分层原理

### 2.1 每一层做什么

```
GraphManager           Qt 桥接层
├─ 异步加载 (QtConcurrent::run)
├─ 信号槽 (statsChanged, loadingSucceeded...)
├─ shared_ptr 生命周期管理
└─ 进度翻译 (ProgressInterface → Qt signals)

InteractiveGraph       应用逻辑层
├─ 关键帧管理 (keyframes map)
├─ 边来源追踪 (EdgeSource: Manual/Auto/Anchor)
├─ 信息矩阵计算 (InformationMatrixCalculator)
├─ 后台优化线程 (std::thread)
├─ LVBA 格式导出
└─ 参数管理 (ParameterServer)

GraphSLAM              纯 g2o 封装
├─ add_se3_node / add_se3_edge
├─ 鲁棒核函数
├─ optimize() 同步调用
└─ save/load .g2o 文件

g2o                    第三方库
SparseOptimizer, VertexSE3, EdgeSE3, LM solver...
```

### 2.2 为什么有三层

**GraphSLAM**: 来自 `hdl_graph_slam`，剥离 ROS 依赖，把 "g2o 的 SE3 图操作" 抽象出来。可以独立测试 —— `tests/test_data_loading.cpp` 就是直接用它。

**InteractiveGraph**: GraphSLAM 只知道顶点和边，不知道 "关键帧"、"点云"、"回环是手动还是自动的"。InteractiveGraph 把 GraphSLAM 的能力**业务化**了 —— `add_edge()` 不止创建 g2o 边，还要同时计算信息矩阵、打 EdgeSource 标签、分配 ID。

**GraphManager**: InteractiveGraph 不依赖 Qt（只依赖抽象 `ProgressInterface`），UI 层不能直接操作它。GraphManager 做的事情 InteractiveGraph 本身做不了 —— `QtConcurrent::run` 异步加载、发射信号给 UI 绑定、`shared_ptr` 保活。

### 2.3 设计原则

**依赖方向永远向内**：外层可以依赖内层，内层绝不依赖外层。

```
GraphManager → InteractiveGraph → GraphSLAM → g2o
  (Qt)          (业务)            (纯图)      (数学)
```

验证方式：能不能在**不引入 Qt 头文件**的情况下编译 InteractiveGraph？能 → 依赖方向对。

**每层只多一件事**：如果一层做了两件不相关的事，就该拆。

**怎么想出来的**：不是先画出三层架构再写代码，而是写着写着发现 "这段东西不归这个类管"，就拆一层出来。SOLID 原则不是设计指南，是**重构方向**。

---

## 3. InteractiveGraph 的核心角色：桥梁

### 3.1 连接数学侧和可视化侧

```
数学侧（g2o）                    可视化侧
graph (SparseOptimizer)         keyframes map
num_vertices / num_edges         ├─ pose (从 g2o 顶点读)
optimize() → 改变顶点 pose       └─ cloud (PCL 点云)
save() / load()                 edge_sources map
chi2_before / chi2_after          └─ "这条边是谁加的"
```

**add_edge 干了什么**：同时更新两边 —— 调 g2o 建边（数学侧）+ 记 `edge_source = ManualLoop`（可视化侧）。

**optimize 干了什么**：在 g2o 图里跑优化 → vertex 的 pose 变了 → 由于 keyframe 的 `estimate()` 是 `node->estimate()` 的透传 → 可视化侧自动拿到新值。

### 3.2 edge_sources 是怎么工作的

`edge_sources` 是 InteractiveGraph **额外加的一个 `unordered_map<long, EdgeSource>`**，g2o 内部完全不知道这东西。

项目自己管理边 ID，没用 g2o 的默认 ID：

**加载时 — 重新编号**:
```cpp
edge_id_gen = 0;
for (auto& edge : graph->edges()) {
    edge->setId(edge_id_gen);        // 覆盖 g2o 原始 ID
    edge_sources[edge_id_gen] = EdgeSource::Original;
    edge_id_gen++;
}
```

**加边时 — 自增编号**:
```cpp
long eid = edge_id_gen++;            // 累加器分配新 ID
edge->setId(eid);                    // 设到 g2o 边上
edge_sources[eid] = source;          // 同一 ID 做 key
```

**渲染时 — ID 做桥梁**:
```cpp
eid = edge->id();                    // 从 g2o 拿
color = graph->edge_source(eid);     // 查 map 定颜色
```

---

## 4. KeyFrame 与 InteractiveKeyFrame

### 4.1 继承关系

```
KeyFrame (基类 — 纯数据)
├─ cloud            点云 (const_ptr)
├─ node             g2o 顶点指针 ← 通往图的桥梁
├─ odom             里程计位姿
├─ accum_distance   累积距离
├─ stamp_nsec       时间戳
├─ floor_coeffs / utm_coord / IMU ... 可选传感器
├─ save() / load()
└─ estimate()       透传 node->estimate()

        ↓ public 继承

InteractiveKeyFrame (派生 — 空间查询)
├─ 继承 KeyFrame 全部字段
├─ min_pt / max_pt  AABB 包围盒
├─ kdtree_          KD-Tree 空间索引
├─ normals_         法向量
├─ neighbors()      半径邻域搜索
└─ normals()        法向量获取（延迟计算+缓存）
```

### 4.2 实际使用情况

**真正被用到的字段只有**：`node`、`cloud`、`id()`、`accum_distance`（仅右键菜单1处）、`stamp_nsec`（仅 LVBA 导出1处）。

**从来没被访问过**：`odom`、`floor_coeffs`、`utm_coord`、`acceleration`、`orientation`、以及 InteractiveKeyFrame 多出来的所有功能（KD-Tree、法向量、包围盒）。

### 4.3 加载后 keyframes 自身不再变化

```
加载时:
  graph.g2o → g2o vertices (权威位姿)
  keyframe data → kf.odom, kf.cloud (补充数据)
  kf.node = pointer to g2o vertex (桥梁)

运行时:
  graph.optimize() → 修改 g2o vertices 内部值
  → kf.estimate() 自动拿到新值 (指向同一个对象)
  → kf.odom 不变

导出时:
  直接读 kf.node->estimate() → 拿到优化后的值
```

**keyframes map 就是一个用 ID 查 g2o 顶点的字典，加载后再也没被写过。** 所有动态状态（位姿）存在 g2o 那边。一层间接指针，省掉了全套同步逻辑。

---

## 5. ParameterServer

### 5.1 是什么

`map<string, any>` 的键值对字典，从 ROS 惯用模式搬过来。用的时候带默认值：

```cpp
int iters = params.param<int>("g2o_solver_num_iterations", 64);
```

key 不存在 → 自动用默认值注册。存在 → 直接返回。

### 5.2 用在哪

**只有两个地方**：

1. `interactive_graph.cpp` — 优化迭代次数：`params.param<int>("g2o_solver_num_iterations", 64)`
2. `information_matrix_calculator.hpp` — 9 个信息矩阵参数（`use_const_inf_matrix`、`const_stddev_x`、`fitness_score_thresh` 等）

### 5.3 当前状态

**过度设计**。所有调用都带着默认值，没有任何地方从文件加载参数覆盖。当前等价于几个 `const` 常量。如果后续要做用户可配置，`load` 逻辑加上文件读取就够了，所有调用点不用改。这是一个**预留的扩展点**，目前还没被利用。

---

## 6. 多线程安全

### 6.1 optimization_mutex 的三种使用模式

| 使用位置 | 手法 | 为什么 |
|---|---|---|
| `removeEdge()` | `unique_lock + try_to_lock` | UI 线程调用，不能阻塞。失败返回 false |
| `optimize_background()` | `lock_guard` | 后台工作线程，可以等 |
| `graph_statistics()` | 裸 `try_lock() + unlock()` | 定时器轮询，拿不到用缓存 |

### 6.2 裸 try_lock() 的风险

**位置**: `interactive_graph.cpp:453`

```cpp
if (optimization_mutex.try_lock()) {
    // ... 数据读取 ...
    optimization_mutex.unlock();  // ← 如果中间抛异常，锁泄漏
}
```

没有 RAII 保护，异常会导致锁永久泄漏。应该用：

```cpp
std::unique_lock<std::mutex> lock(optimization_mutex, std::try_to_lock);
if (lock.owns_lock()) {
    // ... 数据读取 ...
}  // 自动 unlock，异常安全
```

### 6.3 lock_guard vs unique_lock vs try_lock 速查

| | `lock_guard` | `unique_lock` | 裸 `try_lock()` |
|---|---|---|---|
| 拿不到锁 | 阻塞等待 | 取决于构造参数 | 立即返回 false |
| RAII | ✅ | ✅ | ❌ 必须手动 unlock |
| 可提前解锁 | ❌ | ✅ | ✅ |
| 转移所有权 | ❌ | ✅ | ❌ |
| 异常安全 | ✅ | ✅ | ❌ 锁泄漏 |

- `try_to_lock` 是传给 `unique_lock` 构造函数的**标签**（`std::try_to_lock_t`），不是函数
- `try_lock()` 是 `std::mutex` 的成员函数，裸调用无 RAII
- **永远优先用 `unique_lock + try_to_lock` 替代裸 `try_lock()`**

### 6.4 跨线程 GUI 操作

**位置**: `MainWindow.cpp:656-663`

```cpp
QMetaObject::invokeMethod(this, [this]() {
    m_viewport->refreshScene();
    m_viewport->rebuildPointClouds();
    statusBar()->showMessage(tr("Optimization complete"), 3000);
}, Qt::QueuedConnection);
```

这段 lambda 在后台线程执行，但操作的 `m_viewport` 和 `statusBar()` 都是 GUI 对象。`QueuedConnection` 将 lambda 序列化到主线程事件队列。直接调用 = 随机 crash。

| 方式 | 机制 | 适用场景 |
|---|---|---|
| `invokeMethod + QueuedConnection` | 跨线程投递事件 | 跨线程调用 GUI 方法 |
| `QTimer::singleShot(0, ...)` | 一次性定时器延迟执行 | 本线程延迟执行 |

### 6.5 后台线程状态轮询（AutoLoopClosurePanel）

自动回环检测跑在 `std::thread` 里（不是 QThread，没有事件循环），UI 通过 10Hz 定时器轮询：

```cpp
m_pollTimer->setInterval(100);

void onPollStatus() {
    auto status = m_autoLoop->snapshot();  // 线程安全的快照拷贝（内部用 m_status_mutex）
    m_sourceLabel->setText(status.current_source_id);  // 主线程安全
    if (status.edges_inserted > m_lastKnownEdgesInserted)
        emit loopEdgeInserted();  // 通知 MainWindow 刷新视口
}
```

不是用 Qt 信号跨线程（后台线程没有事件循环），而是**轮询 + 线程安全快照**。

### 6.6 缺失的优化取消机制 + 解决方案

**现状**: 没有 `atomic_bool` 取消标志，`closeMap()` 无法打断正在运行的优化。

**g2o 固有限制**: `g2o::SparseOptimizer::optimize(n)` 是原子调用，进入后无法从外部打断。

**解决方案 — 迭代级拆分**:
```cpp
// 原来（不可打断）
g2o->optimize(64);

// 改为（每轮迭代可检查取消标志）
std::atomic<bool> m_cancelled{false};
for (int i = 0; i < maxIter && !m_cancelled.load(); ++i) {
    g2o->optimize(1);
}
```

`join()` 从等 64 轮变成最多等 1 轮。

---

## 7. 渲染管线

### 7.1 updateScene vs rebuildPointClouds

| 方法 | 操作 | 开销 | 调用时机 |
|---|---|---|---|
| `updateScene()` | 更新球体和边线段的变换矩阵 | ~微秒 | 每次数据变化 |
| `rebuildPointClouds()` | 全量 CPU 变换 + GPU 上传 | 几十~几百 ms | 仅优化/回环后 |

**为什么不能合并**: `rebuildPointClouds()` 需要遍历所有关键帧 → 逐点变换 → 合并 → 上传 GPU。60Hz 定时器里跑 = 幻灯片。

### 7.2 两个定时器的分工

| 定时器 | 位置 | 职责 |
|---|---|---|
| OSGRenderer 10ms 定时器 | `OSGRenderer::timerEvent()` | 真正的渲染循环 — `frame()` |
| ViewportWidget 16ms 定时器 | `ViewportWidget::updateScene()` | 正交投影动态跟踪 + FPS 统计 |

`updateScene()` 真正不可替代的只有**正交投影的动态视锥体调整**。

### 7.3 为什么远处卡、近处流畅 — GPU fill-rate 过绘制

**不是 LOD，不是按需加载，是纯 GPU 像素填充率瓶颈**。

```
远处缩小：所有顶点 → 挤在几百平方像素 → 每像素上百次片段着色器 → GPU ROP 满 → 卡
近处放大：所有顶点 → 分散到全屏 → 每像素 2-3 次片段着色器 → 流畅
```

**根因**：点云是单一大 VBO，包围盒覆盖整个地图。OSG 视锥体裁剪按 Geometry 级别 → 永远在视锥体内 → 顶点着色器始终跑全量。瓶颈在片段过绘制，不是顶点数。

### 7.4 单 VBO vs 多 VBO + MatrixTransform — 一次回退

**Git 提交故事** (`3a6ff62` → `00d4a42`，间隔 2 小时):

| 方案 | 单 VBO（最终采用） | 多 VBO + MatrixTransform（被回退） |
|---|---|---|
| Draw call 数 | **1** | N（每个关键帧 1 个） |
| optimize 后更新 | 高（全量重建） | **低（O(1) 矩阵更新）** |
| 每帧渲染 | **低** | 高（N 次状态切换） |
| 决策 | ✅ 优化偶尔发生，渲染每帧发生 | ❌ 优化快了但渲染一直慢 |

---

## 8. 数据到渲染的完整链路（手动回环边）

### 8.1 从右键到像素

```
用户右键 A → "回环起点"、右键 B → "回环终点"
  → LoopClosureDialog（FPFH + 扫描匹配 + 手动微调）
  → onAddEdge()
    → InteractiveGraph::add_edge()
      → inf_calclator 算信息矩阵
      → GraphSLAM::add_se3_edge() → 写入 g2o 图
      → edge_sources[eid] = ManualLoop（标记来源）
    → optimize() → g2o 修改所有顶点位姿
  → accept() → MainWindow 收到 Accepted
    → m_viewport->refreshScene()
      → EdgeLineVisualizer::rebuild()
        遍历 g2o→edges() → 读 vertex->estimate() + edge_source → 着色 → VBO
    → m_viewport->rebuildPointClouds()
      遍历所有 keyframes → kf.cloud + kf.node->estimate() → 合并 → GPU
```

### 8.2 EdgeLineVisualizer 怎么渲染

```cpp
void rebuild(InteractiveGraph* graph, const set<long>& hiddenEdgeIds) {
    for (auto* edge : g2oGraph->edges()) {           // 遍历 g2o 里所有边
        auto* v1 = edge->vertices()[0];              // 拿顶点
        auto* v2 = edge->vertices()[1];
        p1 = v1->estimate().translation();           // 从 g2o 读位姿
        p2 = v2->estimate().translation();
        
        switch (graph->edge_source(eid)) {           // 从 InteractiveGraph 查来源
            case ManualLoop: color = 绿色; break;
            case AutoLoop:   color = 青色; break;
            case Original:   color = 灰色; break;
        }
        m_vertices->push_back(p1, p2);               // 写入 VBO
    }
    m_vertices->dirty();  // → GPU
}
```

**没有中间缓存。** 渲染时直接遍历 g2o 内部数据结构读，g2o 变了渲染就跟着变。这就是为什么 `refreshScene()` 只需要重新遍历一遍就行，不用 "同步" 任何状态。

---

## 9. UI 层面试要点

### 9.1 滑块增量归零模式（LoopClosureDialog）

六个位姿调整滑块不是常规绝对值输入，而是**增量式**：

```cpp
void onSliderPXChanged(double value) {
    double delta = value - m_sliderPrevValues[0];  // 算增量
    m_sliders[0]->blockSignals(true);              // 防止递归
    m_sliders[0]->setValue(0.0);                   // 自动归零
    m_sliders[0]->blockSignals(false);
    applySliderDelta(0, delta, false);             // 应用到终点位姿
}
```

**为什么用 `blockSignals`**：`setValue(0)` 会再次触发 `valueChanged` 信号，不加防护就无限递归。

### 9.2 复选框信号循环防护（EdgeListPanel）

```cpp
bool m_updatingCheckState = false;  // 守卫标志

void onItemChanged(QTreeWidgetItem* item, int column) {
    if (m_updatingCheckState) return;
    m_updatingCheckState = true;
    // ... 修改其他复选框状态 ...
    m_updatingCheckState = false;
}
```

Qt 的 `itemChanged` 不区分 "用户操作" 和 "程序修改"，必须手动防护。

### 9.3 迷你视口 GL 上下文延迟初始化（MiniViewportWidget）

**问题**：Qt 的 `QOpenGLWidget` 在构造函数执行时还没有 GL 上下文。

**解决**：分两段构建：

```
构造函数 → setupGeometries()   ← 创建 CPU 数据结构，不需要 GL
initialized 信号 → initOsg()   ← 设相机、投影矩阵、挂场景图（需要 GL）
```

**为什么这样安全**：`osg::Geometry`、`osg::Vec3Array` 只是内存里的数组，不需要 GL。VBO 在第一次渲染时自动懒创建。相机设置需要 `glViewport` 等 GL 函数 → 必须等上下文存在。

### 9.4 迷你视口使用局部坐标系

迷你视口不渲染世界坐标，渲染的是**相对位姿**：

```
蓝色点云 → 始终在原点（起点帧）
绿色点云 → 在 relPose = beginPose⁻¹ × endPose 处（终点帧）
球体 + 坐标轴 → 标记终点位置和朝向
```

用户拖滑块 → `updateEndPose()` 只重建终点云 → 即时预览对齐效果。

### 9.5 主视口和迷你视口的关系

两个完全独立的 Viewer：

```
MainWindow → ViewportWidget → osgQOpenGLWidget → Viewer #1（完整地图）
LoopClosureDialog → MiniViewportWidget → osgQOpenGLWidget → Viewer #2（仅 2 帧）
```

每个 `osgQOpenGLWidget` 内部持有自己的 `OSGRenderer`（继承自 `osgViewer::Viewer`），各有各的相机、场景图。共享同一块 GPU 但场景完全隔离。

### 9.6 OverlayPanelWidget — 自定义拖拽浮窗

**位置**: `OverlayPanelWidget.cpp`

```cpp
void mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton && 标题栏区域) {
        m_dragging = true;
        m_dragStartPos = event->globalPos() - mapToParent(0,0);
    }
}

void mouseMoveEvent(QMouseEvent* event) {
    if (m_dragging) {
        QPoint newPos = event->globalPos() - m_dragStartPos;
        // 限制在父部件边界内
        newPos.setX(std::max(0, std::min(newPos.x(), parentW - width())));
        newPos.setY(std::max(0, std::min(newPos.y(), parentH - height())));
        move(newPos);
    }
}
```

面试点：手动实现拖拽而非使用 `QDockWidget`，支持悬浮在 3D 视口之上。QSS 样式表实现深色半透明主题。

---

## 10. 设计模式

### 10.1 简单工厂 — 配准方法

**位置**: `registration_methods.cpp:79-141`

```cpp
switch (registration_method) {
    case 0: /* ICP */ break;
    case 1: /* GICP */ break;
    // ...
}
```

**新增算法代价**: 需同步修改 3 处 —— 字符串列表 + switch case + 可能的 `#ifdef` 宏。三处没有编译期约束，容易遗漏。

**为什么不用注册机制**: 仅 6 个固定算法，switch-case 足够清晰。注册机制（静态初始化 + map 查找）对当前规模是过度设计。面试中正确的态度是：**"我知道有更好的模式，但我知道什么时候该用、什么时候不该用。"**

### 10.2 注册机制（Self-Registration Pattern）

如果算法频繁增删，更好的做法：

```cpp
// 每个算法自己的 .cpp 文件
static bool g_registered = []() {
    RegistrationFactory::registerMethod("GICP_OMP", []() {
        return new pclomp::GICP();
    });
    return true;
}();
```

新增算法 = 新建一个 `.cpp` + 一行 CMakeLists.txt。零行改动已有代码。OCP 完美满足。

### 10.3 ProgressInterface — 依赖倒置原则

**位置**: `progress_interface.hpp`

Data Layer 不依赖 Qt，通过抽象接口报告进度。Backend Layer 的 `ProgressReporter` 实现此接口，翻译为 Qt 信号。

---

## 11. 自动回环检测

### 11.1 配准器重复构造的性能 bug

**位置**: `automatic_loop_closure.cpp`, commit `c107560`

```cpp
// 改前 — 每轮循环创建
while (m_running) {
    auto registration = m_reg_methods.method();  // ❌ GICP 构造 = KdTree + 协方差
}

// 改后 — 提到循环外
auto registration = m_reg_methods.method();      // ✅ 一次构造
while (m_running) {
    registration->setInputTarget(cloud);          // 只改输入
}
```

发现方式：逐步骤计时诊断（align 耗时、fitness 耗时、每轮总耗时）。

---

## 12. OSG 与 Qt 集成的坑

| 问题 | commit | 修复 |
|---|---|---|
| 点云渲染为 1px 不可见 | `4d892ff` | 着色器添加 `gl_PointSize` |
| GL3 核心模式状态未配置 | `341d213` | `OSGRenderer` 启用 GL3 核心模式 |
| 固定管线光照/材质干扰渲染 | `deff83c` | 全局禁用固定管线光照/材质/雾效 |
| Qt 6.9 VAO 断言失败 | `17d3998` | `QOpenGLVertexArrayObject::Binder(nullptr)` 在 Qt 6.9 断言 v 非空，改用手动 VAO 恢复 |
| 窗口 resize 后视口比例不对 | `bc943ae` | resize 时同步更新相机 viewport 宽高比 |

---

## 13. 快速参考：关键文件索引

| 关注点 | 文件 |
|---|---|
| 程序入口 & 生命周期 | `main.cpp` |
| 主窗口 & 菜单逻辑 | `src/ui/MainWindow.h/cpp` |
| 3D 视口 & 渲染控制 | `src/ui/ViewportWidget.h/cpp` |
| 回环闭合对话框 | `src/ui/LoopClosureDialog.h/cpp` |
| 迷你 3D 视口 | `src/ui/MiniViewportWidget.h/cpp` |
| 浮动面板容器 | `src/ui/OverlayPanelWidget.h/cpp` |
| 自动回环控制面板 | `src/ui/AutoLoopClosurePanel.h/cpp` |
| 回环边列表 | `src/ui/EdgeListPanel.h/cpp` |
| 图数据 & 优化 | `src/data/hdl_graph_slam/interactive_graph.hpp/cpp` |
| g2o 封装 | `src/data/hdl_graph_slam/graph_slam.hpp/cpp` |
| 关键帧数据结构 | `src/data/hdl_graph_slam/keyframe.hpp/cpp` |
| 交互式关键帧 | `src/data/hdl_graph_slam/interactive_keyframe.hpp/cpp` |
| 自动回环检测 | `src/data/hdl_graph_slam/automatic_loop_closure.hpp/cpp` |
| 配准工厂 | `src/data/hdl_graph_slam/registration_methods.hpp/cpp` |
| 信息矩阵计算 | `src/data/hdl_graph_slam/information_matrix_calculator.hpp` |
| 参数服务器 | `src/data/hdl_graph_slam/parameter_server.hpp` |
| 进度接口（跨层解耦） | `src/data/hdl_graph_slam/progress_interface.hpp` |
| 点云渲染器 | `src/visualizers/KeyframePointCloudVisualizer.h` |
| 边线渲染器 | `src/visualizers/EdgeLineVisualizer.h` |
| 场景管理器 | `src/visualizers/GraphSceneVisualizer.h` |
| OSG-Qt 桥接 | `src/osgQOpenGL/OSGRenderer.h/cpp` |
| 后端桥接 | `src/backend/graph_manager.hpp/cpp` |
| 构建配置 | `CMakeLists.txt` |

---

*更新于 2026-07-09 | 基于代码审查 + Git 提交历史 (f6af6cf 回溯至 e3afd76)*
