# 大点云渲染性能优化报告 —— 方案 A：点预算自适应体素降采样

> 项目：InteractiveSLAM（交互式位姿图编辑与回环闭合工具）
> 日期：2026-08-17
> 状态：已实现，用户环境编译通过

---

## 1. 需求背景

传入地图数据量大（合并后点云总数上千万）时，渲染帧率严重下降。用户提出两个问题：

1. 项目是否已有类似常见点云渲染软件的空间索引 + LOD + 外存渲染 + GPU 加速方案？
2. 是否可以"以某渲染帧率为目标，多次降采样至合适点数后再渲染几何体"，或采用其他解法？

经代码核查，**项目原本没有任何空间索引 / LOD / 外存 / GPU 加速方案，也没有帧率目标自适应降采样**。经与用户确认，选择 **方案 A：点预算自适应降采样**（改动最小、见效最快，保留全量数据、提供档位控制）先行实现。

---

## 2. 改造前现状与瓶颈分析

### 2.1 原有渲染管线（代码依据）

```
打开地图(Ctrl+O)
  └─ GraphManager::openMapData()           后台线程加载
       └─ InteractiveGraph::load_map_data()  graph.g2o + 关键帧
            └─ KeyFrame::load()              优先 raw.pcd（全分辨率），回退 cloud.pcd
  └─ loadingSucceeded → ViewportWidget::onGraphLoaded()  主线程
       └─ GraphSceneVisualizer::buildFromGraph()
            └─ rebuildPointClouds()
                 └─ KeyframePointCloudVisualizer:
                      appendCloud()  CPU 逐点按位姿变换到世界坐标 → m_allWorldPoints
                      finish()       全部点填入单个 osg::Geometry（VBO/VAO, GL_POINTS）
  └─ 每帧: 顶点着色器处理全部点（CoreShaders.h, uPointSize + z_clipping discard）
```

### 2.2 关键代码位置

| 环节 | 位置 |
|---|---|
| 单一大 VBO 合并渲染 | `src/visualizers/KeyframePointCloudVisualizer.h`（`finish()` 全量上传） |
| 重建触发（主线程） | `src/ui/MainWindow.cpp:324,688` → `ViewportWidget::rebuildPointClouds()` |
| 加载优先全分辨率 | `src/data/hdl_graph_slam/keyframe.cpp:240-248`（优先 `raw.pcd`） |
| 采样步长只作用于球体/边 | `src/visualizers/GraphSceneVisualizer.h:177`（注释明确"点云不受采样影响"） |
| FPS 仅统计显示 | `src/ui/ViewportWidget.cpp:471-480`（无任何反馈控制） |
| 唯一的降采样 | `keyframe.cpp:119-124`（**保存时** 0.02m 体素滤波写 `cloud.pcd`，渲染不用它） |

### 2.3 上千万点的瓶颈

1. **每帧顶点着色器处理全部点**：点大小 3px 时 overdraw 严重；Z 裁剪的 `discard` 在片元阶段，只减少像素填充，不减少顶点处理量；
2. **重建在 UI 线程**：CPU 变换 + GPU 上传全量点，加载/优化后界面长时间卡死；
3. **内存双份**：`m_allWorldPoints`（Eigen::Vector3d，24B/点）+ GPU 端 `Vec3Array`+`Vec4Array`（28B/点），千万点约 500MB+；
4. **单个几何体无法部分剔除**：OSG 视锥剔除对整块 Geometry 无效。

---

## 3. 方案对比与选择

| 方案 | 内容 | 代价 | 结论 |
|---|---|---|---|
| **A 点预算自适应降采样** | 合并后按点预算体素降采样，只上传降采样子集；保留全量数据 | 小 | ✅ **已实现** |
| B 多级 LOD 金字塔 | A + 3~4 级降采样按相机距离切换 | 中 | 后续可叠加 |
| C 空间分块 + 剔除 | 均匀网格分块，每块独立 Geometry 获得视锥剔除 | 较大 | 为外存打基础 |
| D 外存渲染（Potree 式） | 离线八叉树 + 视点流式加载 | 最大 | 上亿点级再考虑 |
| 配套优化 | 后台线程重建 + 默认加载降采样 pcd | 小 | 建议后续跟进 |

**选 A 的理由**：改动集中在单个可视化器，PCL 体素滤波思路无需新增依赖，对 ≤ 预算的小地图**零行为变化**，立竿见影解决"上千万点卡顿"问题。

---

## 4. 实现方案（方案 A）

### 4.1 核心设计

```
全量数据层（内存，始终保留）         渲染数据层（GPU，受预算控制）
┌──────────────────────────┐        ┌──────────────────────────┐
│ m_allWorldPoints         │        │ m_vertices / m_colors    │
│   （世界坐标全量点）        │ ─────▶ │   （降采样后子集）         │
│ m_cloudRanges            │ 映射    │ m_renderRanges           │
│   （每帧全量范围）          │        │   （每帧渲染范围）         │
│ 用途：Z 统计/高亮/导出     │        │ 用途：每帧渲染            │
└──────────────────────────┘        └──────────────────────────┘
     ▲ 渲染索引 m_renderIndices 把渲染点映射回全量点，高亮/着色仍正确
```

- **预算语义 = 点数上限**：全量点数 ≤ 预算时全量直通（与旧行为一致）；超过预算才触发降采样；
- **不做逐帧 FPS 负反馈**：用实时 FPS 做反馈会振荡（降采样→帧率升→恢复→又卡），点预算稳定可预期；
- **重建时机不变**：仅加载、图优化、回环插入后重建，不增加每帧开销。

### 4.2 降采样算法（体素化 + 自适应边长）

```
1) voxelKey(x,y,z,invLeaf)    体素坐标 floor(x/leaf) 取整 → FNV-1a 64 位哈希成单键
2) countVoxels(leaf)          一次 O(n) 遍历统计当前边长下体素数（≈渲染点数）
3) 迭代选边长:
       leaf0 = cbrt(包围盒体积 / 预算) × 2     ← 初始刻意偏大，控制临时哈希内存峰值
       循环（≤6 次）:
           c = countVoxels(leaf)
           if c ≤ 预算: 结束
           leaf *= pow(c/预算, 0.5) × 1.03     ← 表面分布近似点数 ∝ 1/leaf²，0.5 次方快速收敛
4) buildDecimated(leaf)       一次 O(n) 遍历同时完成：
       - 每个体素保留"最先出现"的点 → 渲染索引天然升序
       - 按关键帧切分 → m_renderRanges
       - 每帧至少保留 1 点 → 选中/播放高亮时该帧仍有可见点
```

典型收敛：表面型数据 2 次遍历，体积型数据 1~2 次，瞬态哈希表峰值约 2×预算。

### 4.3 高亮/重着色的正确性适配

原 `recolorHighlight`/`recolorAll` 直接以全量数组下标索引颜色，降采样后会越界/错位。改造后：

- 全部改为遍历 **`m_renderRanges`**（渲染数组上的逐帧范围）；
- 取 Z 值时经 **`m_renderIndices`** 映射回全量点；
- 新增 **`m_highlightIds`**：重建后自动重新应用高亮，切换预算档位不丢选中态；
- 全量直通时 `m_renderFullRes=true`，不建索引映射，**省 8B/点内存**。

---

## 5. 文件改动明细

| 文件 | 改动 |
|---|---|
| `src/visualizers/KeyframePointCloudVisualizer.h` | **核心**。新增渲染数据层（`m_renderIndices`/`m_renderRanges`/`m_renderFullRes`/`m_highlightIds`/`m_bMin`/`m_bMax`）；新增 `setPointBudget`/`pointBudget`/`renderPointCount`/`totalPointCount`；新增 `rebuildRenderCloud`/`countVoxels`/`buildDecimated`/`estimateLeaf`/`voxelKey`/`applyHighlight`；`finish()` 改调 `rebuildRenderCloud()`；`recolorHighlight`/`recolorAll` 改为渲染数组感知；`clear()` 重置新状态 |
| `src/visualizers/GraphSceneVisualizer.h` | 新增 `m_pointBudget=5000000`（默认 500 万）与 `setPointBudget`/`pointBudget`/`renderPointCount`/`totalPointCount`；`rebuildPointClouds()` 创建可视化器时应用预算 |
| `src/ui/ViewportWidget.h/.cpp` | 新增槽 `setPointBudget(int)`；新增信号 `pointCloudStatsChanged(qint64,qint64)`，在点预算变化与点云重建完成时发射 |
| `src/ui/DrawFlags.h` | `DrawFlags` 新增 `int point_budget = 5000000;` |
| `src/ui/RenderingPanel.h/.cpp` | 渲染面板新增「Point Budget」下拉框（全量/1000万/500万/200万/100万/50万，默认 500 万）与「Rendered: X / Y points」实时统计标签 |
| `README.md` | 渲染面板表格补充「点预算」说明 |

### 5.1 UI 交互

- **渲染面板 → Point Budget**：切换档位立即重建渲染数据（触发体素降采样），并显示 `Rendered: X / Y points`（已渲染 / 全量），可直观确认降采样生效；
- 默认 500 万：≤500 万的地图完全不受影响，>500 万的图自动降采样到预算以内；需要全分辨率可切「Full (all points)」。

---

## 6. 设计决策记录

| 决策 | 理由 |
|---|---|
| 预算上限语义（非 FPS 反馈） | 避免降采样↔恢复的振荡；用户可通过档位直接控制质量/性能平衡 |
| 全量数据保留在内存 | 高亮重着色、Z 范围统计、未来导出仍用全量；降采样只影响上传 GPU 的部分 |
| 只在上传前降采样 | 与现有"仅加载/优化后重建"节奏一致，每帧零额外开销 |
| 每帧至少保留 1 点 | 极端降采样下选中/播放高亮仍有可见点 |
| 初始边长偏大 ×2 + 迭代收敛 | 控制瞬态哈希表内存峰值（避免首轮体素数爆炸） |
| 小地图零开销 | 点数 ≤ 预算走全量直通路径，行为与改动前完全一致 |

---

## 7. 验证

- **用户环境**：编译通过（Release，MSVC 2022 / Qt 6.9.1 / vcpkg）。
- **本会话沙箱**：CMake AUTOMOC 阶段调用 `moc.exe` 子进程被沙箱以 EPERM 拦截（`libuv process spawn failed: operation not permitted`），无法在本会话内完成完整构建；代码层面已做静态核查（API 签名、符号引用、信号槽连接一致性）。

---

## 8. 后续建议（可选）

| 优先级 | 事项 | 说明 |
|---|---|---|
| 高 | **配套优化：后台线程重建** | `rebuildPointClouds()` 迁至 `QtConcurrent` + 双缓冲 VBO，消除加载/优化后的 UI 卡死（上千万点时重建本身约 1~2s） |
| 中 | 加载策略 | 默认读 0.02m 降采样 `cloud.pcd` 或提供"高/低分辨率加载"选项，减少首次加载耗时与内存 |
| 中 | 方案 B：多级 LOD | 预生成 3~4 级降采样（1/4、1/16、1/64），按相机距离切换（OSG LOD），近距离保持细节 |
| 低 | 颜色压缩 | 顶点颜色 `Vec4` → `RGB`，省 25% 显存带宽 |
| 低 | 方案 C：空间分块 | 世界网格分块 + 独立 Geometry，获得视锥剔除，为外存渲染打基础 |
| 远期 | 方案 D：外存渲染 | Potree 式离线八叉树 + 视点流式加载，面向上亿点 |

---

## 9. 参考

- 合并渲染与着色：`src/visualizers/KeyframePointCloudVisualizer.h`、`src/visualizers/CoreShaders.h`、`src/visualizers/TurboColormap.h`
- 场景协调：`src/visualizers/GraphSceneVisualizer.h`
- 加载链路：`src/backend/graph_manager.cpp`、`src/data/hdl_graph_slam/interactive_graph.cpp`、`keyframe.cpp`
- 交互面板：`src/ui/RenderingPanel.h/.cpp`、`src/ui/ViewportWidget.h/.cpp`
