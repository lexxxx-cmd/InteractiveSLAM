// ============================================================================
// GraphSceneVisualizer.h
// 图场景可视化器 —— 中央协调器
//
// 功能：构建和管理从 InteractiveGraph 生成的 OSG 场景图，拥有所有子可视化器。
//       是整个可视化系统的核心调度中心。
//
// 场景结构：
//   root (osg::Group)
//   ├── GroundGridVisualizer     (地面网格，静态)
//   ├── CoordinateAxesVisualizer (坐标轴，静态)
//   ├── m_sphereGroup            (顶点球体组，动态，可切换可见性)
//   ├── m_edgeGroup              (边线段组，动态，可切换可见性)
//   └── m_cloudGroup             (点云组，动态，可切换可见性)
//
// 所有球体和边都在世界坐标系中构建（不使用 MatrixTransform），
// 与 EdgeLineVisualizer 使用相同的经过验证的模式。
// ============================================================================

#pragma once

#include <osg/Group>
#include <osg/MatrixTransform>

#include <algorithm>
#include <memory>
#include <set>
#include <unordered_map>
#include <Eigen/Geometry>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "ui/DrawFlags.h"

#include "visualizers/CoordinateAxesVisualizer.h"
#include "visualizers/GroundGridVisualizer.h"
#include "visualizers/VertexSphereVisualizer.h"
#include "visualizers/PointCloudBuilder.h"
#include "visualizers/KeyframePointCloudVisualizer.h"
#include "visualizers/EdgeLineVisualizer.h"

/**
 * @brief 图场景可视化器 —— 场景图构建和管理的中央协调器
 *
 * 核心职责：
 *   1. 从 InteractiveGraph 构建完整的 OSG 场景图
 *   2. 管理所有子可视化器（球体、边线、点云、坐标轴、地面网格）
 *   3. 提供可见性开关（顶点、边、点云各自独立控制）
 *   4. 支持点云 Z 轴裁剪和颜色范围控制
 *   5. 支持顶点选择高亮（高亮选中顶点附近的时序邻居帧）
 *   6. 支持回环检测可视化（搜索源为蓝色、候选为绿色）
 *   7. 支持隐藏指定边（通过 EdgeListPanel 交互）
 *
 * 性能考虑：
 *   - updatePoses() 每帧调用，仅更新球体和边位置
 *   - 点云由 PointCloudBuilder 在后台线程构建，commitPointCloudBuild()
 *     换入场景，不阻塞主线程
 */
class GraphSceneVisualizer {
public:
    /** @brief 构造函数：创建场景根节点、子分组和静态元素 */
    GraphSceneVisualizer() {
        m_root = new osg::Group;
        m_root->setName("GraphScene");

        // 创建子分组，用于各元素的可见性独立开关
        m_sphereGroup = new osg::Group;
        m_sphereGroup->setName("Spheres");
        m_edgeGroup = new osg::Group;
        m_edgeGroup->setName("Edges");
        m_cloudGroup = new osg::Group;
        m_cloudGroup->setName("PointClouds");

        // 创建静态场景元素（坐标轴和地面网格）
        m_axes = std::make_unique<CoordinateAxesVisualizer>();
        m_grid  = std::make_unique<GroundGridVisualizer>(100.0f);

        // 组装场景树
        m_root->addChild(m_grid->getNode());
        m_root->addChild(m_axes->getNode());
        m_root->addChild(m_sphereGroup);
        m_root->addChild(m_edgeGroup);
        m_root->addChild(m_cloudGroup);
    }

    /** @brief 获取场景根节点 */
    osg::ref_ptr<osg::Group> getRootNode() const { return m_root; }

    // ========================================================================
    // 可见性开关
    // ========================================================================

    /** @brief 设置是否绘制顶点球体 */
    void setDrawVertices(bool v) {
        m_sphereGroup->setNodeMask(v ? ~0u : 0u);
    }

    /** @brief 设置是否绘制边线段 */
    void setDrawEdges(bool v) {
        m_edgeGroup->setNodeMask(v ? ~0u : 0u);
    }

    /** @brief 设置是否绘制关键帧点云 */
    void setDrawKeyframeClouds(bool v) {
        m_drawClouds = v;
        m_cloudGroup->setNodeMask(v ? ~0u : 0u);
    }

    /** @brief 设置点云中点的大小 */
    void setPointSize(float size) {
        m_pointSize = size;
        if (m_cloudViz) m_cloudViz->setPointSize(size);
    }

    /** @brief 设置点云透明度 */
    void setPointOpacity(float opacity) {
        m_pointOpacity = opacity;
        if (m_cloudViz) m_cloudViz->setOpacity(opacity);
    }

    /** @brief 当前点云透明度（供后台构建时生成一致的颜色） */
    float getPointOpacity() const { return m_pointOpacity; }

    /**
     * @brief 设置 LOD 多级渲染开关
     *
     * 开启后 PointCloudBuilder 在后台构建多级降采样，
     * ViewportWidget 按相机距离自动切换级别或手动固定层级：
     * 近处全量细节，远处低分辨率轮廓，减少远距离顶点处理量。
     * 渲染固定为"全量 + LOD"（点预算档位已移除）。
     */
    void setLodEnabled(bool enabled) {
        m_lodEnabled = enabled;
    }

    /** @brief 查询 LOD 开关状态 */
    bool lodEnabled() const { return m_lodEnabled; }

    /** @brief 按距离切换点云 LOD 级别（转发到可视化器） */
    void setLodLevel(int level) {
        if (m_cloudViz) m_cloudViz->setLodLevel(level);
    }

    /**
     * @brief 渐进上传推进（每帧由 ViewportWidget::updateScene 调用）
     *
     * 分块点云提交后逐帧显示一块，把一次超大 VBO 上传摊成多次小块上传，
     * 避免"后台构建完成换回主线程那一帧"的 GPU 卡顿。
     */
    void advanceChunkUpload() {
        if (m_cloudViz) m_cloudViz->advanceChunkUpload();
    }

    /** @brief 是否仍有分块在渐进上传中（点云渲染是否完成） */
    bool chunkUploadPending() const {
        return m_cloudViz ? m_cloudViz->chunkUploadPending() : false;
    }

    /** @brief 当前激活的 LOD 级别 */
    int currentLodLevel() const {
        return m_cloudViz ? m_cloudViz->currentLodLevel() : 0;
    }

    /** @brief 点云 LOD 级别总数（≥1） */
    int lodLevelCount() const {
        return m_cloudViz ? m_cloudViz->lodLevelCount() : 1;
    }

    /** @brief 点云世界包围球中心（供相机距离计算） */
    Eigen::Vector3d boundsCenter() const {
        return m_cloudViz ? m_cloudViz->boundsCenter() : Eigen::Vector3d::Zero();
    }

    /** @brief 点云世界包围球半径（供相机距离计算） */
    double boundsRadius() const {
        return m_cloudViz ? m_cloudViz->boundsRadius() : 0.0;
    }

    // ========================================================================
    // Z 轴裁剪控制
    // ========================================================================

    /** @brief 启用/禁用 Z 轴范围裁剪 */
    void setZClipping(bool enabled) {
        if (m_cloudViz) m_cloudViz->setZClipping(enabled);
    }

    /** @brief 设置 Z 轴裁剪范围 */
    void setZClipRange(float minZ, float maxZ) {
        if (m_cloudViz) m_cloudViz->setZClipRange(minZ, maxZ);
    }

    /** @brief 设置按 Z 轴着色的颜色范围 */
    void setColorZRange(float minZ, float maxZ) {
        if (m_cloudViz) m_cloudViz->setColorZRange(minZ, maxZ);
    }

    /** @brief 设置是否自动计算颜色范围（根据数据范围） */
    void setAutoColorRange(bool autoRange) {
        if (m_cloudViz) m_cloudViz->setAutoColorRange(autoRange);
    }

    // —— 查询当前状态 ——
    float getDataZMin() const { return m_cloudViz ? m_cloudViz->getDataZMin() : 0.0f; }
    float getDataZMax() const { return m_cloudViz ? m_cloudViz->getDataZMax() : 0.0f; }
    float getColorZMin() const { return m_cloudViz ? m_cloudViz->getColorZMin() : 0.0f; }
    float getColorZMax() const { return m_cloudViz ? m_cloudViz->getColorZMax() : 0.0f; }
    float getZClipMin() const { return m_cloudViz ? m_cloudViz->getZClipMin() : 0.0f; }
    float getZClipMax() const { return m_cloudViz ? m_cloudViz->getZClipMax() : 0.0f; }
    bool  isZClipping()  const { return m_cloudViz ? m_cloudViz->isZClipping()  : false; }
    bool  isAutoColorRange() const { return m_cloudViz ? m_cloudViz->isAutoColorRange() : true; }

    // ========================================================================
    // 外观参数
    // ========================================================================

    /** @brief 设置边线宽度 */
    void setEdgeWidth(float width) {
        m_edgeWidth = width;
        if (m_edgeLineViz) m_edgeLineViz->setLineWidth(width);
    }

    /** @brief 设置球体半径（触发球体重建） */
    void setSphereRadius(float radius) {
        m_sphereRadius = radius;
        if (m_sphereViz && m_lastGraph) {
            rebuildSpheres(m_lastGraph);
        }
    }

    /**
     * @brief 设置渲染采样步长
     *
     * 步长 N > 1 时，仅渲染每第 N 个关键帧的球体，并自动隐藏除新添加边外的所有边线。
     * 点云不受采样影响。步长恢复为 1 时恢复全部渲染。
     *
     * @param stride 采样步长（1 = 全部渲染）
     */
    void setSampleStride(int stride) {
        if (m_sampleStride == stride) return;
        m_sampleStride = stride;

        if (stride > 1 && m_lastGraph) {
            // 采样激活：收集所有当前边 ID 加入采样隐藏集合
            m_sampleHiddenEdgeIds.clear();
            auto& edges = m_lastGraph->graph->edges();
            for (auto it = edges.begin(); it != edges.end(); ++it) {
                if (*it) m_sampleHiddenEdgeIds.insert((*it)->id());
            }
        } else {
            // 采样关闭：清空采样隐藏集合
            m_sampleHiddenEdgeIds.clear();
        }

        // 重建球体（按步长过滤）
        if (m_lastGraph) rebuildSpheres(m_lastGraph);

        // 重建边线（合并用户隐藏 + 采样隐藏）
        if (m_edgeLineViz && m_lastGraph) {
            m_edgeLineViz->rebuild(m_lastGraph.get(), getEffectiveHiddenEdges());
        }
    }

    /** @brief 获取当前采样步长 */
    int sampleStride() const { return m_sampleStride; }

    /**
     * @brief 获取球体中心位置缓存（用于鼠标拾取检测）
     * @return (球心位置, 顶点ID) 对列表
     */
    const std::vector<std::pair<osg::Vec3d, long>>& sphereCenters() const {
        return m_sphereCenters;
    }

    /**
     * @brief 获取边线段列表（用于右键点击拾取检测）
     * @return 边线段数据向量
     */
    const std::vector<EdgeSegment>& edgeSegments() const {
        return m_edgeLineViz ? m_edgeLineViz->edgeSegments() : m_emptySegments;
    }

    /** @brief 获取当前球体半径 */
    float sphereRadius() const { return m_sphereRadius; }

    // ========================================================================
    // 选择与高亮
    // ========================================================================

    /**
     * @brief 设置选中的顶点 ID
     *
     * 选中顶点后，该顶点的球体变为橙色，同时其时序邻居帧的
     * 点云会高亮为白色。
     *
     * @param id 顶点 ID（设为 -1 取消选择）
     */
    void setSelectedVertex(long id) {
        m_selectedVertexId = id;
        if (m_cloudViz) {
            m_cloudViz->recolorHighlight(getTemporalNeighbors(id));
        }
    }

    /** @brief 获取当前选中的顶点 ID */
    long selectedVertex() const { return m_selectedVertexId; }

    /**
     * @brief 轻量级播放高亮（不重建球体几何体）
     *
     * 用于播放轴功能，在滑块拖动或自动播放时调用。
     * 仅更新两个球体的颜色数组（上一个恢复红色，当前变橙色）
     * + 点云高亮，不触发完整的球体几何体重建。
     *
     * 与 setSelectedVertex() 独立管理，两者互不干扰。
     *
     * @param id 顶点 ID（设为 -1 取消高亮）
     */
    void highlightPlaybackVertex(long id) {
        const osg::Vec4 defaultColor(1.0f, 0.0f, 0.0f, 1.0f);   // 红色 —— 默认
        const osg::Vec4 selectedColor(1.0f, 0.8f, 0.0f, 1.0f);  // 橙色 —— 选中

        // 恢复上一个播放高亮球体为默认红色
        if (m_playbackPrevId >= 0 && m_sphereViz) {
            m_sphereViz->updateSphereColor(m_playbackPrevId, defaultColor);
        }
        // 设置新球体为橙色
        if (id >= 0 && m_sphereViz) {
            m_sphereViz->updateSphereColor(id, selectedColor);
        }
        m_playbackPrevId = id;

        // 点云高亮（已很高效，只更新颜色数组）
        if (m_cloudViz) {
            m_cloudViz->recolorHighlight(getTemporalNeighbors(id));
        }
    }

    /**
     * @brief 设置高亮窗口半宽
     *
     * 选中顶点时，其前后各 highlightWindowHalf 个时序邻居帧的
     * 点云会被高亮为白色，总共 (2*half+1) 帧。
     *
     * @param half 半宽大小
     */
    void setHighlightWindowHalf(int half) { m_highlightWindowHalf = half; }

    /** @brief 获取当前高亮窗口半宽 */
    int  highlightWindowHalf() const { return m_highlightWindowHalf; }

    // ========================================================================
    // 场景构建
    // ========================================================================

    /**
     * @brief 从 InteractiveGraph 构建完整场景图
     *
     * 构建顺序：
     *   1. 清除旧的场景数据
     *   2. 重建顶点球体（世界坐标系）
     *   3. 点云 —— 由 ViewportWidget 通过 PointCloudBuilder 在后台线程
     *      构建，完成后经 commitPointCloudBuild() 换入场景（本方法不阻塞）
     *   4. 重建边线段（世界坐标系，按来源着色）
     *
     * @param graph 交互式图数据共享指针
     * @param flags 绘制标志（当前未使用，兼容接口）
     */
    void buildFromGraph(std::shared_ptr<hdl_graph_slam::InteractiveGraph> graph,
                        const hdl_graph_slam::DrawFlags& /*flags*/) {
        clearGraph();

        if (!graph || graph->keyframes.empty()) return;

        m_lastGraph = graph;

        // 1. 球体 —— 世界坐标系，与边使用相同顶点位置
        rebuildSpheres(graph);
        m_sphereGroup->addChild(m_sphereViz->getNode());

        // 2. 点云 —— 后台异步构建（见 ViewportWidget::rebuildPointClouds），
        //    完成后 commitPointCloudBuild() 换入，此处不阻塞主线程

        // 3. 边线 —— 世界坐标系线段，按 EdgeSource 着色
        m_edgeLineViz = std::make_unique<EdgeLineVisualizer>();
        m_edgeLineViz->rebuild(graph.get(), getEffectiveHiddenEdges());
        m_edgeGroup->addChild(m_edgeLineViz->getNode());

        m_hasGraph = true;
    }

    /**
     * @brief 每帧更新球体和边线位置（不更新点云）
     *
     * 点云重建计算密集，由 PointCloudBuilder 在后台线程构建，
     * 完成后通过 commitPointCloudBuild() 换入（见 ViewportWidget）。
     *
     * @param graph 最新的图数据共享指针
     */
    void updatePoses(std::shared_ptr<hdl_graph_slam::InteractiveGraph> graph) {
        if (!graph) return;
        m_lastGraph = graph;

        rebuildSpheres(graph);

        if (m_edgeLineViz) {
            m_edgeLineViz->rebuild(graph.get(), getEffectiveHiddenEdges());
        }
    }

    /**
     * @brief 替换隐藏边的 ID 集合（由 MainWindow 调用）
     * @param ids 需隐藏的边 ID 集合
     */
    void setHiddenEdges(const std::set<long>& ids) {
        m_hiddenEdgeIds = ids;
    }

    /**
     * @brief 设置回环检测高亮
     *
     * 在自动回环检测过程中，将搜索源顶点标记为蓝色，
     * 候选顶点标记为绿色，方便用户观察。
     *
     * @param sourceId      搜索源顶点 ID（蓝色）
     * @param candidateIds  候选顶点 ID 列表（绿色）
     */
    void setLoopHighlight(long sourceId, const std::vector<long>& candidateIds) {
        m_loopSourceId = sourceId;
        m_loopCandidateIds.clear();
        m_loopCandidateIds.insert(candidateIds.begin(), candidateIds.end());
    }

    /**
     * @brief 换入后台构建完成的点云数据（主线程调用）
     *
     * 由 ViewportWidget 在 PointCloudBuilder 后台构建完成后调用。
     * 点云数据经 KeyframePointCloudVisualizer::commitBuild() 以 swap
     * 方式换入（旧几何体持续渲染到新数据就绪，避免闪烁/撕裂），
     * 然后恢复用户当前的 Z 轴裁剪、颜色范围设置与选中/播放高亮。
     *
     * @param result 后台构建结果（右值，内容被交换移入）
     */
    void commitPointCloudBuild(hdl_graph_slam::PointCloudBuildResult&& result) {
        // 首次调用时创建点云可视化器
        if (!m_cloudViz) {
            m_cloudViz = std::make_unique<KeyframePointCloudVisualizer>();
            m_cloudViz->setPointSize(m_pointSize);
            m_cloudViz->setOpacity(m_pointOpacity);
            m_cloudGroup->addChild(m_cloudViz->getNode());
        }

        // 保存用户当前的 Z 轴裁剪和颜色范围设置
        bool  savedZClip   = m_cloudViz->isZClipping();
        float savedClipMin = m_cloudViz->getZClipMin();
        float savedClipMax = m_cloudViz->getZClipMax();
        bool  savedAutoColor = m_cloudViz->isAutoColorRange();
        float savedColorMin  = m_cloudViz->getColorZMin();
        float savedColorMax  = m_cloudViz->getColorZMax();
        bool  firstBuild     = !m_cloudViz->isClipRangeInitialized();

        // 换入新数据
        m_cloudViz->commitBuild(std::move(result));

        // 恢复用户的 Z 轴裁剪和颜色范围设置
        m_cloudViz->setZClipping(savedZClip);
        // 首次构建时裁剪范围已由 commitBuild 初始化为数据范围，无需覆盖
        if (!firstBuild) {
            m_cloudViz->setZClipRange(savedClipMin, savedClipMax);
        }
        if (savedAutoColor) {
            m_cloudViz->setAutoColorRange(true);
        } else {
            m_cloudViz->setColorZRange(savedColorMin, savedColorMax);
        }

        // 恢复选中/播放高亮；无高亮状态时清除高亮并恢复默认着色
        if (m_selectedVertexId >= 0) {
            m_cloudViz->recolorHighlight(getTemporalNeighbors(m_selectedVertexId));
        } else if (m_playbackPrevId >= 0) {
            m_cloudViz->recolorHighlight(getTemporalNeighbors(m_playbackPrevId));
        } else {
            m_cloudViz->clearHighlight();
        }
    }

    /** @brief 是否已有点云可视化器（是否有已构建/构建中的点云） */
    bool hasPointCloud() const { return m_cloudViz != nullptr; }

    /** @brief 最近一次构建场景所用的图（供后台点云构建使用） */
    std::shared_ptr<hdl_graph_slam::InteractiveGraph> lastGraph() const { return m_lastGraph; }

    /** @brief 清除整个场景 */
    void clear() {
        clearGraph();
    }

private:
    /**
     * @brief 合并用户手动隐藏边 + 采样自动隐藏边，返回有效的隐藏边集
     *
     * 采样未激活时直接返回用户隐藏集合，避免不必要的拷贝。
     * 采样激活时合并两个集合，确保：
     *   - 用户通过 EdgeListPanel 隐藏的边持续隐藏
     *   - 采样前已存在的边全部隐藏
     *   - 新添加的回环边（不在任一集合中）可见
     *
     * @return 合并后的隐藏边 ID 集合
     */
    std::set<long> getEffectiveHiddenEdges() const {
        if (m_sampleStride <= 1) return m_hiddenEdgeIds;
        std::set<long> combined = m_hiddenEdgeIds;
        combined.insert(m_sampleHiddenEdgeIds.begin(), m_sampleHiddenEdgeIds.end());
        return combined;
    }

    /**
     * @brief 重建顶点位姿标记
     *
     * 为每个关键帧创建一个定向标记，位置取顶点平移估计值，
     * 方向取关键帧局部位姿的旋转（右-下-前坐标系，X右/Y下/Z前），
     * 轴向始终沿局部 +Z（前方）。形状取决于状态：
     *   - 普通顶点：截锥体（红色，切掉尖顶）
     *   - 选中 / 播放高亮：尖锥体（黄色，2 倍尺寸）
     *   - 回环搜索源：蓝色截锥体
     *   - 回环候选：绿色截锥体
     *
     * 当采样步长 > 1 时，仅渲染 id % stride == 0 的标记，
     * 但特殊标记（选中、回环）始终渲染。
     *
     * @param graph 交互式图数据
     */
    void rebuildSpheres(std::shared_ptr<hdl_graph_slam::InteractiveGraph> graph) {
        if (!m_sphereViz) {
            m_sphereViz = std::make_unique<VertexSphereVisualizer>(m_sphereRadius);
        }
        m_sphereViz->clear();
        m_sphereViz->setRadius(m_sphereRadius);

        // 清空并预标记中心缓存
        m_sphereCenters.clear();
        m_sphereCenters.reserve(graph->keyframes.size());

        // 定义不同状态的标记颜色
        const osg::Vec4 defaultColor(1.0f, 0.0f, 0.0f, 1.0f);    // 红色 —— 默认
        const osg::Vec4 selectedColor(1.0f, 0.8f, 0.0f, 1.0f);   // 黄色 —— 选中
        const osg::Vec4 loopSourceColor(0.0f, 0.0f, 1.0f, 1.0f); // 蓝色 —— 回环搜索源
        const osg::Vec4 loopCandColor(0.0f, 1.0f, 0.0f, 1.0f);   // 绿色 —— 回环候选

        // 遍历所有关键帧，创建顶点位姿标记
        for (auto& [id, kf] : graph->keyframes) {
            auto* v = dynamic_cast<g2o::VertexSE3*>(kf->node);
            if (!v) continue;

            // 采样过滤：仅渲染 id % stride == 0 的关键帧
            // 但特殊标记（选中、播放高亮、回环源、回环候选）始终渲染，绕过采样
            bool isSpecial = (id == m_selectedVertexId ||
                              id == m_playbackPrevId ||
                              id == m_loopSourceId ||
                              m_loopCandidateIds.count(id));
            if (m_sampleStride > 1 && !isSpecial && (id % m_sampleStride != 0)) continue;

            Eigen::Isometry3d pose = v->estimate();
            Eigen::Vector3d pos = pose.translation();
            osg::Vec3d center(pos.x(), pos.y(), pos.z());

            // 关键帧局部位姿的旋转（右-下-前：X右/Y下/Z前），
            // 标记尖端沿局部 +Z（前方）方向
            Eigen::Matrix3f rot = pose.linear().cast<float>();

            // 缓存标记中心位置（用于鼠标拾取）—— 仅采样后的标记
            m_sphereCenters.emplace_back(center, id);

            // 根据状态选择颜色、形状和尺寸
            osg::Vec4 color = defaultColor;
            float customRadius = -1.0f;  // < 0 表示使用全局默认尺寸
            bool useCone = false;        // 选中/播放高亮使用尖锥体，其余使用截锥体
            if (id == m_selectedVertexId) {
                color = selectedColor;
                customRadius = m_sphereRadius * 2.0f;
                useCone = true;
            } else if (id == m_loopSourceId) {
                color = loopSourceColor;
            } else if (m_loopCandidateIds.count(id)) {
                color = loopCandColor;
            } else if (id == m_playbackPrevId) {
                // 播放轴高亮标记同样放大 2 倍（仅在完整重建时生效）
                // 轻量级 highlightPlaybackVertex 路径仅更新颜色，不做几何重建
                color = selectedColor;
                customRadius = m_sphereRadius * 2.0f;
                useCone = true;
            }

            if (useCone) {
                m_sphereViz->appendCone(center, rot, color, id, customRadius);
            } else {
                m_sphereViz->appendTruncatedCone(center, rot, color, id, customRadius);
            }
        }
        m_sphereViz->finish();
    }

    /**
     * @brief 清除所有动态场景元素
     *
     * 移除球体组、边组、点云组中的所有子节点，
     * 重置所有子可视化器智能指针。
     */
    void clearGraph() {
        m_sphereGroup->removeChildren(0, m_sphereGroup->getNumChildren());
        m_edgeGroup->removeChildren(0, m_edgeGroup->getNumChildren());
        m_cloudGroup->removeChildren(0, m_cloudGroup->getNumChildren());
        m_sphereViz.reset();
        m_cloudViz.reset();
        m_edgeLineViz.reset();
        m_hasGraph = false;
    }

    /**
     * @brief 获取指定顶点的时序邻居顶点 ID 集合
     *
     * 以 centerId 为中心，在 ±m_highlightWindowHalf 范围内
     * 收集所有存在的顶点 ID。用于点云高亮显示。
     *
     * 当 centerId < 0（取消选择）或无图数据时返回空集合。
     *
     * @param centerId 中心顶点 ID
     * @return 邻居顶点 ID 集合
     */
    std::set<long> getTemporalNeighbors(long centerId) const {
        std::set<long> result;
        if (!m_lastGraph || centerId < 0) return result;

        auto itCenter = m_lastGraph->keyframes.find(centerId);
        if (itCenter == m_lastGraph->keyframes.end()) {
            result.insert(centerId);
            return result;
        }

        // 通过 ID 范围收集顶点（与 mergeAdjacentClouds 算法相同）
        long first = centerId - m_highlightWindowHalf;
        long last  = centerId + m_highlightWindowHalf;
        for (long id = first; id <= last; ++id) {
            if (m_lastGraph->keyframes.find(id) != m_lastGraph->keyframes.end()) {
                result.insert(id);
            }
        }
        return result;
    }

    // —— 场景根节点 ——
    osg::ref_ptr<osg::Group> m_root;  ///< 场景根节点

    // —— 静态元素 ——
    std::unique_ptr<CoordinateAxesVisualizer> m_axes;  ///< 坐标轴可视化器
    std::unique_ptr<GroundGridVisualizer>  m_grid;     ///< 地面网格可视化器

    // —— 动态子分组（用于可见性独立开关） ——
    osg::ref_ptr<osg::Group> m_sphereGroup;  ///< 球体子分组
    osg::ref_ptr<osg::Group> m_edgeGroup;    ///< 边线子分组
    osg::ref_ptr<osg::Group> m_cloudGroup;   ///< 点云子分组

    // —— 动态可视化器（世界坐标系几何体） ——
    std::unique_ptr<VertexSphereVisualizer>       m_sphereViz;  ///< 顶点球体可视化器
    std::unique_ptr<KeyframePointCloudVisualizer> m_cloudViz;   ///< 关键帧点云可视化器
    std::unique_ptr<EdgeLineVisualizer>           m_edgeLineViz; ///< 边线可视化器

    // —— 缓存数据 ——
    std::shared_ptr<hdl_graph_slam::InteractiveGraph> m_lastGraph;  ///< 缓存的图引用（用于实时参数更改）
    std::vector<std::pair<osg::Vec3d, long>> m_sphereCenters;  ///< 球心缓存（与 VBO 并行，重建时刷新）
    mutable std::vector<EdgeSegment> m_emptySegments;  ///< 无边时的空向量返回（备用）

    // —— 交互状态 ——
    std::set<long> m_hiddenEdgeIds;        ///< 用户通过 EdgeListPanel 隐藏的边 ID 集合
    long m_loopSourceId = -1;              ///< 回环检测搜索源顶点 ID
    std::set<long> m_loopCandidateIds;     ///< 回环检测候选顶点 ID 集合
    long m_selectedVertexId = -1;          ///< 当前选中的顶点 ID（-1 表示无选中）
    long m_playbackPrevId = -1;            ///< 播放轴上一个高亮的顶点 ID（用于恢复颜色）
    int  m_highlightWindowHalf = 1;        ///< 高亮窗口半宽（与 m_submapWindowHalfSize 一致）

    // —— 渲染采样 ——
    int  m_sampleStride = 1;               ///< 渲染采样步长（1=全部, N=每N帧渲染1个球体）
    std::set<long> m_sampleHiddenEdgeIds;  ///< 采样激活时自动隐藏的边 ID（采样前已存在的边）

    // —— 状态配置 ——
    bool m_hasGraph    = false;   ///< 是否有已加载的图数据
    bool m_drawClouds  = true;    ///< 是否绘制点云
    float m_sphereRadius  = 1.0f; ///< 球体半径
    float m_edgeWidth     = 2.0f; ///< 边线宽度（像素）
    float m_pointSize     = 3.0f; ///< 点云点大小（像素）
    float m_pointOpacity  = 1.0f; ///< 点云透明度（1.0 为不透明）
    bool  m_lodEnabled    = true; ///< LOD 多级渲染开关（渲染固定为全量 + LOD）
};
