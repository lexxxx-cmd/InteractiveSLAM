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
#include <cstdint>
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
#include "visualizers/SpherePickingHandler.h"

/**
 * @brief 图场景可视化器 —— 场景图构建和管理的中央协调器
 *
 * 核心职责：
 *   1. 从 InteractiveGraph 构建完整的 OSG 场景图
 *   2. 管理所有子可视化器（球体、边线、点云、坐标轴、地面网格）
 *   3. 提供可见性开关（顶点、边、点云各自独立控制）
 *   4. 支持点云 Z 轴裁剪和颜色范围控制
 *   5. 支持顶点选择高亮（高亮选中顶点附近的时序邻居帧）
 *   6. 支持回环检测可视化（搜索源为深蓝、候选为品红）
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

    /**
     * @brief 设置原始层（里程计位姿参照底图）开关
     *
     * 打开后显示按里程计位姿变换的点云（淡橙色半透明），
     * 与优化层叠加对比优化前后的差异。首次打开会触发点云
     * 后台重建（生成原始层数据），之后仅做可见性切换。
     */
    void setOdomLayerEnabled(bool enabled) {
        m_odomLayerEnabled = enabled;
        if (m_cloudViz) m_cloudViz->setOdomLayerVisible(enabled);
    }

    /** @brief 查询原始层开关状态 */
    bool odomLayerEnabled() const { return m_odomLayerEnabled; }

    /** @brief 原始层几何体是否已就绪（调用方据此判断是否需要请求构建） */
    bool odomLayerReady() const {
        return m_cloudViz ? m_cloudViz->odomLayerReady() : false;
    }

    /**
     * @brief 已换入原始层的内容签名（0 = 尚未构建过）
     *
     * 原始层内容在逻辑上冻结：调用方算出当前内容签名与它比对，相同即表示
     * 无需重新生成原始层，本次构建可以继续复用已换入的几何体。
     */
    uint64_t committedOdomSignature() const {
        return m_cloudViz ? m_cloudViz->committedOdomSignature() : 0;
    }

    /**
     * @brief 原地更新原始层透明度（O(1)，不重建）
     *
     * 原始层颜色是常量，所有分块共享同一个 1 元素颜色数组，改透明度只需
     * 更新这一个元素。
     */
    void setOriginalLayerOpacity(float opacity) {
        if (m_cloudViz) m_cloudViz->setOriginalLayerOpacity(opacity);
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
     * 开启后 PointCloudBuilder 在后台构建第一层降采样（目标点数 N/2），
     * ViewportWidget 按相机距离自动切换级别或手动固定层级：
     * 近处全量细节，远处低分辨率轮廓，减少远距离顶点处理量。
     * 渲染固定为"全量 + 第一层"两种（点预算档位已移除）。
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

    /** @brief 设置是否绘制视锥体局部坐标轴（调试用，立即重建标记） */
    void setDrawLocalAxes(bool on) {
        if (m_drawLocalAxes == on) return;
        m_drawLocalAxes = on;
        if (m_sphereViz && m_lastGraph) {
            rebuildSpheres(m_lastGraph);
        }
    }

    /** @brief 查询局部坐标轴开关状态 */
    bool drawLocalAxes() const { return m_drawLocalAxes; }

    /** @brief 设置顶点位姿标记的整体不透明度（0.0 全透明 ~ 1.0 不透明） */
    void setVertexOpacity(float opacity) {
        m_vertexOpacity = opacity;
        m_focusedVertexId = -1;  // 手动调透明度视为退出聚焦淡化状态
        if (m_sphereViz) {
            m_sphereViz->setOpacity(opacity);
        }
    }

    /**
     * @brief 双击聚焦淡化：全体标记压到最低不透明度，目标标记稍高
     *
     * 再次调用传入新 ID 时自动切换目标；传 -1（如双击空白处）恢复
     * 用户设置的整体不透明度。
     *
     * @param id 聚焦的顶点 ID（-1 = 取消聚焦淡化）
     */
    void setFocusedVertex(long id) {
        if (!m_sphereViz) return;
        if (id < 0) {
            // 取消：恢复用户设置的整体不透明度
            m_focusedVertexId = -1;
            m_sphereViz->setOpacity(m_vertexOpacity);
            return;
        }
        m_focusedVertexId = id;
        m_sphereViz->setOpacity(kFocusDimOpacity);
        m_sphereViz->updateSphereOpacity(id, kFocusTargetOpacity);
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
     * @brief 获取可拾取标记缓存（用于鼠标拾取检测）
     * @return (标记中心, 顶点ID, 拾取半径) 列表
     */
    const std::vector<PickableCenter>& sphereCenters() const {
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
     * 选中顶点后，该顶点的视锥体放大为 2 倍并变为红色系，
     * 同时其时序邻居帧的点云会高亮为白色。
     *
     * @param id 顶点 ID（设为 -1 取消选择）
     */
    void setSelectedVertex(long id) {
        m_selectedVertexId = id;
        syncCloudHighlight();
    }

    /** @brief 获取当前选中的顶点 ID */
    long selectedVertex() const { return m_selectedVertexId; }

    /**
     * @brief 轻量级播放高亮（不重建视锥体几何体）
     *
     * 播放通道的逻辑只有三步：更新 m_playbackPrevId 状态，然后对
     * "上一帧"与"当前帧"两个标记按 markerStyleFor() 重算并落地样式
     * （颜色 + 尺寸 + 拾取半径一次同步生效），最后更新点云高亮。
     *
     * 样式由"状态 → 样式"的纯函数唯一推导（markerStyleFor 是唯一
     * 事实来源，与 rebuildSpheres 共用），因此任何状态组合
     * （普通/播放/选中/回环）下颜色与尺寸都不可能错位，也无需
     * 针对选中帧、恢复色等写特判分支——上一帧恢复成什么样完全由
     * 它当前的状态决定。
     *
     * 三种视觉状态：
     *   - 普通帧：绿色、1 倍
     *   - 播放高亮：红色、1 倍（连续播放中的小高亮）
     *   - 选中帧：红色、2 倍（仅存在于无播放会话时，见下方接管语义）
     *
     * 播放通道接管标记高亮：会话开始/推进（id >= 0）时清除上一轮
     * 遗留的选中高亮（按普通态轻量恢复，不触发重建），避免旧的大红
     * 标记残留在场景中、播放划过它时被二次放大。会话清理（id = -1）
     * 不动选中态——暂停路径随后的 selectVertex() 会建立新选中。
     *
     * 与 setSelectedVertex() 分工：播放通道只在播放会话期间生效，
     * 会话结束（暂停/单步/播完/关面板/关图/外部选择）由调用方传
     * id = -1 清理，视觉交还给"选中"高亮，避免两个红色标记并存。
     *
     * @param id 顶点 ID（-1 = 清理播放通道：恢复上一个播放帧的视觉样式
     *            并保留已建立的留存前缀，不动其他点云高亮）
     */
    void highlightPlaybackVertex(long id) {
        long prev = m_playbackPrevId;
        long oldSelected = m_selectedVertexId;
        m_playbackPrevId = id;  // 先更新状态，再由状态推导标记样式
        if (id >= 0) m_selectedVertexId = -1;  // 播放接管：清除旧选中

        applyMarkerState(oldSelected);  // 旧选中恢复普通态（若已被清除）
        applyMarkerState(prev);
        applyMarkerState(id);

        // 点云高亮：由状态推导（唯一事实来源）
        // 会话结束（id < 0）不清前缀——留存高亮由 m_retainedPrefixEndId 承载，
        // 随后的 selectVertex 只负责把当前帧切为选中态（红 2 倍标记）。
        if (id < 0) {
            syncCloudHighlight();
            return;
        }
        // 会话进行中：留存模式把前缀终点推进到当前帧。前缀范围 = f(终点)，
        // 因此前进时增长、后退（倒放/倒拖滑块/跳回首帧）时自动缩小回退。
        if (m_playbackRetain) m_retainedPrefixEndId = id;
        syncCloudHighlight();
    }

    /**
     * @brief 设置播放累积高亮开关（"播放留存已播放点云"）
     *
     * 打开后，播放/拖动过程中**所有 id <= 当前帧的关键帧**整帧点云保持
     * 白色高亮（前缀语义），历史帧不随播放推进消失，形成"已播放区域
     * 留存"的视觉效果；终点回退时高亮同步缩小。
     *
     * 与旧实现（只累加"播放经过的帧"）的区别：高亮范围完全由当前帧
     * 推导，与播放路径无关——倒拖滑块、跳转回首帧都能正确回退。
     *
     * 关闭时立即丢弃留存前缀，恢复为仅高亮当前帧的时序邻居。
     * 会话结束后前缀**保留**（点云继续显示已播放区域），直到关闭本
     * 开关、关闭面板（复选框复位）或重新加载地图。
     *
     * @param retain true = 前缀留存模式；false = 仅当前帧高亮（默认）
     */
    void setPlaybackRetain(bool retain) {
        if (m_playbackRetain == retain) return;
        m_playbackRetain = retain;
        if (!retain) {
            m_retainedPrefixEndId = -1;   // 关闭：丢弃留存前缀
        } else if (m_playbackPrevId >= 0) {
            // 播放会话中打开：立即点亮已播放前缀（旧实现需等下一帧才生效）
            m_retainedPrefixEndId = m_playbackPrevId;
        }
        syncCloudHighlight();
    }

    /** @brief 查询播放累积高亮开关状态 */
    bool playbackRetain() const { return m_playbackRetain; }

private:
    /**
     * @brief 依当前交互状态推导点云高亮并落地（点云高亮的唯一事实来源）
     *
     * 三种状态组合：
     *   - 留存前缀（m_playbackRetain 且已播放过）：所有 id <= 前缀终点的
     *     关键帧整帧染白；选中帧的时序邻居作为"集合语义"额外叠加
     *     （继续享受独立几何体的全量白色）；
     *   - 选中优先：无前缀留存时，高亮选中帧的时序邻居；
     *   - 播放兜底：无选中时，高亮播放通道当前帧的时序邻居；
     *   - 三者皆无：清除高亮，恢复默认高程着色。
     *
     * 所有会改变高亮状态的入口（选中、播放推进、留存开关、点云重建）
     * 都只改状态再调用本函数，避免多头发散出不一致的高亮。
     */
    void syncCloudHighlight() {
        if (!m_cloudViz) return;

        if (m_playbackRetain && m_retainedPrefixEndId >= 0) {
            std::set<long> extra;
            // 选中帧落在前缀之外（如 Ctrl+Click 选了后面的帧）才额外叠加其
            // 时序邻居；落在前缀内则不加，避免边界处多亮出 end+1 一帧，
            // 使"高亮的永远是 <= 当前帧"这一语义保持严格。
            if (m_selectedVertexId > m_retainedPrefixEndId)
                extra = getTemporalNeighbors(m_selectedVertexId);
            m_cloudViz->recolorHighlightPrefix(m_retainedPrefixEndId, extra);
            return;
        }

        std::set<long> ids;
        if (m_selectedVertexId >= 0)      ids = getTemporalNeighbors(m_selectedVertexId);
        else if (m_playbackPrevId >= 0)   ids = getTemporalNeighbors(m_playbackPrevId);

        if (ids.empty()) m_cloudViz->clearHighlight();
        else             m_cloudViz->recolorHighlight(ids);
    }

    /**
     * @brief 单个标记的视觉样式（颜色 + 尺寸缩放）
     */
    struct MarkerStyle {
        osg::Vec4 color;  ///< 基色（着色层再做线框提亮与纵向渐变）
        float scale;      ///< 尺寸缩放（1 = 全局默认，2 = 选中放大）
    };

    /**
     * @brief 单个标记的视觉样式 = f(交互状态) —— 唯一事实来源
     *
     * rebuildSpheres（全量重建）与 applyMarkerState（轻量路径）都从
     * 这里取样式，两条路径的视觉语义由同一份代码保证，永不脱节。
     * 优先级：选中 > 回环源 > 回环候选 > 播放高亮 > 默认。
     */
    MarkerStyle markerStyleFor(long id) const {
        const osg::Vec4 defaultColor(0.25f, 0.80f, 0.30f, 1.0f);  // 绿 —— 默认
        const osg::Vec4 selectedColor(1.0f, 0.20f, 0.20f, 1.0f);  // 红 —— 选中/播放高亮
        const osg::Vec4 loopSourceColor(0.0f, 0.0f, 1.0f, 1.0f);  // 深蓝 —— 回环搜索源
        const osg::Vec4 loopCandColor(1.0f, 0.25f, 0.75f, 1.0f);  // 品红 —— 回环候选
        constexpr float kNormalScale   = 1.0f;
        constexpr float kSelectedScale = 2.0f;

        if (id == m_selectedVertexId)     return {selectedColor, kSelectedScale};
        if (id == m_loopSourceId)         return {loopSourceColor, kNormalScale};
        if (m_loopCandidateIds.count(id)) return {loopCandColor, kNormalScale};
        if (id == m_playbackPrevId)       return {selectedColor, kNormalScale};
        return {defaultColor, kNormalScale};
    }

    /**
     * @brief 按当前交互状态把某标记的样式落地到场景（轻量路径）
     *
     * 颜色、尺寸、拾取半径三者一次同步更新；id < 0 或标记不存在时
     * 静默跳过。配合"先改状态、后调本函数"的次序使用。
     */
    void applyMarkerState(long id) {
        if (id < 0 || !m_sphereViz) return;
        const MarkerStyle st = markerStyleFor(id);
        m_sphereViz->updateSphereColor(id, st.color);
        m_sphereViz->updateSphereScale(id, st.scale);
        updatePickRadius(id, st.scale);
    }

    /** @brief 同步可拾取缓存中指定标记的拾取半径（与样式缩放成比例） */
    void updatePickRadius(long id, float scale) {
        for (auto& c : m_sphereCenters) {
            if (c.vertexId == id) {
                c.pickRadius = 2.5f * m_sphereRadius * scale;
                break;
            }
        }
    }

public:

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
            // 原始层 geode 挂到同一组（可见性由 geode 自身 NodeMask 控制）
            m_cloudGroup->addChild(m_cloudViz->getOdomNode());
            m_cloudViz->setOdomLayerVisible(m_odomLayerEnabled);
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

        // 恢复选中/播放/留存高亮；无高亮状态时清除高亮并恢复默认着色
        syncCloudHighlight();
    }

    /** @brief 是否已有点云可视化器（是否有已构建/构建中的点云） */
    bool hasPointCloud() const { return m_cloudViz != nullptr; }

    /** @brief 最近一次构建场景所用的图（供后台点云构建使用） */
    std::shared_ptr<hdl_graph_slam::InteractiveGraph> lastGraph() const { return m_lastGraph; }

    /**
     * @brief 最近一次点云落地的规模与上传统计（Phase 0 基线测量用）
     *
     * 未创建点云可视化器时返回全零统计，调用方无需判空。
     */
    KeyframePointCloudVisualizer::CommitStats lastCloudCommitStats() const {
        return m_cloudViz ? m_cloudViz->lastCommitStats()
                          : KeyframePointCloudVisualizer::CommitStats{};
    }

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
     * 为每个关键帧创建一个相机视锥体标记：锥顶位于顶点平移估计值
     * （相机光心），方向取关键帧局部位姿的旋转
     * （右-下-前坐标系，X右/Y下/Z前），沿局部 +Z（前方）展开。
     * 颜色与尺寸由 markerStyleFor(id) 统一推导（唯一事实来源，
     * 与轻量路径 applyMarkerState 同源）：
     *   - 普通顶点：绿色系、1 倍
     *   - 播放高亮：红色系、1 倍
     *   - 选中：红色系、2 倍
     *   - 回环搜索源：深蓝色
     *   - 回环候选：品红
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
        m_sphereViz->setOpacity(m_focusedVertexId >= 0 ? kFocusDimOpacity
                                                       : m_vertexOpacity);
        m_sphereViz->setDrawLocalAxes(m_drawLocalAxes);

        // 清空并预标记中心缓存
        m_sphereCenters.clear();
        m_sphereCenters.reserve(graph->keyframes.size());

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

            // 颜色和尺寸由统一的状态→样式映射给出（与轻量路径
            // applyMarkerState 同源，重建后所有标记样式即当前状态）；
            // scale > 1 的选中帧按比例放大几何体
            const MarkerStyle style = markerStyleFor(id);
            float customRadius = (style.scale > 1.0f)
                                     ? m_sphereRadius * style.scale
                                     : -1.0f;  // < 0 表示使用全局默认尺寸

            // 缓存可拾取标记（用于鼠标拾取）—— 仅采样后的标记；
            // 视锥体远平面四角距锥顶最远约 2.3 倍特征尺寸
            // （深度 2R + 侧向偏移），拾取半径取 2.5 倍保证整个
            // 视锥体可见部分均可点中；高亮放大的标记（2 倍尺寸）
            // 自动获得成比例的拾取半径
            float renderRadius = (customRadius > 0.0f) ? customRadius : m_sphereRadius;
            m_sphereCenters.push_back({center, id, 2.5f * renderRadius});

            m_sphereViz->appendFrustum(center, rot, style.color, id, customRadius);
        }
        // 聚焦淡化状态：目标标记在重建后仍保持稍高的不透明度
        if (m_focusedVertexId >= 0) {
            m_sphereViz->updateSphereOpacity(m_focusedVertexId, kFocusTargetOpacity);
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

        // 重置交互状态：顶点 ID 通常从 0 开始，旧地图的选中/播放/聚焦
        // ID 残留到新地图几乎必然命中某个无辜顶点，导致其顶着
        // 红色 2 倍高亮 / 聚焦淡化出现
        m_selectedVertexId = -1;
        m_playbackPrevId   = -1;
        m_focusedVertexId  = -1;
        m_retainedPrefixEndId = -1;  // 上一张地图的留存前缀不可跨地图沿用
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
    std::vector<PickableCenter> m_sphereCenters;  ///< 可拾取标记缓存（与 VBO 并行，重建时刷新）
    mutable std::vector<EdgeSegment> m_emptySegments;  ///< 无边时的空向量返回（备用）

    // —— 交互状态 ——
    std::set<long> m_hiddenEdgeIds;        ///< 用户通过 EdgeListPanel 隐藏的边 ID 集合
    long m_loopSourceId = -1;              ///< 回环检测搜索源顶点 ID
    std::set<long> m_loopCandidateIds;     ///< 回环检测候选顶点 ID 集合
    long m_selectedVertexId = -1;          ///< 当前选中的顶点 ID（-1 表示无选中）
    long m_playbackPrevId = -1;            ///< 播放轴上一个高亮的顶点 ID（用于恢复颜色）
    long m_focusedVertexId = -1;           ///< 双击聚焦淡化的目标顶点 ID（-1 = 未聚焦）
    int  m_highlightWindowHalf = 1;        ///< 高亮窗口半宽（与 m_submapWindowHalfSize 一致）

    // —— 聚焦淡化参数 ——
    static constexpr float kFocusDimOpacity    = 0.10f;  ///< 聚焦时全体标记的最低不透明度
    static constexpr float kFocusTargetOpacity = 0.35f;  ///< 聚焦目标标记的稍高不透明度

    // —— 渲染采样 ——
    int  m_sampleStride = 1;               ///< 渲染采样步长（1=全部, N=每N帧渲染1个球体）
    std::set<long> m_sampleHiddenEdgeIds;  ///< 采样激活时自动隐藏的边 ID（采样前已存在的边）

    // —— 状态配置 ——
    bool m_hasGraph    = false;   ///< 是否有已加载的图数据
    bool m_drawClouds  = true;    ///< 是否绘制点云
    bool m_drawLocalAxes = false;  ///< 是否绘制视锥体局部坐标轴（调试用）
    float m_sphereRadius  = 0.1f; ///< 球体半径
    float m_vertexOpacity = 1.0f; ///< 顶点位姿标记不透明度（1.0 不透明）
    float m_edgeWidth     = 2.0f; ///< 边线宽度（像素）
    float m_pointSize     = 2.0f; ///< 点云点大小（像素）
    float m_pointOpacity  = 1.0f; ///< 点云透明度（1.0 为不透明）
    bool  m_lodEnabled    = true; ///< LOD 渲染开关（渲染固定为全量 + 第一层两种）
    bool  m_odomLayerEnabled = false; ///< 原始层（里程计位姿参照底图）开关（默认关闭）
    bool  m_playbackRetain   = false; ///< 播放留存开关（历史帧点云前缀高亮）
    long  m_retainedPrefixEndId = -1; ///< 留存前缀的终点帧 ID（-1 = 无留存；会话结束后仍保留）
};
