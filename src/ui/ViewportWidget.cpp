/**
 * @file ViewportWidget.cpp
 * @brief 3D 视口部件实现
 *
 * 实现 ViewportWidget 的所有功能，包括：
 * - OSG 初始化：摄像机设置、场景图挂载、拾取处理器注册
 * - 透视/正交投影切换：支持动态跟踪正交投影半高（基于摄像机距离）
 * - 图谱加载与重置：onGraphLoaded/onGraphClosed
 * - 渲染控制接口：顶点/边/点云可见性、球体大小、线宽等
 * - 定时更新：约 60Hz 的 updateScene 用于 FPS 统计和正交投影动态更新
 * - 顶点拾取：Ctrl+Click 选中高亮，右键弹出上下文菜单
 * - 浮动面板管理：注册/定位叠加面板
 */

#include "ui/ViewportWidget.h"

#include <QVBoxLayout>
#include <QColor>
#include <QKeyEvent>
#include <QResizeEvent>
#include <QtConcurrent/QtConcurrent>
#include <chrono>
#include <cmath>
#include <cstdint>

#include <osg/Notify>
#include <osg/Math>
#include <osgGA/TrackballManipulator>

#include <Eigen/Geometry>

#include "osgQOpenGL/osgQOpenGLWidget.h"
#include "osgQOpenGL/OSGRenderer.h"
#include "backend/graph_manager.hpp"
#include "visualizers/SpherePickingHandler.h"
#include "visualizers/PointCloudBuilder.h"
#include "visualizers/CloudPerfLog.h"
#include "ui/FirstPersonManipulator.h"
#include "ui/OverlayPanelWidget.h"

// ---------------------------------------------------------------------------
// 构造 / 析构
// ---------------------------------------------------------------------------

/**
 * @brief 构造函数
 *
 * 创建 OSG OpenGL 嵌入部件，初始化场景可视化器，
 * 连接 OSG 初始化信号，启动 16ms 定时器（~60Hz）用于场景更新。
 */
ViewportWidget::ViewportWidget(QWidget* parent)
    : QWidget(parent) {

    // 无边距布局填充整个可用区域
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // 创建 OSG 嵌入部件
    m_osgWidget = new osgQOpenGLWidget(this);
    layout->addWidget(m_osgWidget);
    // 键盘焦点：第一人称模式（Shift + WASD）需要接收键盘事件
    m_osgWidget->setFocusPolicy(Qt::StrongFocus);
    m_osgWidget->installEventFilter(this);

    // 初始化场景可视化器
    m_sceneViz = std::make_unique<GraphSceneVisualizer>();

    // OSG 初始化就绪信号（OpenGL 上下文创建后触发）
    connect(m_osgWidget, &osgQOpenGLWidget::initialized,
            this, &ViewportWidget::initOsg);

    // 姿态更新定时器（~60 Hz）
    m_updateTimer = new QTimer(this);
    connect(m_updateTimer, &QTimer::timeout, this, &ViewportWidget::updateScene);
    m_updateTimer->start(16);

    // 后台点云构建监视器：构建完成在主线程换入场景
    m_cloudBuildWatcher =
        new QFutureWatcher<hdl_graph_slam::PointCloudBuildResult>(this);
    connect(m_cloudBuildWatcher, &QFutureWatcher<hdl_graph_slam::PointCloudBuildResult>::finished,
            this, &ViewportWidget::onCloudBuildFinished);
}

ViewportWidget::~ViewportWidget() = default;

// ---------------------------------------------------------------------------
// 正交投影辅助
// ---------------------------------------------------------------------------

/**
 * @brief 应用正交投影
 *
 * 根据场景包围球半径计算正交投影矩阵的半高（带 20% 边距）。
 * 如果场景未加载，使用默认值 10.0。
 */
void ViewportWidget::applyOrthographicProjection() {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;

    osg::Camera* camera = viewer->getCamera();
    int vpW = m_osgWidget->width();
    int vpH = m_osgWidget->height();
    if (vpW < 1 || vpH < 1) return;

    double aspect = static_cast<double>(vpW) / static_cast<double>(vpH);

    // 从场景包围球计算半高（带 20% 边距）
    double halfHeight = 10.0;  // 场景加载前的默认值
    osg::Node* scene = viewer->getSceneData();
    if (scene) {
        const osg::BoundingSphere& bs = scene->getBound();
        if (bs.valid() && bs.radius() > 0.0) {
            halfHeight = bs.radius() * 1.2;
        }
    }

    double halfWidth = halfHeight * aspect;
    double farDist = halfHeight * 20.0;  // 慷慨的远平面

    camera->setProjectionMatrixAsOrtho(
        -halfWidth, halfWidth,
        -halfHeight, halfHeight,
        0.1, farDist);

    m_orthoHalfHeight = halfHeight;
}

// ---------------------------------------------------------------------------
// 透视投影辅助
// ---------------------------------------------------------------------------

/**
 * @brief 应用透视投影
 *
 * 设置 30° FOV，远平面根据场景包围球半径动态计算。
 */
void ViewportWidget::applyPerspectiveProjection() {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;

    osg::Camera* camera = viewer->getCamera();
    int vpW = m_osgWidget->width();
    int vpH = m_osgWidget->height();
    if (vpW < 1 || vpH < 1) return;

    double aspect = static_cast<double>(vpW) / static_cast<double>(vpH);
    double fovY  = 30.0;  // 度

    // 根据场景边界计算远平面
    double farDist = 1000.0;
    osg::Node* scene = viewer->getSceneData();
    if (scene) {
        const osg::BoundingSphere& bs = scene->getBound();
        if (bs.valid() && bs.radius() > 0.0) {
            farDist = bs.radius() * 20.0;
        }
    }

    camera->setProjectionMatrixAsPerspective(fovY, aspect, 0.1, farDist);
}

// ---------------------------------------------------------------------------
// 统一投影分发
// ---------------------------------------------------------------------------

/**
 * @brief 根据 m_useOrthographic 标志应用对应的投影方式
 */
void ViewportWidget::applyProjection() {
    if (m_useOrthographic) {
        applyOrthographicProjection();
    } else {
        applyPerspectiveProjection();
    }
}

void ViewportWidget::setHighlightWindowHalf(int n) {
    m_sceneViz->setHighlightWindowHalf(n);
}

void ViewportWidget::setUseOrthographic(bool enabled) {
    if (m_useOrthographic == enabled) return;
    m_useOrthographic = enabled;
    applyProjection();
    m_osgWidget->update();
}

// ---------------------------------------------------------------------------
// OSG 初始化（OpenGL 上下文准备就绪后调用一次）
// ---------------------------------------------------------------------------

/**
 * @brief OSG 初始化
 *
 * 在 osgQOpenGLWidget 的 OpenGL 上下文创建后调用。
 * 设置摄像机、清除色、场景根节点、轨迹球操作器、
 * 正交投影以及球体拾取事件处理器。
 */
void ViewportWidget::initOsg() {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;

    // 启用模型视图和投影统一变量（用于着色器）
    osg::State* state = viewer->getCamera()->getGraphicsContext()->getState();
    if (state) {
        state->setUseModelViewAndProjectionUniforms(true);
        state->setUseVertexAttributeAliasing(true);
    }

    // 深色背景（与 interactive_slam 保持一致）
    viewer->getCamera()->setClearColor(osg::Vec4(0.0706f, 0.0706f, 0.0863f, 1.0f));

    // 设置场景根节点
    viewer->setSceneData(m_sceneViz->getRootNode());

    // 轨迹球摄像机操作器
    // 关闭松手后的惯性甩动：OSG 的 StandardManipulator 默认 allowThrow=true，
    // 拖拽旋转后松手会继续旋转（表现为"甩动"），此处显式关闭
    auto* manipulator = new osgGA::TrackballManipulator;
    manipulator->setAllowThrow(false);
    viewer->setCameraManipulator(manipulator);

    // 正交投影（替换 OSG 默认的透视投影）
    applyProjection();

    // 注册拾取处理器（使用惰性数据提供器）
    // 球心坐标和边段数据在每次事件触发时实时查询，确保图谱加载/位姿更新后自动反映
    m_pickingHandler = new SpherePickingHandler(
        // 球心坐标提供器
        [this]() -> const std::vector<PickableCenter>* {
            return &m_sceneViz->sphereCenters();
        },
        // 边段提供器
        [this]() -> const std::vector<EdgeSegment>* {
            return &m_sceneViz->edgeSegments();
        },
        m_sceneViz->sphereRadius(),
        // --- 选择回调（Ctrl+Click） ---
        [this](long vertexId) {
            QMetaObject::invokeMethod(this, [this, vertexId]() {
                onVertexPicked(vertexId);
            }, Qt::QueuedConnection);
        },
        // --- 上下文菜单回调（右键） ---
        [this](const PickingHit& hit) {
            QMetaObject::invokeMethod(this, [this, hit]() {
                // 从图谱数据中丰富顶点信息字段
                PickingHit enriched = hit;
                if (hit.vertexId >= 0 && m_graph) {
                    auto it = m_graph->keyframes.find(hit.vertexId);
                    if (it != m_graph->keyframes.end()) {
                        auto& kf = it->second;
                        auto pos = kf->estimate().translation();
                        enriched.vtxPosX = pos.x();
                        enriched.vtxPosY = pos.y();
                        enriched.vtxPosZ = pos.z();
                        enriched.vtxCloudSize = kf->cloud ? static_cast<long>(kf->cloud->size()) : 0;
                        enriched.vtxAccumDist = kf->accum_distance;
                        enriched.vtxDegree = static_cast<int>(kf->node->edges().size());
                    }
                }
                // hit.screenX/Y 是 OSG 视口的设备像素坐标（原点左下、
                // Y 向上）。换算成 Qt 逻辑坐标（原点左上、Y 向下）后
                // 才能映射为全局坐标：先除 DPR 回逻辑像素，再翻 Y
                // （OSG Y 最大值是 traits 高度 - 1，与 Qt 高度差 1 像素，
                // 取整误差可忽略）。
                const double dpr = m_osgWidget->devicePixelRatioF();
                const int qtX = (dpr > 0.0)
                    ? static_cast<int>(hit.screenX / dpr) : static_cast<int>(hit.screenX);
                const int qtY = (dpr > 0.0)
                    ? static_cast<int>(m_osgWidget->height() - hit.screenY / dpr)
                    : static_cast<int>(m_osgWidget->height() - hit.screenY);
                QPoint globalPos = m_osgWidget->mapToGlobal(QPoint(qtX, qtY));
                emit contextMenuRequested(
                    enriched.vertexId, enriched.edgeId,
                    enriched.edgeV1, enriched.edgeV2,
                    enriched.edgeDist,
                    QString::fromStdString(enriched.edgeKernel),
                    globalPos,
                    enriched.vtxCloudSize,
                    enriched.vtxPosX, enriched.vtxPosY, enriched.vtxPosZ,
                    enriched.vtxAccumDist, enriched.vtxDegree);
            }, Qt::QueuedConnection);
        },
        // --- 双击兜底回调（未命中点云：球体聚焦 / 空白取消聚焦淡化） ---
        [this](long vertexId) {
            QMetaObject::invokeMethod(this, [this, vertexId]() {
                if (vertexId < 0) {
                    // 双击空白处：取消聚焦淡化，恢复整体不透明度与滚轮缩放系数
                    m_sceneViz->setFocusedVertex(-1);
                    restoreWheelZoomFactor();
                    m_osgWidget->update();
                    return;
                }
                focusOnVertex(vertexId);
            }, Qt::QueuedConnection);
        },
        // --- 双击点云命中回调（命中点设为轨迹球旋转中心并居中） ---
        [this](const osg::Vec3d& point) {
            QMetaObject::invokeMethod(this, [this, point]() {
                onFocusPoint(point);
            }, Qt::QueuedConnection);
        },
        // --- Ctrl+双击回调（切换到该帧位姿视角） ---
        [this](long vertexId) {
            QMetaObject::invokeMethod(this, [this, vertexId]() {
                onFrameView(vertexId);
            }, Qt::QueuedConnection);
        },
        // --- 实时球体半径提供器（右键边命中阈值随渲染面板尺寸调整保持正确） ---
        [this]() -> float {
            return m_sceneViz->sphereRadius();
        });
    viewer->addEventHandler(m_pickingHandler);

    emit initialized();
}

// ---------------------------------------------------------------------------
// 图谱加载
// ---------------------------------------------------------------------------

/**
 * @brief 图谱加载完成
 *
 * 将图谱数据传递给场景可视化器构建场景（球体/边线同步，
 * 点云后台异步构建），应用当前设置，更新投影并重置摄像机。
 *
 * 点云构建完成后（onCloudBuildFinished）发射 cloudDataReady 供 UI
 * 面板初始化，并刷新渲染统计。
 */
void ViewportWidget::onGraphLoaded(std::shared_ptr<hdl_graph_slam::InteractiveGraph> graph) {
    m_graph = graph;
    ++m_cloudBuildSeq;  // 使任何在途构建结果作废
    m_fpSavedSpeed = -1.0;  // 换图后速度按新场景尺度重新推导（旧速度可能不适用）
    m_sceneViz->buildFromGraph(graph, m_flags);

    // 应用当前设置
    m_sceneViz->setPointOpacity(m_flags.draw_keyframe_vertices ? 1.0f : 0.0f);
    m_osgWidget->update();

    // 后台异步构建点云（完成后发射 cloudDataReady）
    rebuildPointClouds();

    // 为新加载的场景更新正交投影，然后让摄像机定格到整个场景
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (viewer) {
        applyProjection();
        viewer->home();
    }
}

/**
 * @brief 图谱关闭
 *
 * 重置共享指针，清空场景可视化器，并使在途点云构建结果作废。
 * 场景清空时采样步长/隐藏边集合随之重置（新图边 ID 从 0 重编号，
 * 旧快照会误隐藏同号新边），这里把重置以 stride=1 广播给播放轴、
 * 自动回环与渲染面板，保持 UI 与场景一致。
 */
void ViewportWidget::onGraphClosed() {
    m_graph.reset();
    ++m_cloudBuildSeq;  // 使在途构建结果作废
    m_fpSavedSpeed = -1.0;  // 图已关闭，下次加载按新场景尺度重新推导速度
    const int prevStride = m_sceneViz->sampleStride();
    m_sceneViz->clear();
    if (prevStride != 1) {
        m_flags.sample_stride = 1;
        emit sampleStrideChanged(1);
    }
    m_osgWidget->update();
}

// ---------------------------------------------------------------------------
// 渲染控制
// ---------------------------------------------------------------------------

void ViewportWidget::setDrawVertices(bool v) {
    m_flags.draw_verticies = v;
    m_flags.draw_keyframe_vertices = v;
    m_sceneViz->setDrawVertices(v);
    m_osgWidget->update();
}

void ViewportWidget::setDrawEdges(bool v) {
    m_flags.draw_edges = v;
    m_sceneViz->setDrawEdges(v);
    m_osgWidget->update();
}

void ViewportWidget::setDrawKeyframeClouds(bool v) {
    m_flags.draw_keyframe_vertices = v;
    m_sceneViz->setDrawKeyframeClouds(v);
    m_osgWidget->update();
}

// ---------------------------------------------------------------------------
// 原始层内容签名
// ---------------------------------------------------------------------------

namespace {

/**
 * @brief 计算原始层的内容签名（0 保留为"尚未构建"哨兵，正常情况不返回 0）
 *
 * 原始层的内容只由"有哪些帧、每帧多少点"决定——里程计位姿永不变化，
 * 所以这个签名足以判定已换入的原始层能否继续复用。
 *
 * 用累加（交换律）而不是链式哈希：graph->keyframes 是 std::unordered_map，
 * 迭代顺序不保证跨增删稳定，链式哈希会因为顺序抖动产生"假变化"，把冻结
 * 复用退化成每轮都重建。
 *
 * 关键帧增删、单帧点数变化、换图都会改变签名；"删一帧又加一帧、帧数不变"
 * 也会变（帧 ID 与点数都参与混合）。
 */
uint64_t originalLayerSignature(
    const std::shared_ptr<hdl_graph_slam::InteractiveGraph>& graph) {
    if (!graph) return 0;
    uint64_t sum = 0;
    for (const auto& kv : graph->keyframes) {
        const uint64_t n = (kv.second && kv.second->cloud)
                               ? static_cast<uint64_t>(kv.second->cloud->size())
                               : 0ull;
        uint64_t e = static_cast<uint64_t>(kv.first) * 14695981039346656037ull;
        e = (e ^ n) * 1099511628211ull;
        sum += e;
    }
    sum ^= static_cast<uint64_t>(graph->keyframes.size()) * 1099511628211ull;
    return sum ? sum : 1ull;
}

}  // namespace

void ViewportWidget::setOdomLayerEnabled(bool enabled) {
    m_flags.odom_layer_enabled = enabled;
    bool wasEnabled = m_sceneViz->odomLayerEnabled();
    m_sceneViz->setOdomLayerEnabled(enabled);
    // 原始层数据按需生成：只有"需要（重新）构建"时才触发后台重建——尚未构建过，
    // 或者图内容已经变了（签名不符，例如原始层关着的时候增删了关键帧）。
    // 已经与当前图内容一致的冻结层只切换可见性，不再白付一次全量重建。
    if (enabled && !wasEnabled && m_graph && m_sceneViz->hasPointCloud()) {
        const bool reusable =
            m_sceneViz->odomLayerReady() &&
            originalLayerSignature(m_graph) == m_sceneViz->committedOdomSignature();
        if (!reusable) requestCloudBuild();
    }
    m_osgWidget->update();
}

void ViewportWidget::setDrawSE3Edges(bool v) {
    m_flags.draw_se3_edges = v;
    // SE3 边是常规边集合的一部分
    m_sceneViz->setDrawEdges(v);
    m_osgWidget->update();
}

void ViewportWidget::setEdgeWidth(int width) {
    m_sceneViz->setEdgeWidth(static_cast<float>(width));
    m_osgWidget->update();
}

void ViewportWidget::setSphereRadius(float radius) {
    m_sceneViz->setSphereRadius(radius);
    m_osgWidget->update();
}

void ViewportWidget::setVertexOpacity(int opacity) {
    m_sceneViz->setVertexOpacity(opacity / 100.0f);
    m_osgWidget->update();
}

void ViewportWidget::setSampleStride(int stride) {
    m_flags.sample_stride = stride;
    m_sceneViz->setSampleStride(stride);
    m_osgWidget->update();
    emit sampleStrideChanged(stride);
}

void ViewportWidget::setPointSize(int size) {
    m_sceneViz->setPointSize(static_cast<float>(size));
    m_osgWidget->update();
}

void ViewportWidget::setPointOpacity(int opacity) {
    m_sceneViz->setPointOpacity(opacity / 100.0f);
    m_osgWidget->update();
}

void ViewportWidget::setLodEnabled(bool enabled) {
    m_flags.lod_enabled = enabled;
    m_sceneViz->setLodEnabled(enabled);
    // 已有点云数据时触发后台异步重建（生成/移除第一层 LOD）
    if (m_graph && m_sceneViz->hasPointCloud()) {
        requestCloudBuild();
    }
}

void ViewportWidget::setLodMode(bool manual) {
    m_lodManualMode = manual;
    if (manual) {
        // 立即应用当前手动层级（应用时 clamp 到有效范围，不覆盖用户设定值，
        // 保证数据重建后能恢复到用户选择的层级）
        int level = qBound(0, m_lodManualLevel, m_sceneViz->lodLevelCount() - 1);
        m_sceneViz->setLodLevel(level);
        emit lodLevelChanged(level, m_sceneViz->lodLevelCount());
    } else {
        // 恢复自动距离切换：立即按当前距离重新评估
        emit lodLevelChanged(m_sceneViz->currentLodLevel(),
                             m_sceneViz->lodLevelCount());
    }
    m_osgWidget->update();
}

void ViewportWidget::setLodManualLevel(int level) {
    m_lodManualLevel = level;
    if (m_lodManualMode && m_sceneViz->hasPointCloud()) {
        int maxLevel = m_sceneViz->lodLevelCount() - 1;
        int clamped = qBound(0, level, maxLevel);
        m_sceneViz->setLodLevel(clamped);
        emit lodLevelChanged(clamped, m_sceneViz->lodLevelCount());
        m_osgWidget->update();
    }
}

void ViewportWidget::setZClipping(bool enabled) {
    m_sceneViz->setZClipping(enabled);
    m_osgWidget->update();
}

void ViewportWidget::setZClipMin(double minZ) {
    m_sceneViz->setZClipRange(static_cast<float>(minZ), m_sceneViz->getZClipMax());
    m_osgWidget->update();
}

void ViewportWidget::setZClipMax(double maxZ) {
    m_sceneViz->setZClipRange(m_sceneViz->getZClipMin(), static_cast<float>(maxZ));
    m_osgWidget->update();
}

void ViewportWidget::setColorZMin(double minZ) {
    m_sceneViz->setColorZRange(static_cast<float>(minZ), m_sceneViz->getColorZMax());
    m_osgWidget->update();
}

void ViewportWidget::setColorZMax(double maxZ) {
    m_sceneViz->setColorZRange(m_sceneViz->getColorZMin(), static_cast<float>(maxZ));
    m_osgWidget->update();
}

void ViewportWidget::setAutoColorRange(bool autoRange) {
    m_sceneViz->setAutoColorRange(autoRange);
    m_osgWidget->update();
}

void ViewportWidget::setBackgroundColor(const QColor& color) {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (viewer) {
        viewer->getCamera()->setClearColor(
            osg::Vec4(color.redF(), color.greenF(), color.blueF(), 1.0f));
    }
    m_osgWidget->update();
}

void ViewportWidget::setHiddenEdges(const std::set<long>& ids) {
    m_sceneViz->setHiddenEdges(ids);
}

void ViewportWidget::setLoopHighlight(long sourceId, const std::vector<long>& candidateIds) {
    m_sceneViz->setLoopHighlight(sourceId, candidateIds);
}

void ViewportWidget::resetCamera() {
    exitFirstPersonMode();
    restoreWheelZoomFactor();
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (viewer) {
        viewer->home();
    }
    m_osgWidget->update();
}

/**
 * @brief 恢复聚焦时提高的滚轮缩放系数
 *
 * 由 resetCamera 与"双击空白处取消聚焦"调用。
 */
void ViewportWidget::restoreWheelZoomFactor() {
    if (m_savedWheelZoomFactor < 0.0) return;  // 未处于聚焦加速状态
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (viewer) {
        auto* manip = dynamic_cast<osgGA::TrackballManipulator*>(
            viewer->getCameraManipulator());
        if (manip) {
            manip->setWheelZoomFactor(m_savedWheelZoomFactor);
        }
    }
    m_savedWheelZoomFactor = -1.0;
}

// ---------------------------------------------------------------------------
// 第一人称模式（Shift 切换）
// ---------------------------------------------------------------------------

/**
 * @brief 进入第一人称模式
 *
 * 从当前相机位姿取视线方向初始化 yaw/pitch，行走速度按场景包围球
 * 半径设定；切换操作器到 FirstPersonManipulator（保留轨迹球实例，
 * 退出时恢复并以第一人称位姿无缝衔接）。
 *
 * 滚轮调速后经操作器回调回传当前速度，在状态栏回显；同一张图内
 * Shift 反复切换时沿用用户上次调好的速度（m_fpSavedSpeed）。
 */
void ViewportWidget::enterFirstPersonMode() {
    if (m_fpActive) return;
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer || !viewer->getSceneData()) return;
    auto* trackball = dynamic_cast<osgGA::TrackballManipulator*>(
        viewer->getCameraManipulator());
    if (!trackball) return;

    osg::Vec3d eye, center, up;
    viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
    osg::Vec3d dir = center - eye;
    if (dir.length2() < 1e-12) dir.set(0.0, 1.0, 0.0);

    if (!m_fpManip) m_fpManip = new FirstPersonManipulator;
    // 速度回调在 OSG 渲染线程被调用，不能直接碰 Qt UI：
    // 用 QueuedConnection 投递回本对象所在（Qt 主）线程后再发信号
    m_fpManip->setSpeedCallback([this](double speed) {
        QMetaObject::invokeMethod(this, [this, speed]() {
            emit firstPersonSpeedChanged(speed);
        }, Qt::QueuedConnection);
    });
    const double sceneR = viewer->getSceneData()->getBound().radius();
    // 同一张图内保留用户上次调好的速度；否则按场景尺度推导（米/秒）
    const double speed = (m_fpSavedSpeed > 0.0)
        ? m_fpSavedSpeed
        : std::max(sceneR * 0.15, 1.5);
    m_fpManip->startFrom(eye, dir, speed);

    m_savedManip = viewer->getCameraManipulator();
    viewer->setCameraManipulator(m_fpManip.get(), false);  // false = 不重置 home
    m_fpActive = true;
    // 先回显初始速度，再发模式提示：状态栏最终留下的是操作帮助文案
    emit firstPersonSpeedChanged(m_fpManip->walkSpeed());
    emit firstPersonModeChanged(true);
}

/**
 * @brief 退出第一人称模式
 *
 * 取当前第一人称位姿，恢复轨迹球操作器并 setTransformation 到同一
 * 位姿，视角无跳变衔接。
 */
void ViewportWidget::exitFirstPersonMode() {
    if (!m_fpActive) return;
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) { m_fpActive = false; return; }

    osg::Vec3d eye, dir;
    m_fpManip->getPose(eye, dir);
    m_fpSavedSpeed = m_fpManip->walkSpeed();  // 保留本次调好的速度，同图内下次进入沿用

    viewer->setCameraManipulator(m_savedManip.get(), false);
    auto* trackball = dynamic_cast<osgGA::TrackballManipulator*>(
        viewer->getCameraManipulator());
    if (trackball) {
        trackball->setTransformation(eye, eye + dir, osg::Vec3d(0.0, 0.0, 1.0));
    }
    m_savedManip = nullptr;
    m_fpActive = false;
    emit firstPersonModeChanged(false);
}

/**
 * @brief 事件过滤器：拦截 osgQOpenGLWidget 的键盘事件驱动第一人称模式
 *
 * - Shift 按下 → 在任意状态下切换第一人称模式（进入/退出）；
 * - 第一人称模式下 W/A/S/D 按下/释放 → 写入行走键状态（不转发给 OSG）。
 * 其余事件一律放行。osgQOpenGLWidget 默认无键盘焦点策略，
 * 已在构造时设为 StrongFocus（点击视口一次即获得焦点）。
 */
bool ViewportWidget::eventFilter(QObject* watched, QEvent* event) {
    if (watched == m_osgWidget) {
        switch (event->type()) {
        case QEvent::KeyPress: {
            auto* ke = static_cast<QKeyEvent*>(event);
            if (ke->isAutoRepeat()) break;
            // Shift 在两种状态下都可切换（进入/退出第一人称）
            if (ke->key() == Qt::Key_Shift) {
                if (m_fpActive) exitFirstPersonMode();
                else enterFirstPersonMode();
                return true;
            }
            // WASD 仅在第一人称模式下作为行走键拦截
            if (m_fpActive) {
                switch (ke->key()) {
                case Qt::Key_W: m_fpManip->setKey('W', true); return true;
                case Qt::Key_A: m_fpManip->setKey('A', true); return true;
                case Qt::Key_S: m_fpManip->setKey('S', true); return true;
                case Qt::Key_D: m_fpManip->setKey('D', true); return true;
                case Qt::Key_Q: m_fpManip->setKey('Q', true); return true;
                case Qt::Key_E: m_fpManip->setKey('E', true); return true;
                default: break;
                }
            }
            break;
        }
        case QEvent::KeyRelease: {
            auto* ke = static_cast<QKeyEvent*>(event);
            if (ke->isAutoRepeat()) break;
            if (m_fpActive) {
                switch (ke->key()) {
                case Qt::Key_W: m_fpManip->setKey('W', false); return true;
                case Qt::Key_A: m_fpManip->setKey('A', false); return true;
                case Qt::Key_S: m_fpManip->setKey('S', false); return true;
                case Qt::Key_D: m_fpManip->setKey('D', false); return true;
                case Qt::Key_Q: m_fpManip->setKey('Q', false); return true;
                case Qt::Key_E: m_fpManip->setKey('E', false); return true;
                default: break;
                }
            }
            break;
        }
        default:
            break;
        }
    }
    return QWidget::eventFilter(watched, event);
}

/**
 * @brief 双击聚焦：相机飞到指定位姿球体正后上方，沿扫描方向看向球心
 *
 * 位姿轴约定（由调试轴验证）：红X=右、绿Y=下、蓝Z=前方扫描方向。
 * 相机坐标系对齐：
 *   - 相机 right      = 位姿局部 +X（红，右）
 *   - 相机视线（前向）= 位姿局部 +Z（蓝，前方扫描方向）
 *   - 相机 up         = 位姿局部 −Y（绿轴向下，取反为实际上方）
 * 相机位置 = 球心正后方（-Z）再沿 up 抬升 lift（正后上方），
 * 距离 dist 稍远，兼顾观察球体与前方场景。
 * 数学：f=Z, up=−Y → s=Z^(−Y)=X（right）、u=X^Z=−Y（up）。
 */
void ViewportWidget::focusOnVertex(long vertexId) {
    if (!m_graph) return;
    if (m_fpActive) exitFirstPersonMode();  // 聚焦使用轨迹球，先退出第一人称
    auto it = m_graph->keyframes.find(vertexId);
    if (it == m_graph->keyframes.end()) return;
    const auto& pose = it->second->estimate();   // Eigen::Isometry3d

    // 球心（位姿平移）、前向（局部 z 轴=扫描方向）、up（局部 y 轴取反）
    osg::Vec3d center(pose.translation().x(),
                      pose.translation().y(),
                      pose.translation().z());
    Eigen::Vector3d dirZ = pose.rotation() * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d dirY = pose.rotation() * Eigen::Vector3d::UnitY();
    osg::Vec3d forward(dirZ.x(), dirZ.y(), dirZ.z());
    osg::Vec3d up(-dirY.x(), -dirY.y(), -dirY.z());
    forward.normalize();
    up.normalize();

    // 相机距离：以位姿球体半径的比例（聚焦到单个位姿，近距离观察球体）
    double radius = m_sceneViz->sphereRadius();
    double dist = (radius > 1e-6) ? radius * 12.0 : 6.0;   // 正后方距离（稍远）
    double lift = (radius > 1e-6) ? radius * 4.0 : 2.0;    // 沿 up（实际上方）的抬升量
    // 相机位于球心正后上方：正后方（-forward）再沿 up 抬升 lift
    osg::Vec3d eye = center - forward * dist + up * lift;

    // 设置轨迹球相机：eye / center / up，球心居中、朝向球心。
    // 只需 setTransformation：TrackballManipulator 会据此更新内部状态，
    // 每帧由 getInverseMatrix() 自动生成视图矩阵（手动 setViewMatrix
    // 会与操作器每帧的矩阵计算冲突）
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;
    auto* manip = dynamic_cast<osgGA::TrackballManipulator*>(
        viewer->getCameraManipulator());
    if (manip) {
        manip->setTransformation(eye, center, up);
        // 聚焦后相机距离骤减（约 radius×12，可能仅数米），而 Trackball 的
        // 滚轮/平移步长都与当前距离成正比，退回工作视距会变得极慢。
        // 提高滚轮缩放系数（OSG 默认 0.1），保存原值供 resetCamera /
        // 双击空白处取消聚焦时恢复
        if (m_savedWheelZoomFactor < 0.0) {
            m_savedWheelZoomFactor = manip->getWheelZoomFactor();
        }
        manip->setWheelZoomFactor(0.5);
    }
    // 聚焦淡化：全体标记透明度压到最低，目标锥体保持稍高不透明度
    m_sceneViz->setFocusedVertex(vertexId);
    m_osgWidget->update();
}

/**
 * @brief 双击点云居中：轨迹球旋转中心移到命中的点云点
 *
 * 保持相机眼点与向上方向不变，仅把视线中心（= 轨迹球旋转中心）
 * 移到命中点：该点随即位于屏幕中央，后续拖拽即绕该点公转，
 * 旋转过程中该点恒居屏幕中心。同时恢复滚轮缩放系数
 * （若此前处于聚焦缩小状态，避免缩放步长过细）。
 */
void ViewportWidget::onFocusPoint(const osg::Vec3d& point) {
    if (m_fpActive) exitFirstPersonMode();  // 轨迹球操作，先退出第一人称
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;
    auto* manip = dynamic_cast<osgGA::TrackballManipulator*>(
        viewer->getCameraManipulator());
    if (!manip) return;

    osg::Vec3d eye, center, up;
    manip->getTransformation(eye, center, up);
    manip->setTransformation(eye, point, up);
    restoreWheelZoomFactor();
    m_osgWidget->update();
}

/**
 * @brief Ctrl+双击：切换到指定关键帧的位姿视角
 *
 * 相机眼点 = 关键帧位姿平移（相机光心），视线沿位姿局部 +Z（扫描
 * 方向），up 取局部 −Y（Y 轴向下）。轨迹球中心放在前方 lookAhead
 * 处，进入后拖拽绕其旋转；聚焦淡化与滚轮缩放语义与旧双击聚焦一致。
 * 数学同 focusOnVertex：f=Z, up=−Y。
 */
void ViewportWidget::onFrameView(long vertexId) {
    if (vertexId < 0 || !m_graph) return;
    if (m_fpActive) exitFirstPersonMode();
    auto it = m_graph->keyframes.find(vertexId);
    if (it == m_graph->keyframes.end()) return;
    const auto& pose = it->second->estimate();

    osg::Vec3d eye(pose.translation().x(),
                   pose.translation().y(),
                   pose.translation().z());
    Eigen::Vector3d dirZ = pose.rotation() * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d dirY = pose.rotation() * Eigen::Vector3d::UnitY();
    osg::Vec3d forward(dirZ.x(), dirZ.y(), dirZ.z());
    osg::Vec3d up(-dirY.x(), -dirY.y(), -dirY.z());
    forward.normalize();
    up.normalize();

    // 轨迹球中心放在前方（帧视角下的注视点）
    double radius = m_sceneViz->sphereRadius();
    double lookAhead = (radius > 1e-6) ? radius * 10.0 : 5.0;
    osg::Vec3d center = eye + forward * lookAhead;

    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;
    auto* manip = dynamic_cast<osgGA::TrackballManipulator*>(
        viewer->getCameraManipulator());
    if (!manip) return;
    manip->setTransformation(eye, center, up);
    if (m_savedWheelZoomFactor < 0.0) {
        m_savedWheelZoomFactor = manip->getWheelZoomFactor();
    }
    manip->setWheelZoomFactor(0.5);
    m_sceneViz->setFocusedVertex(vertexId);
    m_osgWidget->update();
}

/**
 * @brief 播放跟随：切换到指定关键帧的位姿视角（仅相机，不改标记不透明度）
 *
 * 相机数学与 onFrameView 完全一致：eye = 位姿平移，forward = 局部 +Z，
 * up = 局部 −Y，轨迹球中心放在前方 lookAhead 处。
 *
 * 与 onFrameView 的两点差异（见头文件注释）：
 *   - 不调用 setFocusedVertex()，避免与播放独显的不透明度覆盖互相打架；
 *   - 不保存/改写滚轮缩放系数，逐帧调用不污染全局缩放手感。
 * 另加退化保护：逐帧调用必须安全，绝不把 inf/NaN 写进轨迹球。
 */
void ViewportWidget::followFrameView(long vertexId) {
    if (vertexId < 0 || !m_graph) return;
    if (m_fpActive) exitFirstPersonMode();  // 第一人称操作器不认轨迹球位姿
    auto it = m_graph->keyframes.find(vertexId);
    if (it == m_graph->keyframes.end()) return;
    const auto& pose = it->second->estimate();

    osg::Vec3d eye(pose.translation().x(),
                   pose.translation().y(),
                   pose.translation().z());
    Eigen::Vector3d dirZ = pose.rotation() * Eigen::Vector3d::UnitZ();
    Eigen::Vector3d dirY = pose.rotation() * Eigen::Vector3d::UnitY();
    osg::Vec3d forward(dirZ.x(), dirZ.y(), dirZ.z());
    osg::Vec3d up(-dirY.x(), -dirY.y(), -dirY.z());

    // 退化保护：旋转矩阵异常导致零向量时直接放弃，避免归一化产生 inf
    if (forward.length2() < 1e-18 || up.length2() < 1e-18) return;
    forward.normalize();
    up.normalize();

    // 轨迹球中心放在前方（帧视角下的注视点）
    double radius = m_sceneViz->sphereRadius();
    double lookAhead = (radius > 1e-6) ? radius * 10.0 : 5.0;
    osg::Vec3d center = eye + forward * lookAhead;

    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;
    auto* manip = dynamic_cast<osgGA::TrackballManipulator*>(
        viewer->getCameraManipulator());
    if (!manip) return;
    manip->setTransformation(eye, center, up);
    // 注意：此处刻意不改滚轮缩放系数、不做聚焦淡化（播放独显已接管标记不透明度）
    m_osgWidget->update();
}

// ---------------------------------------------------------------------------
// 手动场景刷新（边删除等数据变化后调用）
// ---------------------------------------------------------------------------

void ViewportWidget::refreshScene() {
    if (!m_graph) return;
    m_sceneViz->updatePoses(m_graph);
    m_osgWidget->update();
}

// ---------------------------------------------------------------------------
// 异步点云构建调度
// ---------------------------------------------------------------------------

/**
 * @brief 请求重建点云（异步，自动合并）
 *
 * 若已有构建任务在运行，仅标记 pending，当前任务完成后自动再启动一次，
 * 避免多次连续触发（自动回环插入边、图优化等）导致并行构建竞争。
 */
void ViewportWidget::armPerfCycle() {
    if (m_perfCycleActive) return;  // 幂等：一轮周期内只有首个入口生效
    m_perfCycleActive = true;
    m_perfRequestAt = std::chrono::steady_clock::now();
    const hdl_graph_slam::GpuMemory::Info vram = hdl_graph_slam::GpuMemory::query();
    m_perfVRAMBeforeMiB = vram.valid ? vram.usedMiB : 0;
}

void ViewportWidget::requestCloudBuild() {
    if (!m_graph) return;
    m_cloudBuildPending = true;

    // Phase 0：一轮构建周期从"用户/图变化发出的首个请求"开始计时。
    // 构建期间可能因优化完成/预算变化被合并出多次请求，只有首个请求
    // 刷新起点，才能反映用户感知的端到端延迟。
    armPerfCycle();

    if (m_cloudBuildRunning) return;  // 已有任务在跑，完成后再处理
    startCloudBuild();
}

/**
 * @brief 启动后台点云构建任务
 *
 * 计算密集部分（位姿快照、CPU 变换、体素降采样、顶点数组构建）
 * 在 QtConcurrent 工作线程执行，不阻塞 UI。完成后由
 * onCloudBuildFinished() 在主线程换入场景。
 */
void ViewportWidget::startCloudBuild() {
    m_cloudBuildRunning = true;
    m_cloudBuildPending = false;
    m_cloudBuildActiveSeq = m_cloudBuildSeq;

    // Phase 0：链式构建（上一轮结束后直接续跑，不经过 requestCloudBuild）
    // 也要保证计时已就绪；已武装时本调用无副作用。
    armPerfCycle();

    auto graph = m_graph;
    hdl_graph_slam::BuildOptions options;
    options.lodEnabled = m_sceneViz->lodEnabled();
    // 原始层开关：打开时显示里程计位姿参照底图（淡橙半透明）
    options.showOriginalLayer = m_sceneViz->odomLayerEnabled();
    // 原始层冻结复用：内容签名与已换入的那一层一致时**不再生成**——不重算
    // 8000 多万次 odom 变换、不新建 2.12 GiB 数组、不重传缓冲；已换入的几何体
    // 由 commitBuild 原样保留。实测开启原始层曾让端到端从 43.96 s 涨到 66.68 s
    // （后台 +9.2 s，主线程换入 +13.5 s），而其中绝大部分本就可省。
    const uint64_t odomSig = originalLayerSignature(graph);
    // "已经构建过"必须与签名一起判：签名在从未构建时也已有值（例如原始层一直
    // 关着、或刚打开过地图），只看签名会把"尚未构建"误判成"可复用"，底图就
    // 永远不会出现。
    options.buildOriginalLayer =
        options.showOriginalLayer &&
        (!m_sceneViz->odomLayerReady() ||
         odomSig != m_sceneViz->committedOdomSignature());
    options.odomSignature = odomSig;
    // 传入当前颜色参数：builder 在后台生成与当前设置一致的颜色，
    // 主线程 commit 时零遍历（避免全量重着色卡顿）
    options.useAutoColorRange = m_sceneViz->isAutoColorRange();
    options.colorZMin = m_sceneViz->getColorZMin();
    options.colorZMax = m_sceneViz->getColorZMax();
    options.opacity   = m_sceneViz->getPointOpacity();
    m_cloudBuildWatcher->setFuture(QtConcurrent::run([graph, options]() {
        return hdl_graph_slam::PointCloudBuilder::build(graph, options);
    }));
}

/**
 * @brief 后台点云构建完成（主线程回调）
 *
 * 若构建期间图已更换/关闭（版本号不匹配）则丢弃结果；
 * 否则换入场景、发射数据范围信号并刷新视图。
 */
void ViewportWidget::onCloudBuildFinished() {
    m_cloudBuildRunning = false;

    if (m_cloudBuildActiveSeq == m_cloudBuildSeq) {
        auto result = m_cloudBuildWatcher->result();
        // Phase 0：阶段统计随 result 一起被 move 进场景，先留一份给日志
        const hdl_graph_slam::BuildTimings buildTimings = result.timings;

        m_sceneViz->commitPointCloudBuild(std::move(result));
        m_perfCommitAt = std::chrono::steady_clock::now();

        logCloudBuildPerf(buildTimings);  // Phase 0 基线日志

        // 发射数据范围信号供 UI 面板初始化/更新
        emit cloudDataReady(m_sceneViz->getDataZMin(), m_sceneViz->getDataZMax());

        // LOD 状态提示：手动模式恢复用户选择的层级；自动模式从 level 0 起步
        if (m_sceneViz->lodEnabled()) {
            if (m_lodManualMode && m_sceneViz->lodLevelCount() > 1) {
                int level = qBound(0, m_lodManualLevel,
                                   m_sceneViz->lodLevelCount() - 1);
                m_sceneViz->setLodLevel(level);
                emit lodLevelChanged(level, m_sceneViz->lodLevelCount());
            } else {
                emit lodLevelChanged(0, m_sceneViz->lodLevelCount());
            }
        }
        m_osgWidget->update();

        // 点云渲染完成判定：分块渐进上传开始前标记"待检测"；
        // 若本次构建无分块（空点云）则立即通知完成，避免加载指示卡死
        m_chunkUploadWasPending = m_sceneViz->chunkUploadPending();
        if (!m_chunkUploadWasPending) {
            // Phase 0：本次构建无分块（空点云）→ 无渐进上传阶段，直接结束计时
            m_perfCycleActive = false;
            emit cloudRenderFinished(m_cloudBuildSeq);
        }
    } else {
        // 构建被丢弃（图已更换/关闭）：没有新点云要渲染，立即通知完成。
        // 注意携带被丢弃结果的代际（activeSeq，旧值）：消费方按信号自带
        // 代际过滤，若此刻新图已加载（seq 已递增），此过期信号不会误关
        // 新一轮加载的遮罩
        m_perfCycleActive = false;  // Phase 0：本轮无有效结果，结束计时
        emit cloudRenderFinished(m_cloudBuildActiveSeq);
    }

    // 构建期间有新请求（预算变化/优化完成等）→ 用最新状态再构建一次
    if (m_cloudBuildPending) {
        startCloudBuild();
    }
}

/**
 * @brief 输出一次点云构建周期的基线日志（Phase 0 测量）
 *
 * 说明：本项目是 WIN32 GUI 程序，没有文件日志也没有 message handler，
 * qInfo 在无调试器时不可见，因此这里统一写入 CloudPerfLog
 * （<exe目录>/cloud_perf.log），同时转发 qInfo。
 *
 * 关注三个数：
 *   1. 后台构建阶段耗时与合计——优化后"重建"的 CPU 成本；
 *   2. 请求→换入延迟——用户感知的卡顿长度（Phase 2 要把它压到 ms 级）；
 *   3. VBO 数组字节与显存占用——双份点云的实际显存代价。
 */
void ViewportWidget::logCloudBuildPerf(const hdl_graph_slam::BuildTimings& t) {
    // 注意：KeyframePointCloudVisualizer 声明在**全局命名空间**（不在
    // hdl_graph_slam 里），所以此处不能加 hdl_graph_slam:: 前缀
    const KeyframePointCloudVisualizer::CommitStats c =
        m_sceneViz->lastCloudCommitStats();

    const double endToEndMs = std::chrono::duration<double, std::milli>(
                                  m_perfCommitAt - m_perfRequestAt)
                                  .count();

    auto mib = [](size_t bytes) {
        return QString::number(static_cast<double>(bytes) / (1024.0 * 1024.0), 'f', 1);
    };

    hdl_graph_slam::CloudPerfLog& log = hdl_graph_slam::CloudPerfLog::instance();
    log.write(QStringLiteral("=== 点云构建周期（后台构建 + 主线程换入） ==="));
    // 规模口径：t.mainRenderPoints/t.mainVboBytes 只含**主级别**，t.lodPoints/
    // t.lodVboBytes 只含 LOD 级别，而 t.odomPoints/t.odomVboBytes 是原始层的
    // **整层**合计。直接并排会造出"原始层比主层点数还多"的假象——实测原始层比
    // 主级别多的 75,141 恰好等于第一层 LOD 的点数，一度看起来像统计串了。
    // 这里显式打印两层的层合计，并把原始层是否为本轮重建讲清楚。
    log.write(QStringLiteral("  规模: 关键帧=%1  全量点=%2")
                  .arg(t.frameCount)
                  .arg(static_cast<qulonglong>(t.totalPoints)));
    log.write(QStringLiteral("    优化层: 主级别=%1 + LOD=%2 = %3 点")
                  .arg(static_cast<qulonglong>(t.mainRenderPoints))
                  .arg(static_cast<qulonglong>(t.lodPoints))
                  .arg(static_cast<qulonglong>(t.mainRenderPoints + t.lodPoints)));
    if (c.odomRebuilt) {
        log.write(QStringLiteral("    原始层: %1 点（全分辨率单级，本轮重建）")
                      .arg(static_cast<qulonglong>(t.odomPoints)));
    } else if (c.odomResidentBytes > 0) {
        log.write(QStringLiteral("    原始层: 冻结复用（沿用已换入几何体，未重建、未重传，"
                                 "仍驻留 %1 MiB）")
                      .arg(mib(c.odomResidentBytes)));
    } else {
        log.write(QStringLiteral("    原始层: 未启用"));
    }
    log.write(QStringLiteral("  后台阶段(ms): 快照=%1 变换=%2 滤波=%3 主数组=%4 "
                             "LOD=%5 原始层=%6 | 合计=%7")
                  .arg(t.snapshotMs, 0, 'f', 1)
                  .arg(t.transformMs, 0, 'f', 1)
                  .arg(t.outlierMs, 0, 'f', 1)
                  .arg(t.mainArraysMs, 0, 'f', 1)
                  .arg(t.lodMs, 0, 'f', 1)
                  .arg(t.odomMs, 0, 'f', 1)
                  .arg(t.totalMs, 0, 'f', 1));
    log.write(QStringLiteral("  端到端: 请求→换入=%1 ms").arg(endToEndMs, 0, 'f', 1));
    log.write(QStringLiteral("  本轮落地: 级别=%1  主层块=%2  原始层新建块=%3  "
                             "数组池化复用=%4  新建=%5")
                  .arg(static_cast<qulonglong>(c.levels))
                  .arg(static_cast<qulonglong>(c.chunks))
                  .arg(static_cast<qulonglong>(c.odomChunks))
                  .arg(static_cast<qulonglong>(c.pooledReuse))
                  .arg(static_cast<qulonglong>(c.freshArrays)));
    log.write(QStringLiteral("  数组字节: 本轮生成 优化层(主=%1 + LOD=%2) = %3 MiB "
                             "| 原始层=%4 MiB | 合计=%5 MiB")
                  .arg(mib(t.mainVboBytes), mib(t.lodVboBytes),
                       mib(t.mainVboBytes + t.lodVboBytes), mib(t.odomVboBytes),
                       mib(t.totalVboBytes())));
    log.write(QStringLiteral("            已换入 优化层=%1 MiB | 原始层=%2 MiB | 合计=%3 MiB")
                  .arg(mib(c.arrayBytes), mib(c.odomResidentBytes),
                       mib(c.arrayBytes + c.odomResidentBytes)));

    const hdl_graph_slam::GpuMemory::Info vram = hdl_graph_slam::GpuMemory::query();
    if (vram.valid) {
        // 注意采样时机：这里是"换入完成、但渐进上传还没跑"的时刻，VBO 尚未分配，
        // 所以 used 只反映 CPU 侧数组。真正的驻留量由 updateScene() 在渐进上传
        // 结束时补打（那里才是判定"到底装没装进显存"的唯一时机）。
        log.write(QStringLiteral("  显存(换入后/上传前): %1  used=%2 MiB / total=%3 MiB  "
                                 "(构建请求前 used=%4 MiB)")
                      .arg(vram.deviceName.isEmpty() ? QStringLiteral("(unknown)")
                                                     : vram.deviceName)
                      .arg(vram.usedMiB)
                      .arg(vram.totalMiB)
                      .arg(m_perfVRAMBeforeMiB));
    } else {
        log.write(QStringLiteral("  显存: 查询不可用（%1）")
                      .arg(hdl_graph_slam::GpuMemory::summaryLine()));
    }
}

/**
 * @brief 重建点云（异步入口，保持历史调用点兼容）
 */
void ViewportWidget::rebuildPointClouds() {
    requestCloudBuild();
}

// ---------------------------------------------------------------------------
// 帧更新（定时器驱动）
// ---------------------------------------------------------------------------

/**
 * @brief 定时场景更新（~60Hz）
 *
 * 在正交投影模式下，动态跟踪摄像机距离，使旋转/缩放操作手感自然，
 * 与透视投影下的体验一致。
 *
 * 注意：几何体在加载或显式刷新后保持不变。对于大型图谱，
 * 每 16ms 重建 20000+ 个球体和边的操作会严重降低帧率。
 * OSGRenderer 有自己的 10ms 定时器驱动渲染循环，
 * 因此这里只跟踪 FPS，不执行逐帧几何体重建。
 */
void ViewportWidget::updateScene() {
    // 分块点云渐进上传推进（每帧一块，摊平大 VBO 上传的 GPU 卡顿）
    m_sceneViz->advanceChunkUpload();

    // 点云渲染完成检测：渐进上传从"进行中"变为"完成"时通知 UI
    // （MainWindow 据此停止加载动画，保证 spinner 持续到点云全部渲染出来）
    {
        const bool pending = m_sceneViz->chunkUploadPending();
        if (m_chunkUploadWasPending && !pending) {
            // Phase 0：分块渐进上传全部落地，这才是"点云真的画完了"的时刻
            if (m_perfCycleActive) {
                const double totalMs = std::chrono::duration<double, std::milli>(
                                           std::chrono::steady_clock::now() - m_perfRequestAt)
                                           .count();
                hdl_graph_slam::CloudPerfLog::instance().write(
                    QStringLiteral("  分块渐进上传完成: 请求→全部可见=%1 ms")
                        .arg(totalMs, 0, 'f', 1));
                // 显存必须**在这里**采样。上面"数组字节"那几行是在换入后、渐进上传
                // 之前打的，那时 VBO 还没分配，used 只反映 CPU 侧数组（实测两次构建
                // 分别只涨了 0 和 55 MiB，而数组有 2124/4248 MiB），据此根本判断不了
                // 数组是否真的驻留显存——这是唯一能给出答案的时刻。
                const hdl_graph_slam::GpuMemory::Info vramAfter =
                    hdl_graph_slam::GpuMemory::query();
                if (vramAfter.valid) {
                    const qlonglong delta =
                        static_cast<qlonglong>(vramAfter.usedMiB) -
                        static_cast<qlonglong>(m_perfVRAMBeforeMiB);
                    hdl_graph_slam::CloudPerfLog::instance().write(
                        QStringLiteral("  显存(上传后): used=%1 MiB / total=%2 MiB  "
                                       "(构建请求前 used=%3 MiB，净增=%4 MiB)")
                            .arg(vramAfter.usedMiB)
                            .arg(vramAfter.totalMiB)
                            .arg(m_perfVRAMBeforeMiB)
                            .arg(delta));
                } else {
                    hdl_graph_slam::CloudPerfLog::instance().write(
                        QStringLiteral("  显存(上传后): 查询不可用（%1）")
                            .arg(hdl_graph_slam::GpuMemory::summaryLine()));
                }
                m_perfCycleActive = false;
            }
            // 渐进上传完成：携带当前代际（正常完成路径）
            emit cloudRenderFinished(m_cloudBuildSeq);
        }
        m_chunkUploadWasPending = pending;
    }

    // 正交投影模式下，动态跟踪摄像机距离
    if (m_useOrthographic) {
        osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
        if (viewer) {
            auto* manip = dynamic_cast<osgGA::TrackballManipulator*>(
                viewer->getCameraManipulator());
            if (manip) {
                double dist = manip->getDistance();
                osg::Camera* camera = viewer->getCamera();
                int vpW = m_osgWidget->width();
                int vpH = m_osgWidget->height();
                if (vpW > 0 && vpH > 0) {
                    double aspect = static_cast<double>(vpW) / static_cast<double>(vpH);
                    // 匹配透视图 30° FOV，使切换时感觉无缝
                    double halfHeight = dist * std::tan(osg::DegreesToRadians(30.0) * 0.5);
                    double halfWidth = halfHeight * aspect;
                    double farDist = halfHeight * 40.0;
                    camera->setProjectionMatrixAsOrtho(
                        -halfWidth, halfWidth,
                        -halfHeight, halfHeight,
                        0.1, farDist);
                }
            }
        }
    }

    if (!m_graph) return;

    // LOD 级别切换：手动模式固定层级，否则按相机到点云包围球的距离
    // 自动切换（带迟滞防抖）。级别 0 = 全量（近处），级别 1 = 第一层
    // 降采样（远处）。升级阈值 dist > R×2，降级阈值 dist < R×1.5，
    // 两阈值之间存在死区，避免相机在边界来回导致频繁切换。
    if (!m_lodManualMode && m_sceneViz->lodEnabled() &&
        m_sceneViz->lodLevelCount() > 1) {
        osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
        if (viewer) {
            osg::Vec3d eye, center, up;
            viewer->getCamera()->getViewMatrixAsLookAt(eye, center, up);
            Eigen::Vector3d c = m_sceneViz->boundsCenter();
            double R = m_sceneViz->boundsRadius();
            if (R > 1e-6) {
                double dist = (eye - osg::Vec3d(c.x(), c.y(), c.z())).length();
                int cur = m_sceneViz->currentLodLevel();
                int maxLevel = m_sceneViz->lodLevelCount() - 1;
                int target = cur;
                if (cur < maxLevel && dist > R * 2.0) {
                    target = cur + 1;  // 拉远 → 低分辨率
                } else if (cur > 0 && dist < R * 1.5) {
                    target = cur - 1;  // 拉近 → 高分辨率
                }
                if (target != cur) {
                    m_sceneViz->setLodLevel(target);
                    // 提示层级切换（渲染面板持续显示 + 状态栏临时提示）
                    emit lodLevelChanged(target, m_sceneViz->lodLevelCount());
                    m_osgWidget->update();
                }
            }
        }
    }

    // FPS 跟踪（滚动 1 秒平均）
    m_frameCount++;
    static auto lastTime = std::chrono::steady_clock::now();
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - lastTime).count();
    if (elapsed >= 1.0f) {
        m_fps = static_cast<float>(m_frameCount) / elapsed;
        m_frameCount = 0;
        lastTime = now;
        emit fpsUpdated(m_fps);
    }
}

// ---------------------------------------------------------------------------
// 拾取
// ---------------------------------------------------------------------------

/**
 * @brief 顶点拾取回调（Ctrl+Click）
 *
 * 设置选中顶点 ID，刷新场景以显示高亮颜色，发射 vertexSelected 信号。
 */
void ViewportWidget::onVertexPicked(long vertexId) {
    m_sceneViz->setSelectedVertex(vertexId);

    // 重建球体以应用/取消高亮颜色
    if (m_graph) {
        m_sceneViz->updatePoses(m_graph);
        m_osgWidget->update();
    }

    emit vertexSelected(vertexId);
}

void ViewportWidget::selectVertex(long vertexId) {
    onVertexPicked(vertexId);
}

void ViewportWidget::highlightPlaybackVertex(long vertexId) {
    m_sceneViz->highlightPlaybackVertex(vertexId);
    m_osgWidget->update();
}

void ViewportWidget::setPlaybackRetain(bool retain) {
    m_sceneViz->setPlaybackRetain(retain);
    m_osgWidget->update();
}

// ---------------------------------------------------------------------------
// 叠加面板管理
// ---------------------------------------------------------------------------

/**
 * @brief 注册浮动叠加面板
 *
 * 将面板重新设置父级为视口，调整大小，定位并显示。
 */
void ViewportWidget::registerOverlay(OverlayPanelWidget* overlay) {
    if (!overlay) return;

    // 重新设置父级到当前视口
    overlay->setParent(this);

    // 添加到跟踪列表
    m_overlays.append(overlay);

    // 设置初始大小
    overlay->adjustSize();

    // 定位并显示
    updateOverlayPositions();
    overlay->raise();
    overlay->show();
}

/**
 * @brief 更新所有叠加面板的位置
 *
 * 将可见面板按"右上角对齐 + 纵向堆叠"方式排列。
 */
void ViewportWidget::updateOverlayPositions() {
    int yOffset = m_overlayMargin;

    for (auto* overlay : m_overlays) {
        if (!overlay->isVisible()) continue;

        int panelW = overlay->width();
        int panelH = overlay->height();

        // 锚定到右上角
        int newX = width() - panelW - m_overlayMargin;
        int newY = yOffset;

        // 钳制在视口边界内
        newX = std::max(m_overlayMargin,
                        std::min(newX, width() - panelW - m_overlayMargin));
        newY = std::max(m_overlayMargin,
                        std::min(newY, height() - panelH - m_overlayMargin));

        overlay->move(newX, newY);

        // 下一个面板在下方堆叠
        yOffset = newY + panelH + 6;
    }
}

/**
 * @brief 窗口尺寸改变事件
 *
 * 重新应用投影矩阵并更新面板位置。
 */
void ViewportWidget::resizeEvent(QResizeEvent* event) {
    QWidget::resizeEvent(event);
    applyProjection();
    updateOverlayPositions();
}
