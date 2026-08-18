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
#include <QResizeEvent>
#include <QtConcurrent/QtConcurrent>
#include <chrono>
#include <cmath>

#include <osg/Notify>
#include <osg/Math>

#include "osgQOpenGL/osgQOpenGLWidget.h"
#include "osgQOpenGL/OSGRenderer.h"
#include "backend/graph_manager.hpp"
#include "visualizers/SpherePickingHandler.h"
#include "visualizers/PointCloudBuilder.h"
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
    viewer->setCameraManipulator(new osgGA::TrackballManipulator);

    // 正交投影（替换 OSG 默认的透视投影）
    applyProjection();

    // 注册拾取处理器（使用惰性数据提供器）
    // 球心坐标和边段数据在每次事件触发时实时查询，确保图谱加载/位姿更新后自动反映
    m_pickingHandler = new SpherePickingHandler(
        // 球心坐标提供器
        [this]() -> const std::vector<std::pair<osg::Vec3d, long>>* {
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
                QPoint globalPos = m_osgWidget->mapToGlobal(
                    QPoint(static_cast<int>(hit.screenX),
                           static_cast<int>(hit.screenY)));
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
 */
void ViewportWidget::onGraphClosed() {
    m_graph.reset();
    ++m_cloudBuildSeq;  // 使在途构建结果作废
    m_sceneViz->clear();
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
    // 已有点云数据时触发后台异步重建（生成/移除多级 LOD）
    if (m_graph && m_sceneViz->hasPointCloud()) {
        requestCloudBuild();
    }
}

void ViewportWidget::setLodMode(bool manual) {
    m_lodManualMode = manual;
    if (manual) {
        // 立即应用当前手动层级（clamp 到有效范围）
        int maxLevel = m_sceneViz->lodLevelCount() - 1;
        int level = qBound(0, m_lodManualLevel, maxLevel);
        m_lodManualLevel = level;
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
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (viewer) {
        viewer->home();
    }
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
void ViewportWidget::requestCloudBuild() {
    if (!m_graph) return;
    m_cloudBuildPending = true;
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

    auto graph = m_graph;
    hdl_graph_slam::BuildOptions options;
    options.lodEnabled = m_sceneViz->lodEnabled();
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
        m_sceneViz->commitPointCloudBuild(std::move(result));

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
    }

    // 构建期间有新请求（预算变化/优化完成等）→ 用最新状态再构建一次
    if (m_cloudBuildPending) {
        startCloudBuild();
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
    // 自动选择级别（带迟滞防抖）。级别 0 = 全量（最近），级别越高点数越少。
    // 级间比例 2×：升级阈值 dist > R×2×2^k，降级阈值 dist < R×1.5×2^(k-1)，
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
                if (cur < maxLevel && dist > R * 2.0 * std::pow(2.0, cur)) {
                    target = cur + 1;  // 拉远 → 低分辨率
                } else if (cur > 0 && dist < R * 1.5 * std::pow(2.0, cur - 1)) {
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
