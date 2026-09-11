/**
 * @file ViewportWidget.h
 * @brief 3D 视口部件头文件
 *
 * ViewportWidget 是应用程序中央的 3D 渲染视口，基于 osgQOpenGLWidget 和
 * OpenSceneGraph (OSG) 构建。它是整个 SLAM 数据可视化最核心的 UI 组件。
 *
 * 主要功能：
 * - 管理 OSG 场景图（通过 GraphSceneVisualizer）
 * - 支持透视/正交投影切换
 * - 顶点拾取（Ctrl+Click）和右键上下文菜单
 * - 管理浮动叠加面板（OverlayPanelWidget）的位置
 * - 提供渲染控制接口（显示/隐藏顶点、边、点云等）
 * - FPS 统计
 */

#pragma once

#include <QWidget>
#include <QTimer>
#include <QFutureWatcher>
#include <memory>
#include <set>
#include <vector>
#include <osg/Group>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include "visualizers/GraphSceneVisualizer.h"
#include "visualizers/PointCloudBuilder.h"
#include "ui/DrawFlags.h"

class osgQOpenGLWidget;
class GraphManager;
class OverlayPanelWidget;
class SpherePickingHandler;
class FirstPersonManipulator;

namespace hdl_graph_slam {
class InteractiveGraph;
}

/**
 * @brief 中央 3D 视口部件
 *
 * 基于 osgQOpenGLWidget 实现，在 Qt 界面中嵌入 OSG 3D 渲染。
 * 设计参考 3DPCViewer 的 VisualAreaWidget。
 *
 * 使用方式：
 * - 构造后自动创建 OSG 渲染环境和场景可视化器
 * - 通过 onGraphLoaded() 加载图谱数据到场景
 * - 通过各类 setter 方法控制渲染效果
 * - 信号 vertexSelected / contextMenuRequested 供 MainWindow 使用
 */
class ViewportWidget : public QWidget {
    Q_OBJECT

public:
    explicit ViewportWidget(QWidget* parent = nullptr);
    ~ViewportWidget() override;

public slots:
    /**
     * @brief 程序化选中指定顶点（等价于 Ctrl+Click）
     *
     * 用于播放轴功能，效果与 Ctrl+Click 选中关键帧完全相同：
     * 选中视锥体变红色（2 倍尺寸）+ 邻域点云高亮 + 发射 vertexSelected 信号。
     *
     * @param vertexId 要选中的顶点 ID（-1 取消选择）
     */
    void selectVertex(long vertexId);

    /**
     * @brief 轻量级播放高亮（不重建球体几何体）
     *
     * 用于播放轴滑块拖动/自动播放时实时更新球体颜色和点云高亮。
     * 只更新颜色数组，不触发完整的球体几何体重建。
     *
     * @param vertexId 顶点 ID（-1 取消高亮）
     */
    void highlightPlaybackVertex(long vertexId);

    /**
     * @brief 设置播放累积高亮开关（"播放点云留存"）
     *
     * 打开后播放过的帧点云保留白色高亮（不随播放推进消失），
     * 关闭或播放会话结束时清空留存。
     *
     * @param retain true = 累积留存模式
     */
    void setPlaybackRetain(bool retain);

    /**
     * @brief 图谱加载完成后的回调
     * @param graph 加载的图谱对象（共享指针）
     */
    void onGraphLoaded(std::shared_ptr<hdl_graph_slam::InteractiveGraph> graph);

    /// 图谱关闭后的回调（清空场景）
    void onGraphClosed();

    /// 刷新场景（更新顶点/边位姿）
    void refreshScene();

    /**
     * @brief 重建点云（异步）
     * @note 计算密集的 CPU 变换/降采样在后台线程执行，完成后在主线程
     *       换入场景，不阻塞 UI。多次调用会自动合并为一次构建。
     */
    void rebuildPointClouds();

    // === 渲染控制 ===
    void setDrawVertices(bool v);            ///< 是否绘制顶点
    void setDrawEdges(bool v);               ///< 是否绘制边
    void setDrawKeyframeClouds(bool v);      ///< 是否绘制关键帧点云
    void setDrawSE3Edges(bool v);            ///< 是否绘制 SE3 约束边
    void setEdgeWidth(int width);            ///< 设置边线宽度
    void setSphereRadius(float radius);      ///< 设置顶点球体半径
    void setVertexOpacity(int opacity);      ///< 设置顶点位姿标记不透明度（0-100）
    void setSampleStride(int stride);        ///< 设置渲染采样步长
    int  sampleStride() const { return m_flags.sample_stride; }  ///< 当前采样步长
    void setPointSize(int size);             ///< 设置点云点大小
    void setPointOpacity(int opacity);       ///< 设置点云不透明度（0-100）
    void setLodEnabled(bool enabled);        ///< 设置 LOD 多级渲染开关
    void setOdomLayerEnabled(bool enabled);  ///< 设置原始层（里程计位姿参照底图）开关

    /**
     * @brief 设置 LOD 切换模式
     * @param manual true = 手动固定层级（由 setLodManualLevel 控制）；
     *               false = 按相机距离自动切换（默认）
     */
    void setLodMode(bool manual);

    /**
     * @brief 手动指定 LOD 层级（仅手动模式下生效）
     * @param level 目标级别（0 = 全量，1 = 第一层降采样）
     */
    void setLodManualLevel(int level);
    void setBackgroundColor(const QColor& color);  ///< 设置背景色
    void setHiddenEdges(const std::set<long>& ids);     ///< 设置隐藏边集合
    void setLoopHighlight(long sourceId, const std::vector<long>& candidateIds);  ///< 闭环高亮
    void resetCamera();                      ///< 重置摄像机
    void restoreWheelZoomFactor();           ///< 恢复聚焦时提高的滚轮缩放系数

    // === 第一人称模式（Shift 切换） ===
    void enterFirstPersonMode();             ///< 进入第一人称模式
    void exitFirstPersonMode();              ///< 退出并恢复轨迹球相机（位姿无缝衔接）
    bool firstPersonActive() const { return m_fpActive; }

    /**
     * @brief 双击聚焦：相机移动到指定位姿球体局部 x 轴负方向，
     *        视线沿位姿朝向（局部 x 轴）看向球心，球心位于视角中心
     * @param vertexId 目标顶点 ID
     */
    void focusOnVertex(long vertexId);
    void onFocusPoint(const osg::Vec3d& point);  ///< 双击点云居中（旋转中心=命中点）
    void onFrameView(long vertexId);             ///< Ctrl+双击：切换到帧位姿视角

    // === Z 裁剪 + 高程颜色范围 ===
    void setZClipping(bool enabled);          ///< 启用/禁用 Z 裁剪
    void setZClipMin(double minZ);            ///< 设置 Z 裁剪最小值
    void setZClipMax(double maxZ);            ///< 设置 Z 裁剪最大值
    void setColorZMin(double minZ);           ///< 设置颜色范围 Z 最小值
    void setColorZMax(double maxZ);           ///< 设置颜色范围 Z 最大值
    void setAutoColorRange(bool autoRange);   ///< 启用/禁用自动颜色范围

    /**
     * @brief 设置 Ctrl+Click 点云高亮的窗口半宽
     * @param n 邻域窗口半宽
     */
    void setHighlightWindowHalf(int n);

    // === 投影方式 ===
    void applyOrthographicProjection();   ///< 切换为正交投影
    void applyPerspectiveProjection();    ///< 切换为透视投影
    void applyProjection();               ///< 根据 m_useOrthographic 标志应用投影

    /**
     * @brief 设置是否使用正交投影
     * @param enabled true=正交, false=透视
     */
    void setUseOrthographic(bool enabled);
    bool isOrthographic() const { return m_useOrthographic; }

    // === 叠加面板管理 ===
    void registerOverlay(OverlayPanelWidget* overlay);  ///< 注册浮动面板
    void updateOverlayPositions();                       ///< 更新所有面板位置

signals:
    void fpsUpdated(float fps);                           ///< FPS 更新信号（~1Hz）
    void initialized();                                   ///< OSG 初始化完成信号
    void cloudDataReady(float dataZMin, float dataZMax);  ///< 点云数据范围就绪信号
    void vertexSelected(long vertexId);                   ///< 顶点选中信号

    /**
     * @brief 点云全部渲染完成信号
     *
     * 在点云分块渐进上传全部完成时发射（含"无点云/构建被丢弃"的兜底，
     * 保证 UI 的加载指示一定能结束）。MainWindow 据此停止加载动画。
     */
    void cloudRenderFinished();

    /**
     * @brief LOD 层级状态信号
     *
     * 在以下时机发射：
     *   - LOD 多级构建完成（level=0，levelCount>1 表示 LOD 已就绪）；
     *   - 相机距离变化导致层级切换（level 为切换后的目标级别）。
     * 供渲染面板持续显示当前层级、状态栏临时提示切换。
     *
     * @param level      当前级别（0 = 主级别/全量，越大点数越少）
     * @param levelCount 级别总数（1 = 未启用 LOD）
     */
    void lodLevelChanged(int level, int levelCount);

    /**
     * @brief 采样步长变化信号
     *
     * 当用户通过 RenderingPanel 调整 sampleStride 时发射，
     * PlaybackPanel 监听此信号以重建采样后的播放帧列表。
     */
    void sampleStrideChanged(int stride);

    /**
     * @brief 第一人称模式切换信号（供状态栏提示操作方式）
     * @param active true = 进入第一人称模式，false = 退出
     */
    void firstPersonModeChanged(bool active);

    /**
     * @brief 右键上下文菜单请求信号
     * @param vertexId 选中顶点 ID（-1 表示未选中顶点）
     * @param edgeId   选中边 ID（-1 表示未选中边）
     * @param edgeV1   边起点顶点 ID
     * @param edgeV2   边终点顶点 ID
     * @param edgeDist 边长度
     * @param edgeKernel 边使用的鲁棒核函数
     * @param screenPos 右键点击的屏幕位置
     * @param vtxCloudSize 顶点点云规模
     * @param vtxPosX/Y/Z 顶点位置
     * @param vtxAccumDist 顶点累积距离
     * @param vtxDegree    顶点度数（关联边数）
     */
    void contextMenuRequested(long vertexId, long edgeId,
                              long edgeV1, long edgeV2,
                              double edgeDist, const QString& edgeKernel,
                              QPoint screenPos,
                              long vtxCloudSize,
                              double vtxPosX, double vtxPosY, double vtxPosZ,
                              double vtxAccumDist, int vtxDegree);

protected:
    void resizeEvent(QResizeEvent* event) override;
    bool eventFilter(QObject* watched, QEvent* event) override; ///< 拦截第一人称模式的键盘事件（Shift 切换 / WASD 行走）

private slots:
    void initOsg();          ///< OSG 初始化（osgQOpenGLWidget 准备就绪后调用）
    void updateScene();      ///< 定时场景更新（姿态/边刷新 + FPS 统计）
    void onVertexPicked(long vertexId);  ///< 顶点选中回调（Ctrl+Click）
    void onCloudBuildFinished();         ///< 后台点云构建完成回调（主线程）

private:
    // === 异步点云构建调度 ===
    void requestCloudBuild();   ///< 请求一次点云构建（运行中则合并为一次）
    void startCloudBuild();     ///< 启动后台构建任务
    osgQOpenGLWidget* m_osgWidget = nullptr;                      ///< OSG 嵌入 Qt 的 OpenGL 部件
    std::unique_ptr<GraphSceneVisualizer> m_sceneViz;             ///< 场景可视化器
    std::shared_ptr<hdl_graph_slam::InteractiveGraph> m_graph;    ///< 当前加载的图谱
    hdl_graph_slam::DrawFlags m_flags;                             ///< 绘制标志

    QTimer* m_updateTimer = nullptr;                              ///< 场景更新定时器（~60Hz）
    osg::ref_ptr<SpherePickingHandler> m_pickingHandler;           ///< 球体拾取事件处理器

    // 叠加面板（浮动于视口之上）
    QVector<OverlayPanelWidget*> m_overlays;  ///< 注册的叠加面板列表
    int m_overlayMargin = 10;                 ///< 面板边缘间距

    // 正交投影跟踪
    double m_orthoHalfHeight = 10.0;   ///< 正交投影半高
    bool m_useOrthographic = false;    ///< 是否使用正交投影（默认：透视）

    // FPS 跟踪（滚动平均）
    int m_frameCount = 0;    ///< 帧计数器
    float m_fpsAccum = 0.0f;  ///< FPS 累加器
    float m_fps = 0.0f;       ///< 当前 FPS 值

    // 异步点云构建状态
    QFutureWatcher<hdl_graph_slam::PointCloudBuildResult>* m_cloudBuildWatcher = nullptr; ///< 后台构建监视器
    int m_cloudBuildSeq = 0;         ///< 构建版本号（图变化时递增，使在途结果作废）
    int m_cloudBuildActiveSeq = -1;  ///< 当前在途构建对应的版本号
    bool m_cloudBuildRunning = false; ///< 是否有构建任务正在运行
    bool m_cloudBuildPending = false; ///< 构建期间是否收到新的构建请求（合并用）
    bool m_chunkUploadWasPending = false; ///< 上一帧是否有点云分块在渐进上传（完成检测用）

    // LOD 模式状态
    bool m_lodManualMode = true;  ///< true = 手动固定层级（默认手动，不随距离自动切换）
    int  m_lodManualLevel = 1;    ///< 手动模式下固定的目标层级（默认 level 1 = 第一层降采样）

    // 双击聚焦状态
    double m_savedWheelZoomFactor = -1.0; ///< 聚焦前的滚轮缩放系数（-1 = 未修改）

    // 第一人称模式状态
    osg::ref_ptr<FirstPersonManipulator> m_fpManip;   ///< 第一人称操作器
    osg::ref_ptr<osgGA::CameraManipulator> m_savedManip; ///< 进入前保存的轨迹球操作器
    bool m_fpActive = false;                          ///< 是否处于第一人称模式
};
