/**
 * @file MiniViewportWidget.h
 * @brief 迷你视口部件头文件
 *
 * 轻量级 OSG 视口（512×512），用于在 LoopClosureDialog 中
 * 预览两个关键帧点云的相对位姿对齐效果。
 *
 * 核心功能：
 * - 起点（begin）点云固定在世界原点，渲染为蓝色
 * - 终点（end）点云根据相对位姿变换渲染，渲染为绿色
 * - 显示终点处的坐标轴（XYZ 彩色箭头）
 * - 支持透视/正交投影切换
 *
 * 设计特点：
 * - OSG 几何体对象在构造函数中创建（不需要 GL 上下文）
 * - 摄像机设置（视口、场景数据）在 initOsg() 中等待 GL 上下文就绪
 * - **顶点按帧局部坐标只建一次**，终点位姿变化只改一个 osg::MatrixTransform
 *   的矩阵（updateEndPose 因此是 O(1)，与点数无关；旧实现每次调整都逐点
 *   重算并把 19.5 万点的顶点/颜色数组整份重传）
 * - 两张云各自单色：颜色数组只放 1 个元素并 BIND_OVERALL（与逐点 push 同一个
 *   颜色等价，但省掉每点 16 字节）
 */

#pragma once

#include <QWidget>
#include <osg/ref_ptr>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/Uniform>
#include <osg/Vec4>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class osgQOpenGLWidget;
namespace osgViewer { class Viewer; }

/**
 * @brief 迷你 3D 视口部件
 *
 * 512×512 的 OSG 视口，显示两帧点云的相对位姿关系。
 * 起点点云（蓝色）固定于原点，终点点云（绿色）按照相对位姿变换显示。
 * 在终点点云位置绘制坐标轴以辅助观察。
 * 用于 LoopClosureDialog 中供用户预览和调整闭环位姿。
 */
class MiniViewportWidget : public QWidget {
    Q_OBJECT
public:
    using PointT = pcl::PointXYZI;               ///< 点云数据类型
    using CloudPtr = pcl::PointCloud<PointT>::ConstPtr;  ///< 点云共享指针

    explicit MiniViewportWidget(QWidget* parent = nullptr);
    ~MiniViewportWidget() override;

    QSize sizeHint() const override { return QSize(512, 512); }

    /**
     * @brief 初始设置 — 在两个关键帧都选定后调用一次
     * @param beginCloud 起点点云
     * @param beginPose  起点在世界坐标系下的位姿
     * @param endCloud   终点点云
     * @param endPose    终点在世界坐标系下的位姿
     */
    void setClouds(CloudPtr beginCloud, const Eigen::Isometry3d& beginPose,
                   CloudPtr endCloud,   const Eigen::Isometry3d& endPose);

    /**
     * @brief 设置终点位姿（O(1)，与点数无关）
     *
     * 只把相对位姿写进 m_endTransform 的矩阵，**不动任何顶点数组**：
     * 终点云、终点球、坐标轴的顶点在 setClouds() 时已按"终点帧局部坐标"建好，
     * 位姿变化由变换节点承担。
     *
     * @param endPose   更新后的终点世界位姿
     * @param beginPose 起点世界位姿（用于计算相对变换）
     */
    void updateEndPose(const Eigen::Isometry3d& endPose,
                       const Eigen::Isometry3d& beginPose);

    /// 重置摄像机到初始位置
    void resetCamera();

    // === 投影方式 ===
    void applyOrthographicProjection();    ///< 切换为正交投影
    void applyPerspectiveProjection();     ///< 切换为透视投影
    void applyProjection();                ///< 根据标志应用对应的投影

    void setUseOrthographic(bool enabled); ///< 设置是否使用正交投影
    bool isOrthographic() const { return m_useOrthographic; }

private slots:
    void initOsg();  ///< OSG 初始化（OpenGL 上下文就绪后调用）

private:
    void setupGeometries();  ///< 在构造函数中创建几何体（不需要 GL 上下文）

    /**
     * @brief 在给定顶点数组上画一个单位球（球心在几何体局部原点，半径 0.3）
     *
     * 球体顶点只与"球心在原点"有关，因此可以只建一次、之后靠变换节点搬位置，
     * 而不必每次调整位姿都重新三角化一遍（96×97 个顶点 + 11 万条索引）。
     *
     * @param geom   目标几何体（需要已设好着色器/状态集）
     * @param color  球体颜色（起点用亮蓝、终点用亮绿，与原实现一致）
     */
    static void buildUnitSphereAtOrigin(osg::Geometry* geom, const osg::Vec4& color);

    void rebuildBeginGeometry();  ///< 建起点云 + 原点的起点球（只做一次）
    void rebuildEndGeometry();    ///< 建终点云/终点球/坐标轴（终点帧局部坐标，只做一次）

    osgQOpenGLWidget* m_osgWidget = nullptr;  ///< OSG 嵌入部件

    osg::ref_ptr<osg::Group> m_root;  ///< 场景根节点

    osg::ref_ptr<osg::Geode>   m_beginGeode;   ///< 起点云几何节点
    osg::ref_ptr<osg::Geometry> m_beginGeom;    ///< 起点云几何体
    osg::ref_ptr<osg::Geometry> m_beginSphereGeom;  ///< 原点的起点球（不随位姿变化）

    osg::ref_ptr<osg::Geode>   m_endGeode;     ///< 终点云几何节点
    osg::ref_ptr<osg::Geometry> m_endGeom;      ///< 终点云几何体（帧局部坐标，常驻）

    /**
     * @brief 终点侧几何体的位姿变换节点
     *
     * 挂 m_endGeode（终点云）与 m_endOverlayGeode（终点球 + 坐标轴）。这些几何体
     * 的顶点一律按**终点帧局部坐标**建好，位姿变化只改这个节点的矩阵——
     * 于是 updateEndPose() 是 O(1)，不再逐点重算 19.5 万个顶点、也不再重传 VBO。
     * OSG 会顺着这个节点算世界包围盒，resetCamera()/viewer->home() 取景因此仍正确。
     */
    osg::ref_ptr<osg::MatrixTransform> m_endTransform;

    osg::ref_ptr<osg::Geode>   m_endOverlayGeode;  ///< 终点叠加几何节点（球 + 坐标轴）
    osg::ref_ptr<osg::Geometry> m_endSphereGeom;   ///< 终点球几何体（局部原点，随变换平移）
    osg::ref_ptr<osg::Geometry> m_axesGeom;        ///< 坐标轴几何体（终点帧局部坐标）

    CloudPtr m_beginCloud;  ///< 起点云数据
    CloudPtr m_endCloud;    ///< 终点云数据
    bool     m_initialized = false;     ///< OSG 是否已初始化
    bool     m_useOrthographic = false; ///< 是否使用正交投影（默认：透视）
};
