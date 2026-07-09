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
 */

#pragma once

#include <QWidget>
#include <osg/ref_ptr>
#include <osg/Group>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Uniform>
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
     * @brief 在新的相对位姿下重建终点点云几何体
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
    void rebuildBeginCloud(const Eigen::Isometry3d& pose);  ///< 重建起点云几何体
    void rebuildEndCloud(const Eigen::Isometry3d& relPose); ///< 重建终点云几何体
    void rebuildOverlay(const Eigen::Isometry3d& relPose);  ///< 重建叠加几何体（球体+坐标轴）

    osgQOpenGLWidget* m_osgWidget = nullptr;  ///< OSG 嵌入部件

    osg::ref_ptr<osg::Group> m_root;  ///< 场景根节点

    osg::ref_ptr<osg::Geode>   m_beginGeode;   ///< 起点云几何节点
    osg::ref_ptr<osg::Geometry> m_beginGeom;    ///< 起点云几何体

    osg::ref_ptr<osg::Geode>   m_endGeode;     ///< 终点云几何节点
    osg::ref_ptr<osg::Geometry> m_endGeom;      ///< 终点云几何体

    osg::ref_ptr<osg::Geode>   m_overlayGeode;  ///< 叠加几何节点
    osg::ref_ptr<osg::Geometry> m_sphereGeom;   ///< 球体几何体（起点/终点位置标记）
    osg::ref_ptr<osg::Geometry> m_axesGeom;     ///< 坐标轴几何体（终点位置）

    CloudPtr m_beginCloud;  ///< 起点云数据
    CloudPtr m_endCloud;    ///< 终点云数据
    bool     m_initialized = false;     ///< OSG 是否已初始化
    bool     m_useOrthographic = false; ///< 是否使用正交投影（默认：透视）
};
