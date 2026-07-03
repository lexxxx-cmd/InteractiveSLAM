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

/// @brief Lightweight OSG viewport (512×512) that renders two keyframe point
///        clouds in RELATIVE pose — begin (blue) at origin, end (green) at
///        rel = beginPose⁻¹·endPose.  Used inside LoopClosureDialog.
///
/// Geometry objects are created in the constructor (no GL context needed);
/// only viewer setup (camera, scene data) waits for initOsg().
class MiniViewportWidget : public QWidget {
    Q_OBJECT
public:
    using PointT = pcl::PointXYZI;
    using CloudPtr = pcl::PointCloud<PointT>::ConstPtr;

    explicit MiniViewportWidget(QWidget* parent = nullptr);
    ~MiniViewportWidget() override;

    QSize sizeHint() const override { return QSize(512, 512); }

    /// Initial setup — called once after both keyframes are selected.
    void setClouds(CloudPtr beginCloud, const Eigen::Isometry3d& beginPose,
                   CloudPtr endCloud,   const Eigen::Isometry3d& endPose);

    /// Rebuild the end-keyframe geometry at the new relative pose.
    void updateEndPose(const Eigen::Isometry3d& endPose,
                       const Eigen::Isometry3d& beginPose);

    /// Reset camera to home position.
    void resetCamera();

    /// Replace the default perspective projection with orthographic.
    void applyOrthographicProjection();

private slots:
    void initOsg();

private:
    void setupGeometries();  // called from constructor (no GL context needed)
    void rebuildBeginCloud(const Eigen::Isometry3d& pose);
    void rebuildEndCloud(const Eigen::Isometry3d& relPose);
    void rebuildOverlay(const Eigen::Isometry3d& relPose);

    osgQOpenGLWidget* m_osgWidget = nullptr;

    osg::ref_ptr<osg::Group> m_root;

    osg::ref_ptr<osg::Geode>   m_beginGeode;
    osg::ref_ptr<osg::Geometry> m_beginGeom;

    osg::ref_ptr<osg::Geode>   m_endGeode;
    osg::ref_ptr<osg::Geometry> m_endGeom;

    osg::ref_ptr<osg::Geode>   m_overlayGeode;
    osg::ref_ptr<osg::Geometry> m_sphereGeom;
    osg::ref_ptr<osg::Geometry> m_axesGeom;

    CloudPtr m_beginCloud;
    CloudPtr m_endCloud;
    bool     m_initialized = false;
};
