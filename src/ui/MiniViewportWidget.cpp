#include "ui/MiniViewportWidget.h"

#include <QVBoxLayout>
#include <osg/Array>
#include <osg/PrimitiveSet>
#include <osg/StateSet>
#include <osg/LineWidth>
#include <osgGA/TrackballManipulator>
#include <osgViewer/Viewer>

#include "osgQOpenGL/osgQOpenGLWidget.h"
#include "osgQOpenGL/OSGRenderer.h"
#include "visualizers/CoreShaders.h"

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

MiniViewportWidget::MiniViewportWidget(QWidget* parent)
    : QWidget(parent) {

    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    m_osgWidget = new osgQOpenGLWidget(this);
    m_osgWidget->setFixedSize(512, 512);
    m_osgWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    layout->addWidget(m_osgWidget);

    // Create geometries NOW (no GL context needed for osg::Geometry)
    setupGeometries();

    connect(m_osgWidget, &osgQOpenGLWidget::initialized,
            this, &MiniViewportWidget::initOsg);
}

MiniViewportWidget::~MiniViewportWidget() = default;

// ---------------------------------------------------------------------------
// Orthographic projection helper
// ---------------------------------------------------------------------------

void MiniViewportWidget::applyOrthographicProjection() {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;

    osg::Camera* camera = viewer->getCamera();

    // Fixed 512×512 viewport — aspect is always 1:1
    double halfHeight = 10.0;  // default
    osg::Node* scene = viewer->getSceneData();
    if (scene) {
        const osg::BoundingSphere& bs = scene->getBound();
        if (bs.valid() && bs.radius() > 0.0) {
            halfHeight = bs.radius() * 1.2;
        }
    }

    double farDist = halfHeight * 20.0;

    camera->setProjectionMatrixAsOrtho(
        -halfHeight, halfHeight,
        -halfHeight, halfHeight,
        0.1, farDist);
}

// ---------------------------------------------------------------------------
// Perspective projection helper
// ---------------------------------------------------------------------------

void MiniViewportWidget::applyPerspectiveProjection() {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;

    osg::Camera* camera = viewer->getCamera();
    double aspect = 1.0;   // fixed 512×512
    double fovY   = 30.0;  // degrees

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
// Unified projection dispatch
// ---------------------------------------------------------------------------

void MiniViewportWidget::applyProjection() {
    if (m_useOrthographic) {
        applyOrthographicProjection();
    } else {
        applyPerspectiveProjection();
    }
}

void MiniViewportWidget::setUseOrthographic(bool enabled) {
    if (m_useOrthographic == enabled) return;
    m_useOrthographic = enabled;
    applyProjection();
    m_osgWidget->update();
}

// ---------------------------------------------------------------------------
// Geometry setup — called from constructor, BEFORE GL context exists
// ---------------------------------------------------------------------------

void MiniViewportWidget::setupGeometries() {
    m_root = new osg::Group;

    // --- Begin cloud geode ---
    m_beginGeode = new osg::Geode;
    m_beginGeom = new osg::Geometry;
    m_beginGeom->setUseDisplayList(false);
    m_beginGeom->setUseVertexBufferObjects(true);
    m_beginGeom->setUseVertexArrayObject(true);
    m_beginGeom->setDataVariance(osg::Object::STATIC);
    applyPointCloudShader(m_beginGeom->getOrCreateStateSet(), 2.0f);
    m_beginGeode->addDrawable(m_beginGeom);
    m_root->addChild(m_beginGeode);

    // --- End cloud geode ---
    m_endGeode = new osg::Geode;
    m_endGeom = new osg::Geometry;
    m_endGeom->setUseDisplayList(false);
    m_endGeom->setUseVertexBufferObjects(true);
    m_endGeom->setUseVertexArrayObject(true);
    m_endGeom->setDataVariance(osg::Object::DYNAMIC);
    applyPointCloudShader(m_endGeom->getOrCreateStateSet(), 2.0f);
    m_endGeode->addDrawable(m_endGeom);
    m_root->addChild(m_endGeode);

    // --- Overlay geode (spheres + axes) ---
    m_overlayGeode = new osg::Geode;

    // Sphere geometry (GL_TRIANGLES)
    m_sphereGeom = new osg::Geometry;
    m_sphereGeom->setUseDisplayList(false);
    m_sphereGeom->setUseVertexBufferObjects(true);
    m_sphereGeom->setUseVertexArrayObject(true);
    m_sphereGeom->setDataVariance(osg::Object::DYNAMIC);
    applySimpleColorShader(m_sphereGeom->getOrCreateStateSet());
    m_overlayGeode->addDrawable(m_sphereGeom);

    // Axis geometry (GL_LINES)
    m_axesGeom = new osg::Geometry;
    m_axesGeom->setUseDisplayList(false);
    m_axesGeom->setUseVertexBufferObjects(true);
    m_axesGeom->setUseVertexArrayObject(true);
    m_axesGeom->setDataVariance(osg::Object::DYNAMIC);
    {
        auto* ss = m_axesGeom->getOrCreateStateSet();
        applySimpleColorShader(ss);
        ss->setAttributeAndModes(new osg::LineWidth(2.0f),
                                 osg::StateAttribute::ON);
    }
    m_overlayGeode->addDrawable(m_axesGeom);

    m_root->addChild(m_overlayGeode);
}

// ---------------------------------------------------------------------------
// OSG Viewer setup — called when GL context is ready
// ---------------------------------------------------------------------------

void MiniViewportWidget::initOsg() {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;

    osg::State* state = viewer->getCamera()->getGraphicsContext()->getState();
    if (state) {
        state->setUseModelViewAndProjectionUniforms(true);
        state->setUseVertexAttributeAliasing(true);
    }

    // Dark background
    viewer->getCamera()->setClearColor(osg::Vec4(0.1f, 0.1f, 0.12f, 1.0f));

    // Trackball camera
    viewer->setCameraManipulator(new osgGA::TrackballManipulator);

    // Perspective/orthographic projection (toggled via UI)
    applyProjection();

    // Attach the pre-built scene graph
    viewer->setSceneData(m_root);
    m_initialized = true;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void MiniViewportWidget::setClouds(CloudPtr beginCloud,
                                    const Eigen::Isometry3d& beginPose,
                                    CloudPtr endCloud,
                                    const Eigen::Isometry3d& endPose) {
    m_beginCloud = beginCloud;
    m_endCloud   = endCloud;

    Eigen::Isometry3d rel = beginPose.inverse() * endPose;

    rebuildBeginCloud(Eigen::Isometry3d::Identity());  // begin at origin
    rebuildEndCloud(rel);
    rebuildOverlay(rel);

    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (viewer) {
        applyProjection();
        viewer->home();
    }
    m_osgWidget->update();
}

void MiniViewportWidget::updateEndPose(const Eigen::Isometry3d& endPose,
                                        const Eigen::Isometry3d& beginPose) {
    Eigen::Isometry3d rel = beginPose.inverse() * endPose;
    rebuildEndCloud(rel);
    rebuildOverlay(rel);
    m_osgWidget->update();
}

void MiniViewportWidget::resetCamera() {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (viewer) {
        viewer->home();
    }
}

// ---------------------------------------------------------------------------
// Internal: rebuild geometry
// ---------------------------------------------------------------------------

void MiniViewportWidget::rebuildBeginCloud(const Eigen::Isometry3d& pose) {
    if (!m_beginCloud || m_beginCloud->empty() || !m_beginGeom) return;

    auto* verts = new osg::Vec3Array;
    auto* colors = new osg::Vec4Array;
    verts->reserve(m_beginCloud->size());
    colors->reserve(m_beginCloud->size());

    osg::Vec4 blue(0.0f, 0.0f, 1.0f, 1.0f);
    for (const auto& pt : m_beginCloud->points) {
        Eigen::Vector3d p = pose * Eigen::Vector3d(pt.x, pt.y, pt.z);
        verts->push_back(osg::Vec3(p.x(), p.y(), p.z()));
        colors->push_back(blue);
    }

    m_beginGeom->setVertexArray(verts);
    m_beginGeom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
    m_beginGeom->removePrimitiveSet(0, m_beginGeom->getNumPrimitiveSets());
    m_beginGeom->addPrimitiveSet(
        new osg::DrawArrays(GL_POINTS, 0, verts->size()));
    m_beginGeom->dirtyBound();
}

void MiniViewportWidget::rebuildEndCloud(const Eigen::Isometry3d& relPose) {
    if (!m_endCloud || m_endCloud->empty() || !m_endGeom) return;

    auto* verts = new osg::Vec3Array;
    auto* colors = new osg::Vec4Array;
    verts->reserve(m_endCloud->size());
    colors->reserve(m_endCloud->size());

    osg::Vec4 green(0.0f, 1.0f, 0.0f, 1.0f);
    for (const auto& pt : m_endCloud->points) {
        Eigen::Vector3d p = relPose * Eigen::Vector3d(pt.x, pt.y, pt.z);
        verts->push_back(osg::Vec3(p.x(), p.y(), p.z()));
        colors->push_back(green);
    }

    m_endGeom->setVertexArray(verts);
    m_endGeom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
    m_endGeom->removePrimitiveSet(0, m_endGeom->getNumPrimitiveSets());
    m_endGeom->addPrimitiveSet(
        new osg::DrawArrays(GL_POINTS, 0, verts->size()));
    m_endGeom->dirtyBound();
}

void MiniViewportWidget::rebuildOverlay(const Eigen::Isometry3d& relPose) {
    if (!m_sphereGeom || !m_axesGeom) return;

    // --- Spheres (solid, via ring+sector tessellation) ---
    {
        const float pi = 3.14159265f;
        const float radius = 0.3f;
        const int rings = 12;
        const int sectors = 12;

        auto* verts = new osg::Vec3Array;
        auto* colors = new osg::Vec4Array;
        auto* indices = new osg::DrawElementsUInt(GL_TRIANGLES);

        osg::Vec4 blue(0.2f, 0.2f, 1.0f, 1.0f);
        osg::Vec4 green(0.2f, 1.0f, 0.2f, 1.0f);

        osg::Vec3 centers[2] = {
            osg::Vec3(0, 0, 0),
            osg::Vec3(relPose.translation().x(),
                      relPose.translation().y(),
                      relPose.translation().z())
        };
        osg::Vec4 sphereColors[2] = { blue, green };

        for (int si = 0; si < 2; ++si) {
            unsigned int base = verts->size();
            for (int r = 0; r <= rings; ++r) {
                float phi = float(r) * pi / float(rings);
                float sinPhi = std::sin(phi);
                float cosPhi = std::cos(phi);
                for (int s = 0; s <= sectors; ++s) {
                    float theta = float(s) * 2.0f * pi / float(sectors);
                    float sinT = std::sin(theta);
                    float cosT = std::cos(theta);
                    verts->push_back(osg::Vec3(
                        centers[si].x() + radius * sinPhi * cosT,
                        centers[si].y() + radius * cosPhi,
                        centers[si].z() + radius * sinPhi * sinT));
                    colors->push_back(sphereColors[si]);
                }
            }
            for (int r = 0; r < rings; ++r) {
                for (int s = 0; s < sectors; ++s) {
                    unsigned int a = base + r * (sectors + 1) + s;
                    unsigned int b = a + sectors + 1;
                    indices->push_back(a);
                    indices->push_back(b);
                    indices->push_back(a + 1);
                    indices->push_back(b);
                    indices->push_back(b + 1);
                    indices->push_back(a + 1);
                }
            }
        }

        m_sphereGeom->setVertexArray(verts);
        m_sphereGeom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
        m_sphereGeom->removePrimitiveSet(0, m_sphereGeom->getNumPrimitiveSets());
        m_sphereGeom->addPrimitiveSet(indices);
        m_sphereGeom->dirtyBound();
    }

    // --- Coordinate axes at end-keyframe position (1.5× scale) ---
    {
        Eigen::Vector3d t = relPose.translation();
        Eigen::Matrix3d R = relPose.linear();
        double s = 1.5;

        auto* verts = new osg::Vec3Array;
        auto* colors = new osg::Vec4Array;

        osg::Vec3 origin(t.x(), t.y(), t.z());
        osg::Vec4 red(1, 0, 0, 1), grn(0, 1, 0, 1), blu(0, 0, 1, 1);

        Eigen::Vector3d axX = R.col(0) * s;
        Eigen::Vector3d axY = R.col(1) * s;
        Eigen::Vector3d axZ = R.col(2) * s;

        // X axis (red)
        verts->push_back(origin);
        verts->push_back(osg::Vec3(t.x() + axX.x(), t.y() + axX.y(), t.z() + axX.z()));
        colors->push_back(red); colors->push_back(red);

        // Y axis (green)
        verts->push_back(origin);
        verts->push_back(osg::Vec3(t.x() + axY.x(), t.y() + axY.y(), t.z() + axY.z()));
        colors->push_back(grn); colors->push_back(grn);

        // Z axis (blue)
        verts->push_back(origin);
        verts->push_back(osg::Vec3(t.x() + axZ.x(), t.y() + axZ.y(), t.z() + axZ.z()));
        colors->push_back(blu); colors->push_back(blu);

        m_axesGeom->setVertexArray(verts);
        m_axesGeom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
        m_axesGeom->removePrimitiveSet(0, m_axesGeom->getNumPrimitiveSets());
        m_axesGeom->addPrimitiveSet(
            new osg::DrawArrays(GL_LINES, 0, verts->size()));
        m_axesGeom->dirtyBound();
    }
}
