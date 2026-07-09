/**
 * @file MiniViewportWidget.cpp
 * @brief 迷你视口部件实现
 *
 * 实现一个 512×512 的 OSG 迷你视口，用于在 LoopClosureDialog 中
 * 预览两个关键帧点云之间的相对位姿。
 *
 * 主要功能：
 * - 起点（蓝色）和终点（绿色）两帧点云的相对位姿显示
 * - 终点处坐标轴（XYZ 颜色箭头）显示
 * - 支持透视/正交投影切换
 * - 以环面+扇区细分方式绘制球体作为顶点标记
 */

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
// 构造 / 析构
// ---------------------------------------------------------------------------

/**
 * @brief 构造函数
 *
 * 创建 OSG 嵌入部件（固定 512×512），在构造函数中预先创建几何体
 * （不需要 GL 上下文即可创建 osg::Geometry 对象），
 * 然后连接 OSG 初始化信号。
 */
MiniViewportWidget::MiniViewportWidget(QWidget* parent)
    : QWidget(parent) {

    // 无边距布局
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(0, 0, 0, 0);

    // 创建固定大小的 OSG 嵌入部件
    m_osgWidget = new osgQOpenGLWidget(this);
    m_osgWidget->setFixedSize(512, 512);
    m_osgWidget->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::Fixed);
    layout->addWidget(m_osgWidget);

    // 现在创建几何体（无需 GL 上下文）
    setupGeometries();

    // 连接 OSG 初始化信号
    connect(m_osgWidget, &osgQOpenGLWidget::initialized,
            this, &MiniViewportWidget::initOsg);
}

MiniViewportWidget::~MiniViewportWidget() = default;

// ---------------------------------------------------------------------------
// 正交投影辅助
// ---------------------------------------------------------------------------

/**
 * @brief 应用正交投影
 *
 * 固定 512×512 视口，宽高比始终为 1:1。
 */
void MiniViewportWidget::applyOrthographicProjection() {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;

    osg::Camera* camera = viewer->getCamera();

    // 固定 512×512 视口 — 宽高比始终为 1:1
    double halfHeight = 10.0;  // 默认值
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
// 透视投影辅助
// ---------------------------------------------------------------------------

void MiniViewportWidget::applyPerspectiveProjection() {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;

    osg::Camera* camera = viewer->getCamera();
    double aspect = 1.0;   // 固定 512×512
    double fovY   = 30.0;  // 度

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
// 几何体设置 — 在构造函数中调用（GL 上下文存在之前）
// ---------------------------------------------------------------------------

/**
 * @brief 创建场景几何体结构（不依赖 GL 上下文）
 *
 * 创建场景根节点、起点云/终点云/叠加层三个 Geode 及其 Geometry 对象。
 * 设置顶点数组对象和缓冲区对象的使能以获得更好性能。
 */
void MiniViewportWidget::setupGeometries() {
    m_root = new osg::Group;

    // --- 起点云几何节点（蓝色） ---
    m_beginGeode = new osg::Geode;
    m_beginGeom = new osg::Geometry;
    m_beginGeom->setUseDisplayList(false);
    m_beginGeom->setUseVertexBufferObjects(true);
    m_beginGeom->setUseVertexArrayObject(true);
    m_beginGeom->setDataVariance(osg::Object::STATIC);   // 起点不变
    applyPointCloudShader(m_beginGeom->getOrCreateStateSet(), 2.0f);
    m_beginGeode->addDrawable(m_beginGeom);
    m_root->addChild(m_beginGeode);

    // --- 终点云几何节点（绿色） ---
    m_endGeode = new osg::Geode;
    m_endGeom = new osg::Geometry;
    m_endGeom->setUseDisplayList(false);
    m_endGeom->setUseVertexBufferObjects(true);
    m_endGeom->setUseVertexArrayObject(true);
    m_endGeom->setDataVariance(osg::Object::DYNAMIC);    // 终点会随位姿变化
    applyPointCloudShader(m_endGeom->getOrCreateStateSet(), 2.0f);
    m_endGeode->addDrawable(m_endGeom);
    m_root->addChild(m_endGeode);

    // --- 叠加几何节点（球体 + 坐标轴） ---
    m_overlayGeode = new osg::Geode;

    // 球体几何体（GL_TRIANGLES，环面+扇区细分）
    m_sphereGeom = new osg::Geometry;
    m_sphereGeom->setUseDisplayList(false);
    m_sphereGeom->setUseVertexBufferObjects(true);
    m_sphereGeom->setUseVertexArrayObject(true);
    m_sphereGeom->setDataVariance(osg::Object::DYNAMIC);
    applySimpleColorShader(m_sphereGeom->getOrCreateStateSet());
    m_overlayGeode->addDrawable(m_sphereGeom);

    // 坐标轴几何体（GL_LINES）
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
// OSG 视口设置 — 在 GL 上下文就绪时调用
// ---------------------------------------------------------------------------

/**
 * @brief OSG 初始化
 *
 * 设置状态、清除色、轨迹球操作器、投影矩阵，
 * 并将预构建的场景图挂载到视口中。
 */
void MiniViewportWidget::initOsg() {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (!viewer) return;

    // 启用模型视图和投影统一变量
    osg::State* state = viewer->getCamera()->getGraphicsContext()->getState();
    if (state) {
        state->setUseModelViewAndProjectionUniforms(true);
        state->setUseVertexAttributeAliasing(true);
    }

    // 深色背景
    viewer->getCamera()->setClearColor(osg::Vec4(0.0706f, 0.0706f, 0.0863f, 1.0f));

    // 轨迹球摄像机
    viewer->setCameraManipulator(new osgGA::TrackballManipulator);

    // 透视/正交投影（通过 UI 切换）
    applyProjection();

    // 挂载预构建的场景图
    viewer->setSceneData(m_root);
    m_initialized = true;
}

// ---------------------------------------------------------------------------
// 公开 API
// ---------------------------------------------------------------------------

/**
 * @brief 设置起点和终点的点云及位姿
 *
 * 计算相对位姿 rel = beginPose⁻¹ * endPose，
 * 在相对坐标系下重建两个点云和叠加层，然后使摄像机定格居中。
 */
void MiniViewportWidget::setClouds(CloudPtr beginCloud,
                                    const Eigen::Isometry3d& beginPose,
                                    CloudPtr endCloud,
                                    const Eigen::Isometry3d& endPose) {
    m_beginCloud = beginCloud;
    m_endCloud   = endCloud;

    // 计算相对位姿：终点在起点坐标系下的位姿
    Eigen::Isometry3d rel = beginPose.inverse() * endPose;

    // 起点云在原点（没有相对变换），终点云在相对位姿处
    rebuildBeginCloud(Eigen::Isometry3d::Identity());  // 起点在原点
    rebuildEndCloud(rel);
    rebuildOverlay(rel);

    // 更新视口
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (viewer) {
        applyProjection();
        viewer->home();
    }
    m_osgWidget->update();
}

/**
 * @brief 更新终点位姿
 *
 * 重新计算相对位姿，重建终点点云和叠加层。
 * @param endPose   新的终点世界位姿
 * @param beginPose 起点世界位姿
 */
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
// 内部：重建几何体
// ---------------------------------------------------------------------------

/**
 * @brief 重建起点云几何体
 *
 * 将起点点云中的所有点通过位姿变换后渲染为蓝色点集。
 */
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

/**
 * @brief 重建终点云几何体
 *
 * 将终点点云中的所有点通过相对位姿变换后渲染为绿色点集。
 */
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

/**
 * @brief 重建叠加几何体（球体 + 坐标轴）
 *
 * 在起点和终点位置绘制球体标记（蓝色=起点，绿色=终点），
 * 并在终点位置绘制 XYZ 三色坐标轴（1.5倍缩放）。
 *
 * 球体通过环面（rings）× 扇区（sectors）网格细分实现。
 */
void MiniViewportWidget::rebuildOverlay(const Eigen::Isometry3d& relPose) {
    if (!m_sphereGeom || !m_axesGeom) return;

    // --- 球体（实体，通过环面+扇区细分） ---
    {
        const float pi = 3.14159265f;
        const float radius = 0.3f;
        const int rings = 12;    // 环数
        const int sectors = 12;  // 扇区数

        auto* verts = new osg::Vec3Array;
        auto* colors = new osg::Vec4Array;
        auto* indices = new osg::DrawElementsUInt(GL_TRIANGLES);

        osg::Vec4 blue(0.2f, 0.2f, 1.0f, 1.0f);
        osg::Vec4 green(0.2f, 1.0f, 0.2f, 1.0f);

        // 起点在原点的球心，终点在相对位姿平移处的球心
        osg::Vec3 centers[2] = {
            osg::Vec3(0, 0, 0),
            osg::Vec3(relPose.translation().x(),
                      relPose.translation().y(),
                      relPose.translation().z())
        };
        osg::Vec4 sphereColors[2] = { blue, green };

        // 为两个球体生成顶点和索引
        for (int si = 0; si < 2; ++si) {
            unsigned int base = verts->size();
            // 环方向循环（纬度）
            for (int r = 0; r <= rings; ++r) {
                float phi = float(r) * pi / float(rings);
                float sinPhi = std::sin(phi);
                float cosPhi = std::cos(phi);
                // 扇区方向循环（经度）
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
            // 三角面片索引
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

    // --- 终点位置的坐标轴（1.5× 缩放） ---
    {
        Eigen::Vector3d t = relPose.translation();
        Eigen::Matrix3d R = relPose.linear();
        double s = 1.5;

        auto* verts = new osg::Vec3Array;
        auto* colors = new osg::Vec4Array;

        osg::Vec3 origin(t.x(), t.y(), t.z());
        osg::Vec4 red(1, 0, 0, 1), grn(0, 1, 0, 1), blu(0, 0, 1, 1);

        // 取旋转矩阵的各列作为局部坐标轴方向
        Eigen::Vector3d axX = R.col(0) * s;
        Eigen::Vector3d axY = R.col(1) * s;
        Eigen::Vector3d axZ = R.col(2) * s;

        // X 轴（红色）
        verts->push_back(origin);
        verts->push_back(osg::Vec3(t.x() + axX.x(), t.y() + axX.y(), t.z() + axX.z()));
        colors->push_back(red); colors->push_back(red);

        // Y 轴（绿色）
        verts->push_back(origin);
        verts->push_back(osg::Vec3(t.x() + axY.x(), t.y() + axY.y(), t.z() + axY.z()));
        colors->push_back(grn); colors->push_back(grn);

        // Z 轴（蓝色）
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
