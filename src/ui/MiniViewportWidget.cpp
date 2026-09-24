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
 * 场景图结构（顶点一律"只建一次"，位姿交给变换节点）：
 *
 *   m_root
 *    ├─ m_beginGeode      起点云（蓝色，帧局部系 = 原点不动，无需变换）
 *    ├─ 起点球几何体      （原点标记，静态）
 *    └─ m_endTransform   ← updateEndPose() 只改这个矩阵
 *         ├─ m_endGeode          终点云（绿色，顶点=终点帧局部坐标）
 *         └─ m_endOverlayGeode   终点球 + 坐标轴（顶点已在终点帧里摆好）
 *
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

    // --- 起点位置的球体标记（静态，挂在根下即可） ---
    m_beginSphereGeom = new osg::Geometry;
    m_beginSphereGeom->setUseDisplayList(false);
    m_beginSphereGeom->setUseVertexBufferObjects(true);
    m_beginSphereGeom->setUseVertexArrayObject(true);
    m_beginSphereGeom->setDataVariance(osg::Object::STATIC);
    applySimpleColorShader(m_beginSphereGeom->getOrCreateStateSet());
    m_root->addChild(m_beginSphereGeom);

    // --- 终点侧变换节点：终点云与终点叠加层都挂在它下面 ---
    m_endTransform = new osg::MatrixTransform;
    m_endTransform->setMatrix(osg::Matrixd::identity());

    // 终点云几何节点（绿色）。顶点是**终点帧局部坐标**，位姿由父变换节点施加
    m_endGeode = new osg::Geode;
    m_endGeom = new osg::Geometry;
    m_endGeom->setUseDisplayList(false);
    m_endGeom->setUseVertexBufferObjects(true);
    m_endGeom->setUseVertexArrayObject(true);
    m_endGeom->setDataVariance(osg::Object::STATIC);   // 顶点不再随位姿重写
    applyPointCloudShader(m_endGeom->getOrCreateStateSet(), 2.0f);
    m_endGeode->addDrawable(m_endGeom);
    m_endTransform->addChild(m_endGeode);

    // --- 终点叠加几何节点（球体 + 坐标轴），同样在终点帧局部坐标下 ---
    m_endOverlayGeode = new osg::Geode;

    // 终点球几何体（GL_TRIANGLES，球心在局部原点，靠变换节点搬到终点）
    m_endSphereGeom = new osg::Geometry;
    m_endSphereGeom->setUseDisplayList(false);
    m_endSphereGeom->setUseVertexBufferObjects(true);
    m_endSphereGeom->setUseVertexArrayObject(true);
    m_endSphereGeom->setDataVariance(osg::Object::STATIC);
    applySimpleColorShader(m_endSphereGeom->getOrCreateStateSet());
    m_endOverlayGeode->addDrawable(m_endSphereGeom);

    // 坐标轴几何体（GL_LINES，顶点在终点帧里）
    m_axesGeom = new osg::Geometry;
    m_axesGeom->setUseDisplayList(false);
    m_axesGeom->setUseVertexBufferObjects(true);
    m_axesGeom->setUseVertexArrayObject(true);
    m_axesGeom->setDataVariance(osg::Object::STATIC);
    {
        auto* ss = m_axesGeom->getOrCreateStateSet();
        applySimpleColorShader(ss);
        ss->setAttributeAndModes(new osg::LineWidth(2.0f),
                                 osg::StateAttribute::ON);
    }
    m_endOverlayGeode->addDrawable(m_axesGeom);

    m_endTransform->addChild(m_endOverlayGeode);
    m_root->addChild(m_endTransform);
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
 * **一次性**建好两片云的顶点（起点用原点、终点用帧局部坐标）并把 rel 写进
 * 终点变换节点，然后使摄像机定格居中。
 *
 * 这里是唯一一次 O(N) 构建：之后每次微调只改 m_endTransform 的矩阵。
 */
void MiniViewportWidget::setClouds(CloudPtr beginCloud,
                                    const Eigen::Isometry3d& beginPose,
                                    CloudPtr endCloud,
                                    const Eigen::Isometry3d& endPose) {
    m_beginCloud = beginCloud;
    m_endCloud   = endCloud;

    // 计算相对位姿：终点在起点坐标系下的位姿
    Eigen::Isometry3d rel = beginPose.inverse() * endPose;

    // 顶点只与"帧局部坐标"有关，与位姿无关：这里建一次，位姿写进变换节点。
    rebuildBeginGeometry();  // 起点云 + 原点处的起点球
    rebuildEndGeometry();    // 终点云 + 终点球 + 坐标轴（都在终点帧局部系里）
    // 4x4 矩阵逐元素拷贝：Eigen 默认列主序、osg::Matrix 也是列主序，
    // 因此 data() 的 16 个 double 可以直接喂给 osg::Matrixd（行主序的 Eigen
    // 矩阵会在这里因 Eigen 自身的 static_assert 编译失败，不会静默转置错）
    const Eigen::Matrix4d relMat = rel.matrix();
    m_endTransform->setMatrix(osg::Matrixd(relMat.data()));

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
 * 只把相对位姿写进变换节点：O(1)，与点云规模无关（旧实现是逐点重建
 * 终点云顶点数组 + 颜色数组，19.5 万点即每次 5.2 MB 分配与一次 VBO 重传）。
 *
 * 三个 dirtyBound() 的必要性：dirtyBound() 只在"本节点包围球已算过"时才向上
 * 传播。刚 setClouds() 之后还没渲染过一帧就点击按钮的话，几何体的包围球尚未
 * 计算，单靠它传播不到根节点，viewer->home()/Fit View 就会用旧包围盒取景。
 * 因此这里逐层清缓存，保证下一次 getBound() 重新沿变换节点算世界包围球。
 *
 * @param endPose   新的终点世界位姿
 * @param beginPose 起点世界位姿
 */
void MiniViewportWidget::updateEndPose(const Eigen::Isometry3d& endPose,
                                        const Eigen::Isometry3d& beginPose) {
    const Eigen::Isometry3d rel = beginPose.inverse() * endPose;
    // 同 setClouds()：列主序逐元素拷贝，见那里的说明
    const Eigen::Matrix4d relMat = rel.matrix();
    m_endTransform->setMatrix(osg::Matrixd(relMat.data()));
    m_endGeom->dirtyBound();
    m_endTransform->dirtyBound();
    m_root->dirtyBound();
    m_osgWidget->update();
}

void MiniViewportWidget::resetCamera() {
    osgViewer::Viewer* viewer = m_osgWidget->getOsgViewer();
    if (viewer) {
        viewer->home();
    }
}

// ---------------------------------------------------------------------------
// 内部：构建几何体（每个只在 setClouds() 里做一次）
// ---------------------------------------------------------------------------

/**
 * @brief 构建起点侧的几何体：起点云（蓝色）+ 原点处的起点球
 *
 * 起点云在本控件的约定里就是"帧局部坐标 = 世界原点"，因此顶点原样写入，
 * 不做任何位姿变换（与旧实现 pose=Identity 的行为逐位一致）。
 *
 * 颜色数组只放 1 个元素并 BIND_OVERALL：旧实现是给每个点 push 同一份颜色，
 * 二者渲染结果相同（片元拿到的都是同一个颜色），但省掉每点 16 字节 ——
 * 19.5 万点即省 3.1 MB。
 */
void MiniViewportWidget::rebuildBeginGeometry() {
    if (!m_beginCloud || m_beginCloud->empty() || !m_beginGeom) return;

    auto* verts = new osg::Vec3Array;
    verts->reserve(m_beginCloud->size());
    for (const auto& pt : m_beginCloud->points) {
        verts->push_back(osg::Vec3(pt.x, pt.y, pt.z));
    }

    auto* colors = new osg::Vec4Array;
    colors->push_back(osg::Vec4(0.0f, 0.0f, 1.0f, 1.0f));   // 起点云：纯蓝

    m_beginGeom->setVertexArray(verts);
    m_beginGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
    m_beginGeom->removePrimitiveSet(0, m_beginGeom->getNumPrimitiveSets());
    m_beginGeom->addPrimitiveSet(
        new osg::DrawArrays(GL_POINTS, 0, verts->size()));
    m_beginGeom->dirtyBound();

    // 起点位置的球体标记（球心在原点，颜色与旧叠加层一致）
    buildUnitSphereAtOrigin(m_beginSphereGeom,
                            osg::Vec4(0.2f, 0.2f, 1.0f, 1.0f));
}

/**
 * @brief 构建终点侧的几何体：终点云（绿色）+ 终点球 + 坐标轴
 *
 * 全部按**终点帧局部坐标**构建——也就是旧实现里把 relPose 逐点乘进去之前
 * 的那份坐标。位姿改由父节点 m_endTransform 施加，渲染结果与旧实现逐点等价
 * （旧实现算的是 relPose * p，现在算的是 MVP · T_rel · p，T_rel 就是 relPose）。
 */
void MiniViewportWidget::rebuildEndGeometry() {
    // --- 终点云（绿色） ---
    if (m_endCloud && !m_endCloud->empty() && m_endGeom) {
        auto* verts = new osg::Vec3Array;
        verts->reserve(m_endCloud->size());
        for (const auto& pt : m_endCloud->points) {
            verts->push_back(osg::Vec3(pt.x, pt.y, pt.z));
        }

        auto* colors = new osg::Vec4Array;
        colors->push_back(osg::Vec4(0.0f, 1.0f, 0.0f, 1.0f));  // 终点云：纯绿

        m_endGeom->setVertexArray(verts);
        m_endGeom->setColorArray(colors, osg::Array::BIND_OVERALL);
        m_endGeom->removePrimitiveSet(0, m_endGeom->getNumPrimitiveSets());
        m_endGeom->addPrimitiveSet(
            new osg::DrawArrays(GL_POINTS, 0, verts->size()));
        m_endGeom->dirtyBound();
    }

    // --- 终点球（球心在局部原点，随 m_endTransform 平移到终点） ---
    buildUnitSphereAtOrigin(m_endSphereGeom,
                            osg::Vec4(0.2f, 1.0f, 0.2f, 1.0f));

    // --- 坐标轴（终点帧局部坐标：原点起、沿局部 XYZ 各 1.5 单位） ---
    if (m_axesGeom) {
        const double s = 1.5;

        auto* verts = new osg::Vec3Array;
        auto* colors = new osg::Vec4Array;

        const osg::Vec3 origin(0.0f, 0.0f, 0.0f);
        const osg::Vec4 red(1, 0, 0, 1), grn(0, 1, 0, 1), blu(0, 0, 1, 1);

        // X 轴（红色）
        verts->push_back(origin);
        verts->push_back(osg::Vec3(s, 0.0f, 0.0f));
        colors->push_back(red); colors->push_back(red);

        // Y 轴（绿色）
        verts->push_back(origin);
        verts->push_back(osg::Vec3(0.0f, s, 0.0f));
        colors->push_back(grn); colors->push_back(grn);

        // Z 轴（蓝色）
        verts->push_back(origin);
        verts->push_back(osg::Vec3(0.0f, 0.0f, s));
        colors->push_back(blu); colors->push_back(blu);

        m_axesGeom->setVertexArray(verts);
        m_axesGeom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
        m_axesGeom->removePrimitiveSet(0, m_axesGeom->getNumPrimitiveSets());
        m_axesGeom->addPrimitiveSet(
            new osg::DrawArrays(GL_LINES, 0, verts->size()));
        m_axesGeom->dirtyBound();
    }
}

/**
 * @brief 在给定几何体上画一个"球心在局部原点"的球（半径 0.3，12×12 细分）
 *
 * 顶点只与球心在原点有关，所以把它抽出来：起点球直接挂根节点（原点不动），
 * 终点球挂 m_endTransform（位姿变化只是换个矩阵，重新三角化 11 万条索引
 * 完全没必要——那是旧实现每次点击都在做的事）。
 *
 * @param geom  目标几何体（着色器/状态集需已设置）
 * @param color 球体颜色（旧实现里起点球 (0.2,0.2,1)、终点球 (0.2,1,0.2)）
 */
void MiniViewportWidget::buildUnitSphereAtOrigin(osg::Geometry* geom,
                                                 const osg::Vec4& color) {
    if (!geom) return;

    const float pi = 3.14159265f;
    const float radius = 0.3f;
    const int rings = 12;    // 环数
    const int sectors = 12;  // 扇区数

    auto* verts = new osg::Vec3Array;
    auto* colors = new osg::Vec4Array;
    auto* indices = new osg::DrawElementsUInt(GL_TRIANGLES);

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
            verts->push_back(osg::Vec3(radius * sinPhi * cosT,
                                       radius * cosPhi,
                                       radius * sinPhi * sinT));
            colors->push_back(color);
        }
    }
    // 三角面片索引
    for (int r = 0; r < rings; ++r) {
        for (int s = 0; s < sectors; ++s) {
            unsigned int a = r * (sectors + 1) + s;
            unsigned int b = a + sectors + 1;
            indices->push_back(a);
            indices->push_back(b);
            indices->push_back(a + 1);
            indices->push_back(b);
            indices->push_back(b + 1);
            indices->push_back(a + 1);
        }
    }

    geom->setVertexArray(verts);
    geom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
    geom->removePrimitiveSet(0, geom->getNumPrimitiveSets());
    geom->addPrimitiveSet(indices);
    geom->dirtyBound();
}

