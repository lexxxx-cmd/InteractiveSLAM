#include "viewport/osg_viewport_renderer.hpp"
#include "viewport/osg_viewport.hpp"

#include "OSGRenderer.h"

#include <osg/Camera>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/LineWidth>
#include <osg/Vec3>
#include <osg/Vec4>
#include <osg/MatrixTransform>
#include <osgViewer/Viewer>

#include <osgGA/TrackballManipulator>

#include <QOpenGLFunctions>
#include <QOpenGLFramebufferObject>
#include <QQuickWindow>
#include <QDebug>

#include <cmath>

// ---- Default scene: an RGB axis triad (X red / Y green / Z blue) ----
// Cheap, instantly visible proof that the OSG pipeline is alive inside QML.
namespace {

osg::Geometry* makeAxis(float x, float y, float z, float r, float g, float b)
{
    auto* geom = new osg::Geometry;

    osg::Vec3Array* v = new osg::Vec3Array;
    v->push_back(osg::Vec3(0, 0, 0));
    v->push_back(osg::Vec3(x, y, z));
    geom->setVertexArray(v);

    osg::Vec4Array* c = new osg::Vec4Array;
    c->push_back(osg::Vec4(r, g, b, 1.0f));
    geom->setColorArray(c, osg::Array::BIND_OVERALL);

    geom->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::LINES, 0, 2));
    geom->getOrCreateStateSet()->setMode(GL_LIGHTING, osg::StateAttribute::OFF);
    geom->getOrCreateStateSet()->setAttribute(new osg::LineWidth(3.0f));
    return geom;
}

osg::Group* buildAxesScene()
{
    auto* root = new osg::Group;
    auto* geode = new osg::Geode;
    geode->addDrawable(makeAxis(3.0f, 0,    0,    1.0f, 0.2f, 0.2f));   // X red
    geode->addDrawable(makeAxis(0,    3.0f, 0,    0.2f, 1.0f, 0.2f));   // Y green
    geode->addDrawable(makeAxis(0,    0,    3.0f, 0.2f, 0.4f, 1.0f));   // Z blue
    root->addChild(geode);
    return root;
}

} // namespace

OSGViewportRenderer::OSGViewportRenderer()
{
    m_lastFrameTime = std::chrono::steady_clock::now();
    m_lastFpsEmitTime = m_lastFrameTime;
}

OSGViewportRenderer::~OSGViewportRenderer()
{
    // OSGRenderer is a QObject whose parent is this renderer's owner (the viewer
    // keeps refs to the scene graph). Delete it here so the timer / GL resources
    // go away before the Qt GL context is torn down.
    if (m_osg) {
        delete m_osg;
        m_osg = nullptr;
    }
}

void OSGViewportRenderer::buildDefaultScene()
{
    m_scene = buildAxesScene();
    m_osg->setSceneData(m_scene);

    // osgGA manipulator gives us free orbit/pan/zoom. Trackball is the most
    // forgiving default for a viewer app.
    auto* tb = new osgGA::TrackballManipulator;
    tb->setHomePosition(/*eye*/ osg::Vec3(8, -8, 6),
                        /*center*/ osg::Vec3(0, 0, 0),
                        /*up*/ osg::Vec3(0, 0, 1),
                        /*autoComputeHomePosition*/ false);
    m_manipulator = tb;
    m_osg->setCameraManipulator(m_manipulator);
    m_osg->home();   // apply the home view
}

QOpenGLFramebufferObject*
OSGViewportRenderer::createFramebufferObject(const QSize& size)
{
    // OSG needs a depth buffer. CombinedDepthStencil matches the existing
    // GraphViewportRenderer FBO format.
    QOpenGLFramebufferObjectFormat format;
    format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
    format.setSamples(0);
    m_fboSize = size;
    return new QOpenGLFramebufferObject(size, format);
}

void OSGViewportRenderer::synchronize(QQuickFramebufferObject* item)
{
    m_item = item;

    if (!m_osg) {
        // First sync: create the OSG viewer and wire it up.
        m_osg = new OSGRenderer();           // parent stays null; we own it
        m_osg->setDriveExternally(true);     // we drive frame timing via render()
        m_osg->setContinuousUpdate(true);

        // Let OSG ask us (indirectly) for updates by pinging the QML item.
        // QQuickItem::update() must be called on the GUI thread; we bounce
        // through QMetaObject::invokeMethod Queued to be safe, though in practice
        // synchronize() runs shortly before render() on the render thread.
        QQuickFramebufferObject* itemPtr = item;
        m_osg->setUpdateCallback([itemPtr]() {
            QMetaObject::invokeMethod(itemPtr, [itemPtr](){ itemPtr->update(); },
                                      Qt::QueuedConnection);
        });

        m_osg->setupOSG(item->width(), item->height(), item->window()->devicePixelRatio());

        // Viewer defaults to ON_DEMAND; we want every frame for smooth input.
        m_osg->setRunFrameScheme(osgViewer::ViewerBase::CONTINUOUS);

        buildDefaultScene();
    }

    // Keep OSG's viewport in sync with the QML item size.
    if (item->width() > 0 && item->height() > 0) {
        m_osg->resize(static_cast<int>(item->width()),
                      static_cast<int>(item->height()),
                      item->window()->devicePixelRatio());
    }

    // Drain events cloned on the main thread into OSG's event queue.
    // Must happen before OSG frame() so the manipulator sees fresh input.
    drainInput(static_cast<OSGViewport*>(item));

    // Camera reset request from QML (resetCamera()). The flag lives on the
    // viewport (main thread), read here on the render thread.
    if (auto* v = static_cast<OSGViewport*>(item)) {
        if (v->m_cameraResetRequested) {
            v->m_cameraResetRequested = false;
            if (m_manipulator.valid())
                m_manipulator->home(0.0);   // snap home instantly
        }
    }

    // Deliver any FPS sample computed in render() back to the item.
    if (m_hasFpsUpdate) {
        if (auto* v = static_cast<OSGViewport*>(item))
            v->receiveFpsUpdate(m_pendingFpsUpdate);
        m_hasFpsUpdate = false;
    }

    // Continuous rendering: ask for the next frame, just like GraphViewportRenderer.
    item->update();
}

void OSGViewportRenderer::render()
{
    if (!m_osg) return;

    // Qt has bound our FBO. Tell OSG to render into that same framebuffer so it
    // composites into the QML scene instead of its own (offscreen) buffer.
    if (auto* ctx = QOpenGLContext::currentContext())
        m_osg->setDefaultFboId(static_cast<unsigned int>(ctx->defaultFramebufferObject()));

    m_osg->frame();

    // ---- FPS bookkeeping (rolling 60-frame average) ----
    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - m_lastFrameTime).count();
    m_lastFrameTime = now;
    if (dt > 0.0f) {
        m_frameTimeSum -= m_frameTimes[m_frameIndex];
        m_frameTimes[m_frameIndex] = dt;
        m_frameTimeSum += dt;
        m_frameIndex = (m_frameIndex + 1) % FPS_WINDOW;
        if (m_fpsSampleCount < FPS_WINDOW) ++m_fpsSampleCount;
    }
    ++m_frameCount;
    if (now - m_lastFpsEmitTime > std::chrono::milliseconds(500)) {
        float avg = m_frameTimeSum / std::max(1, m_fpsSampleCount);
        m_pendingFpsUpdate = avg > 0.0f ? 1.0f / avg : 0.0f;
        m_hasFpsUpdate = true;
        m_lastFpsEmitTime = now;
    }

    // Restore Qt's expectations: the QQuickFramebufferObject pipeline expects the
    // renderer to leave depth test/writing in a known state. OSG enables depth;
    // that's fine, but disable scissor to avoid clipping Qt's later compositing.
    QOpenGLFunctions f(QOpenGLContext::currentContext());
    f.initializeOpenGLFunctions();
    f.glDisable(GL_SCISSOR_TEST);
}

// ---- Input forwarding ----
// Events were cloned on the GUI thread into OSGViewport's queue. Here (render
// thread, during synchronize) we synthesise real Q*Event objects from the
// clones and feed them to OSGRenderer, which pushes them onto OSG's event queue.
void OSGViewportRenderer::drainInput(OSGViewport* viewport)
{
    if (!m_osg || !viewport) return;

    std::vector<OSGViewport::CloneEvent> events;
    {
        QMutexLocker lock(&viewport->m_queueMutex);
        events.swap(viewport->m_eventQueue);
    }

    for (const auto& c : events) {
        switch (c.type) {
        case OSGViewport::CloneEvent::MousePress:
        case OSGViewport::CloneEvent::MouseRelease:
        case OSGViewport::CloneEvent::MouseMove:
        case OSGViewport::CloneEvent::MouseDoubleClick: {
            QEvent::Type t = QEvent::MouseMove;
            if (c.type == OSGViewport::CloneEvent::MousePress)            t = QEvent::MouseButtonPress;
            else if (c.type == OSGViewport::CloneEvent::MouseRelease)     t = QEvent::MouseButtonRelease;
            else if (c.type == OSGViewport::CloneEvent::MouseDoubleClick) t = QEvent::MouseButtonDblClick;
            QMouseEvent me(t, c.pos, c.pos, c.button, c.buttons, c.modifiers);
            if      (c.type == OSGViewport::CloneEvent::MousePress)          m_osg->mousePressEvent(&me);
            else if (c.type == OSGViewport::CloneEvent::MouseRelease)        m_osg->mouseReleaseEvent(&me);
            else if (c.type == OSGViewport::CloneEvent::MouseMove)           m_osg->mouseMoveEvent(&me);
            else if (c.type == OSGViewport::CloneEvent::MouseDoubleClick)    m_osg->mouseDoubleClickEvent(&me);
            break;
        }
        case OSGViewport::CloneEvent::Wheel: {
            // QWheelEvent has a private ctor signature in Qt6; the portable way
            // to build one is via the QPointF/angleDelta form.
            QWheelEvent we(c.pos, c.pos, QPoint(), c.angleDelta, c.buttons, c.modifiers,
                           Qt::NoScrollPhase, false);
            m_osg->wheelEvent(&we);
            break;
        }
        case OSGViewport::CloneEvent::KeyPress:
        case OSGViewport::CloneEvent::KeyRelease: {
            QKeyEvent ke(c.type == OSGViewport::CloneEvent::KeyPress
                         ? QEvent::KeyPress : QEvent::KeyRelease,
                         c.key, c.modifiers);
            if (c.type == OSGViewport::CloneEvent::KeyPress)
                m_osg->keyPressEvent(&ke);
            else
                m_osg->keyReleaseEvent(&ke);
            break;
        }
        }
    }
}
