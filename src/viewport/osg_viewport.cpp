#include "viewport/osg_viewport.hpp"
#include "viewport/osg_viewport_renderer.hpp"

#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>

OSGViewport::OSGViewport(QQuickItem* parent)
    : QQuickFramebufferObject(parent)
{
    setAcceptedMouseButtons(Qt::LeftButton | Qt::MiddleButton | Qt::RightButton);
    setFlag(ItemAcceptsInputMethod, true);
}

void OSGViewport::receiveFpsUpdate(float fps)
{
    m_currentFps = fps;
    emit fpsUpdated(fps);
}

void OSGViewport::enqueue(const CloneEvent& e)
{
    QMutexLocker lock(&m_queueMutex);
    m_eventQueue.push_back(e);
}

void OSGViewport::resetCamera()
{
    m_cameraResetRequested = true;
    update();   // trigger synchronize → renderer reads the flag
}

QQuickFramebufferObject::Renderer* OSGViewport::createRenderer() const
{
    return new OSGViewportRenderer();
}

// ---- Input overrides: clone the event into the queue (GUI thread) ----
void OSGViewport::mousePressEvent(QMouseEvent* e)
{
    CloneEvent c{CloneEvent::MousePress, e->position(), e->buttons(), e->button(), {}, 0, e->modifiers()};
    enqueue(c); update();
}
void OSGViewport::mouseReleaseEvent(QMouseEvent* e)
{
    CloneEvent c{CloneEvent::MouseRelease, e->position(), e->buttons(), e->button(), {}, 0, e->modifiers()};
    enqueue(c); update();
}
void OSGViewport::mouseMoveEvent(QMouseEvent* e)
{
    CloneEvent c{CloneEvent::MouseMove, e->position(), e->buttons(), e->button(), {}, 0, e->modifiers()};
    enqueue(c); update();
}
void OSGViewport::mouseDoubleClickEvent(QMouseEvent* e)
{
    CloneEvent c{CloneEvent::MouseDoubleClick, e->position(), e->buttons(), e->button(), {}, 0, e->modifiers()};
    enqueue(c); update();
}
void OSGViewport::wheelEvent(QWheelEvent* e)
{
    CloneEvent c{CloneEvent::Wheel, e->position(), e->buttons(), Qt::NoButton, e->angleDelta(), 0, e->modifiers()};
    enqueue(c); update();
}
void OSGViewport::keyPressEvent(QKeyEvent* e)
{
    CloneEvent c{CloneEvent::KeyPress, {}, Qt::NoButton, Qt::NoButton, {}, e->key(), e->modifiers()};
    enqueue(c); update();
}
void OSGViewport::keyReleaseEvent(QKeyEvent* e)
{
    CloneEvent c{CloneEvent::KeyRelease, {}, Qt::NoButton, Qt::NoButton, {}, e->key(), e->modifiers()};
    enqueue(c); update();
}
