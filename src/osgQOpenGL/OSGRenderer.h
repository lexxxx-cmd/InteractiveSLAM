#ifndef OSGRENDERER_H
#define OSGRENDERER_H

#include <QObject>
#include <functional>

#include <osg/ArgumentParser>
#include <osgViewer/Viewer>

class QInputEvent;
class QKeyEvent;
class QMouseEvent;
class QWheelEvent;

// OSGRenderer wraps an osgViewer::Viewer and adapts Qt input events into OSG's
// event queue. Originally coupled to osgQOpenGLWidget/osgQOpenGLWindow; it is now
// widget-agnostic so it can also drive rendering from a QQuickFramebufferObject
// (QML). When used inside a Qt FBO, call setDriveExternally(true) to disable the
// internal QTimer, and install a callback via setUpdateCallback() so the host
// can trigger Qt scenegraph updates on demand.
class OSGRenderer : public QObject, public osgViewer::Viewer
{
    Q_OBJECT

    bool                                       m_osgInitialized {false};
    osg::ref_ptr<osgViewer::GraphicsWindow>    m_osgWinEmb;
    float                                      m_windowScale {1.0f};
    bool                                       m_continuousUpdate {true};

    int                                        _timerId{0};
    osg::Timer                                 _lastFrameStartTime;
    bool                                       _applicationAboutToQuit {false};
    bool                                       _osgWantsToRenderFrame{true};

    // FBO path: the host installs a callback to request a Qt scenegraph update
    // (replaces the former dynamic_cast<osgQOpenGLWidget> coupling).
    std::function<void()>                      m_requestUpdate;

    // When true, the host drives frame timing and the internal QTimer is off.
    bool                                       m_driveExternally {false};

public:

    explicit OSGRenderer(QObject* parent = nullptr);
    explicit OSGRenderer(osg::ArgumentParser* arguments, QObject* parent = nullptr);

    ~OSGRenderer() override;

    bool continuousUpdate() const { return m_continuousUpdate; }
    void setContinuousUpdate(bool continuousUpdate) { m_continuousUpdate = continuousUpdate; }

    //! Disable the internal QTimer — the host will trigger updates instead.
    void setDriveExternally(bool v) { m_driveExternally = v; }

    //! Install a host callback used to request a Qt scenegraph update.
    void setUpdateCallback(std::function<void()> cb) { m_requestUpdate = std::move(cb); }

    //! Forward Qt input events into the OSG event queue (thread-safe queue).
    virtual void keyPressEvent(QKeyEvent* event);
    virtual void keyReleaseEvent(QKeyEvent* event);
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseDoubleClickEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void wheelEvent(QWheelEvent* event);

    virtual void resize(int windowWidth, int windowHeight, float windowScale);

    void setupOSG(int windowWidth, int windowHeight, float windowScale);

    //! Direct subsequent OSG rendering at a Qt-managed framebuffer object.
    void setDefaultFboId(unsigned int fboId);

    // overrided from osgViewer::Viewer
    virtual bool checkNeedToDoFrame() override;

    // overrided from osgViewer::ViewerBase
    void frame(double simulationTime = USE_REFERENCE_TIME) override;

    // overrided from osgViewer::Viewer
    void requestRedraw() override;
    // overrided from osgViewer::Viewer
    bool checkEvents() override;
    void update();

protected:
    void timerEvent(QTimerEvent* event) override;

    void setKeyboardModifiers(QInputEvent* event);

};

#endif // OSGRENDERER_H
