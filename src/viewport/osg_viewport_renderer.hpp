#pragma once

#include <QQuickFramebufferObject>
#include <QOpenGLFramebufferObject>

#include <array>
#include <chrono>
#include <unordered_map>
#include <memory>
#include <osg/ref_ptr>
#include <osg/Group>
#include <osg/MatrixTransform>
#include <osgGA/CameraManipulator>

#include "viewport/drawable_object.hpp"

namespace osg  { class Group; }
namespace osgGA{ class CameraManipulator; }
namespace hdl_graph_slam { class InteractiveGraph; }
class OSGRenderer;

class OSGViewport;
class QMouseEvent;
class QWheelEvent;
class QKeyEvent;

// OSGViewportRenderer — bridges OSG into a QQuickFramebufferObject.
//
// Design notes
//  - The QQuickFramebufferObject::Renderer runs on the Qt render thread. Its
//    render() is called whenever the QML item calls update().
//  - Before render() Qt has bound our framebuffer object. We hand its id to OSG
//    via OSGRenderer::setDefaultFboId() so OSG renders straight into it instead
//    of allocating its own FBO. (Same trick the original QOpenGLWidget path used
//    with QOpenGLWidget::defaultFramebufferObject().)
//  - Continual rendering: synchronize() ends with item->update(), mirroring the
//    existing GraphViewportRenderer loop (~60 fps), which the OSG manipulator
//    needs for smooth interaction.
class OSGViewportRenderer : public QQuickFramebufferObject::Renderer
{
public:
    OSGViewportRenderer();
    ~OSGViewportRenderer() override;

    void render() override;
    void synchronize(QQuickFramebufferObject* item) override;
    QOpenGLFramebufferObject* createFramebufferObject(const QSize& size) override;

    //! Drain queued input events (cloned on the GUI thread) into OSG's queue.
    void drainInput(OSGViewport* viewport);

private:
    void buildDefaultScene();

    // Rebuild all graph-related OSG geometry (called on graph load/change).
    void rebuildGraphScene();

    // Apply current DrawFlags as node-mask visibility toggles.
    void updateGraphVisibility();

    OSGRenderer*                                 m_osg {nullptr};
    osg::ref_ptr<osg::Group>                     m_scene;
    osg::ref_ptr<osgGA::CameraManipulator>       m_manipulator;

    // Graph data snapshot (render thread copy, updated in synchronize)
    std::shared_ptr<hdl_graph_slam::InteractiveGraph> m_renderGraph;
    const void*                                       m_lastGraphPtr = nullptr;
    hdl_graph_slam::DrawFlags                         m_drawFlags;
    bool                                              m_drawFlagsDirty = true;

    // OSG nodes for graph content (children of m_scene)
    osg::ref_ptr<osg::Group> m_graphContentGroup;
    osg::ref_ptr<osg::Group> m_keyframeGroup;
    osg::ref_ptr<osg::Group> m_edgeGroup;

    // Track keyframe transforms for incremental pose updates
    std::unordered_map<long, osg::ref_ptr<osg::MatrixTransform>> m_keyframeTransforms;

    bool      m_callbackInstalled    = false;

    QSize     m_fboSize;

    // ---- FPS counter (rolling average, matches existing renderer) ----
    static constexpr int FPS_WINDOW = 60;
    std::chrono::steady_clock::time_point m_lastFrameTime;
    std::chrono::steady_clock::time_point m_lastFpsEmitTime;
    std::array<float, FPS_WINDOW> m_frameTimes{};
    int   m_frameIndex = 0;
    int   m_fpsSampleCount = 0;
    float m_frameTimeSum = 0.0f;
    float m_pendingFpsUpdate = 0.0f;
    bool  m_hasFpsUpdate = false;
    int   m_frameCount = 0;

    // back-pointer to the item, for update() requests from the OSG callback
    QQuickFramebufferObject* m_item = nullptr;
};
