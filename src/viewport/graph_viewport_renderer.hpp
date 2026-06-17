#pragma once

#include <QQuickFramebufferObject>
#include <QOpenGLFramebufferObject>
#include <QOpenGLExtraFunctions>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <deque>
#include <Eigen/Core>
#include "glk/glsl_shader.hpp"
#include "glk/mesh.hpp"
#include "viewport/drawable_object.hpp"
#include "viewport/keyframe_view.hpp"
#include "viewport/vertex_view.hpp"
#include "viewport/edge_view.hpp"
#include "viewport/line_buffer.hpp"
#include "data/hdl_graph_slam/interactive_graph.hpp"

class GraphViewportRenderer : public QQuickFramebufferObject::Renderer {
public:
    GraphViewportRenderer();
    ~GraphViewportRenderer() override;

    void render() override;
    void synchronize(QQuickFramebufferObject* item) override;
    QOpenGLFramebufferObject* createFramebufferObject(const QSize& size) override;

private:
    void initGL();
    void setupShaders();
    void cleanupGL();
    void syncGraphData();

    // Render thread's own shared_ptr — prevents UAF when main thread replaces graph
    std::shared_ptr<hdl_graph_slam::InteractiveGraph> m_renderGraph;
    int m_lastGraphVersion = -1;

    // Drawable objects (lazy-created from graph data)
    std::vector<hdl_graph_slam::DrawableObject::Ptr> m_drawables;
    std::unordered_map<long, hdl_graph_slam::KeyFrameView::Ptr> m_keyframeViews;
    std::unordered_map<long, hdl_graph_slam::VertexView::Ptr> m_vertexViews;
    std::unordered_map<long, hdl_graph_slam::EdgeView::Ptr> m_edgeViews;

    // Edge line batch
    std::unique_ptr<hdl_graph_slam::LineBuffer> m_lineBuffer;

    // Batched VBO upload state
    std::deque<long> m_pendingUploads;
    std::unordered_set<long> m_uploadedKeyframeIds;

    hdl_graph_slam::DrawFlags m_drawFlags;

    QOpenGLExtraFunctions* m_gl = nullptr;
    std::unique_ptr<glk::GLSLShader> m_rainbowShader;
    QSize m_viewportSize;       // from synchronize (QML item size, logical px)
    QSize m_fboSize;            // from createFramebufferObject (actual FBO, device px)
    Eigen::Matrix4f m_cameraView = [](){
        Eigen::Matrix4f v = Eigen::Matrix4f::Identity();
        v(2, 3) = -10.0f;
        return v;
    }();
    Eigen::Matrix4f m_projectionMatrix = Eigen::Matrix4f::Identity();  // stored for doPickReadback
    bool m_glInitialized = false;

    // ---- GPU color-coded picking ----
    // Single-frame consistent result: ID, depth, and worldPos are all from the
    // same render() invocation, unprojected with m_projectionMatrix × m_cameraView
    // of that frame.  No cross-frame matrix mixing.
    struct PickResult {
        bool hit = false;
        int  vertexId = -1;
        Eigen::Vector3f worldPos{0, 0, 0};
    };

    bool m_infoAttachmentAdded = false;

    // Request from main thread (via synchronize)
    bool  m_pickRequested = false;
    float m_pickLogicalX = 0;   // QML logical pixels
    float m_pickLogicalY = 0;
    float m_devicePixelRatio = 1.0f;

    // Result back to main thread (via synchronize)
    bool      m_pickResultReady = false;
    PickResult m_pickResult;

    void doPickReadback(float logicalX, float logicalY, float dpr);

    // FPS throttling
    std::chrono::steady_clock::time_point m_lastFpsEmitTime;
    float m_pendingFpsUpdate = 0.0f;
    bool m_hasFpsUpdate = false;
    int m_frameCount = 0;
};
