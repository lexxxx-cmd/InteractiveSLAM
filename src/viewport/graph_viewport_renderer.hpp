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
    QSize m_viewportSize;       // from synchronize (QML item size)
    QSize m_fboSize;            // from createFramebufferObject (actual FBO)
    Eigen::Matrix4f m_cameraView = [](){
        Eigen::Matrix4f v = Eigen::Matrix4f::Identity();
        v(2, 3) = -10.0f;
        return v;
    }();
    bool m_glInitialized = false;

    // FPS throttling
    std::chrono::steady_clock::time_point m_lastFpsEmitTime;
    float m_pendingFpsUpdate = 0.0f;
    bool m_hasFpsUpdate = false;
    int m_frameCount = 0;
};
