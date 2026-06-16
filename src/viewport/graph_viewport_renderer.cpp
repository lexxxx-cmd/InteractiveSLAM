#include "viewport/graph_viewport_renderer.hpp"
#include "viewport/graph_viewport.hpp"
#include "glk/primitives.hpp"
#include <QOpenGLContext>
#include <QDebug>
#include <cmath>
#include <algorithm>
#include <chrono>

// ROS (Z-up) → OpenGL (Y-up) coordinate transform
inline Eigen::Matrix4f rosToRender() {
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    m.block<3,3>(0,0) = Eigen::AngleAxisf(-3.14159265f / 2.0f,
                                           Eigen::Vector3f::UnitX()).matrix();
    return m;
}

GraphViewportRenderer::GraphViewportRenderer() = default;

GraphViewportRenderer::~GraphViewportRenderer() { cleanupGL(); }

void GraphViewportRenderer::cleanupGL() {
    if (!m_gl) return;
    m_drawables.clear();
    m_keyframeViews.clear();
    m_vertexViews.clear();
    m_pendingUploads.clear();
    m_uploadedKeyframeIds.clear();
    m_rainbowShader.reset();
    m_gl = nullptr;
    m_glInitialized = false;
}

QOpenGLFramebufferObject* GraphViewportRenderer::createFramebufferObject(const QSize& size) {
    QOpenGLFramebufferObjectFormat format;
    format.setAttachment(QOpenGLFramebufferObject::CombinedDepthStencil);
    format.setSamples(0);
    m_viewportSize = size;
    return new QOpenGLFramebufferObject(size, format);
}

void GraphViewportRenderer::synchronize(QQuickFramebufferObject* item) {
    m_viewportSize = item->size().toSize();
    auto* viewport = static_cast<GraphViewport*>(item);
    auto* manager = viewport->graphManager();

    // Get render thread's own shared_ptr copy (UAF protection)
    if (manager && manager->isLoaded()) {
        m_renderGraph = manager->sharedGraph();
    } else {
        m_renderGraph.reset();
    }

    // FPS throttled pass-back (every 0.5s)
    if (m_hasFpsUpdate) {
        viewport->receiveFpsUpdate(m_pendingFpsUpdate);
        m_hasFpsUpdate = false;
    }

    item->update();
}

void GraphViewportRenderer::initGL() {
    m_gl = glk::gl();
    if (!m_gl) qFatal("GraphViewportRenderer: OpenGL 3.0 not available");
    qDebug() << "GraphViewportRenderer: OpenGL version"
             << (const char*)m_gl->glGetString(GL_VERSION);
    m_glInitialized = true;
}

void GraphViewportRenderer::setupShaders() {
    m_rainbowShader = std::make_unique<glk::GLSLShader>();
    if (!m_rainbowShader->init(QStringLiteral(":/rainbow.vert"),
                                QStringLiteral(":/rainbow.frag")))
        qFatal("GraphViewportRenderer: failed to load rainbow shader");
}

void GraphViewportRenderer::syncGraphData() {
    auto graph = m_renderGraph;
    if (!graph) {
        if (!m_drawables.empty()) {
            m_drawables.clear();
            m_keyframeViews.clear();
            m_vertexViews.clear();
        }
        m_pendingUploads.clear();
        m_uploadedKeyframeIds.clear();
        return;
    }

    // Batched VBO upload — max 5 KeyFrameViews per frame
    static constexpr int UPLOAD_QUOTA = 5;

    if (m_pendingUploads.empty()) {
        for (auto& [id, kf] : graph->keyframes) {
            if (m_keyframeViews.find(id) == m_keyframeViews.end()
                && m_uploadedKeyframeIds.find(id) == m_uploadedKeyframeIds.end()) {
                m_pendingUploads.push_back(id);
            }
        }
    }

    int uploaded = 0;
    while (!m_pendingUploads.empty() && uploaded < UPLOAD_QUOTA) {
        long id = m_pendingUploads.front();
        m_pendingUploads.pop_front();
        auto it = graph->keyframes.find(id);
        if (it != graph->keyframes.end()) {
            auto view = std::make_shared<hdl_graph_slam::KeyFrameView>(it->second);
            m_keyframeViews[id] = view;
            m_drawables.push_back(view);
            m_uploadedKeyframeIds.insert(id);
            uploaded++;
        }
    }

    // Remove stale views not in the current graph
    for (auto it = m_drawables.begin(); it != m_drawables.end();) {
        auto kv = std::dynamic_pointer_cast<hdl_graph_slam::KeyFrameView>(*it);
        if (kv) {
            auto kf = kv->lock();
            if (!kf || graph->keyframes.find(kf->id()) == graph->keyframes.end()) {
                m_keyframeViews.erase(kf ? kf->id() : -1);
                it = m_drawables.erase(it);
                continue;
            }
        }
        ++it;
    }
}

void GraphViewportRenderer::render() {
    if (!m_glInitialized) { initGL(); setupShaders(); }
    if (!m_rainbowShader) return;

    m_gl->glDepthMask(GL_TRUE);
    m_gl->glViewport(0, 0, m_viewportSize.width(), m_viewportSize.height());
    m_gl->glClearColor(0.051f, 0.059f, 0.071f, 1.0f);
    m_gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m_gl->glEnable(GL_DEPTH_TEST);

    m_rainbowShader->use();

    // Perspective projection
    float aspect = (float)m_viewportSize.width() / std::max(m_viewportSize.height(), 1);
    float fovY = 45.0f * 3.14159265f / 180.0f;
    float tanHalfFov = std::tan(fovY / 2.0f);
    Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
    proj(0, 0) = 1.0f / (aspect * tanHalfFov);
    proj(1, 1) = 1.0f / tanHalfFov;
    proj(2, 2) = -(1000.0f + 0.1f) / (1000.0f - 0.1f);
    proj(2, 3) = -(2.0f * 1000.0f * 0.1f) / (1000.0f - 0.1f);
    proj(3, 2) = -1.0f;

    // Simple camera (Phase H will replace with ArcCameraControl)
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    view(2, 3) = -10.0f;  // further back for keyframe view

    m_rainbowShader->set_uniform("view_matrix", view);
    m_rainbowShader->set_uniform("projection_matrix", proj);
    m_rainbowShader->set_uniform("z_range", Eigen::Vector2f(-100.0f, 100.0f));
    m_rainbowShader->set_uniform("z_clipping", int{0});
    m_rainbowShader->set_uniform("point_scale", 1.0f);
    m_rainbowShader->set_uniform("point_size", 3.0f);
    m_rainbowShader->set_uniform("keyframe_scale", 1.0f);
    m_rainbowShader->set_uniform("apply_keyframe_scale", false);
    m_rainbowShader->set_uniform("info_values", Eigen::Vector4i(0, 0, 0, 0));

    // Sync graph data and render drawables
    syncGraphData();

    if (!m_drawables.empty()) {
        // Render all drawable objects (keyframes, point clouds)
        m_rainbowShader->set_uniform("color_mode", 0);
        for (auto& d : m_drawables) {
            if (d && d->available()) {
                d->draw(m_drawFlags, *m_rainbowShader);
            }
        }
    } else {
        // Fallback: axes + grid when no graph is loaded
        Eigen::Matrix4f model_m = Eigen::Matrix4f::Identity();
        m_rainbowShader->set_uniform("model_matrix", model_m);
        m_rainbowShader->set_uniform("color_mode", 1);
        Eigen::Vector4f white(1.0f, 1.0f, 1.0f, 1.0f);
        m_rainbowShader->set_uniform("material_color", white);
        glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM)
            .draw(*m_rainbowShader);
        Eigen::Vector4f grey(0.4f, 0.4f, 0.4f, 1.0f);
        m_rainbowShader->set_uniform("material_color", grey);
        glk::Primitives::instance()->primitive(glk::Primitives::GRID)
            .draw(*m_rainbowShader);
    }

    // FPS counter (throttled to 0.5s)
    m_frameCount++;
    auto now = std::chrono::steady_clock::now();
    float elapsed = std::chrono::duration<float>(now - m_lastFpsEmitTime).count();
    if (elapsed >= 0.5f) {
        m_pendingFpsUpdate = m_frameCount / elapsed;
        m_hasFpsUpdate = true;
        m_frameCount = 0;
        m_lastFpsEmitTime = now;
    }
}
