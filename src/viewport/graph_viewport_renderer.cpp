#include "viewport/graph_viewport_renderer.hpp"
#include "viewport/graph_viewport.hpp"
#include "glk/primitives.hpp"
#include <QOpenGLContext>
#include <QQuickWindow>
#include <QDebug>
#include <cmath>
#include <algorithm>
#include <chrono>
#include <vector>
#include <limits>

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
    // Do NOT call setInternalTextureFormat — let Qt pick its default sized
    // format (e.g. GL_RGBA8).  Unsized GL_RGBA breaks Qt RHI compositing.

    m_fboSize = size;
    auto* fbo = new QOpenGLFramebufferObject(size, format);

    // Attachment 1: also GL_RGBA8 — same type as attachment 0.
    // No integer formats → no driver compatibility issues with MRT.
    fbo->addColorAttachment(size);   // default = GL_RGBA8

    if (!fbo->isValid()) {
        qWarning() << "[PICK] addColorAttachment failed — picking disabled";
        m_infoAttachmentAdded = false;
        delete fbo;
        return new QOpenGLFramebufferObject(size, format);
    }
    m_infoAttachmentAdded = true;
    return fbo;
}

void GraphViewportRenderer::synchronize(QQuickFramebufferObject* item) {
    m_viewportSize = item->size().toSize();
    auto* viewport = static_cast<GraphViewport*>(item);
    auto* manager = viewport->graphManager();

    if (manager && manager->isLoaded()) {
        m_renderGraph = manager->sharedGraph();
    } else {
        m_renderGraph.reset();
    }

    // Sync camera view matrix (main thread → render thread)
    if (viewport->cameraDirty()) {
        m_cameraView = viewport->cameraViewMatrix();
        viewport->clearCameraDirty();
    }

    if (m_hasFpsUpdate) {
        viewport->receiveFpsUpdate(m_pendingFpsUpdate);
        m_hasFpsUpdate = false;
    }

    // ---- Selection sync (every frame, not just pick frames) ----
    m_drawFlags.selectedVertexId = viewport->m_selectedVertexId;

    // ---- Pick bridge: main thread → render thread ----
    if (viewport->m_pickPending) {
        m_pickRequested     = true;
        m_pickLogicalX      = viewport->m_pickMouseX;
        m_pickLogicalY      = viewport->m_pickMouseY;
        m_devicePixelRatio  = item->window()->devicePixelRatio();
        viewport->m_pickPending = false;
        qDebug() << "[PICK] request accepted logicalX=" << m_pickLogicalX
                 << "logicalY=" << m_pickLogicalY << "dpr=" << m_devicePixelRatio;
    }

    // ---- Pick result: render thread → main thread ----
    if (m_pickResultReady) {
        viewport->m_pickedVertexId = m_pickResult.vertexId;
        viewport->m_pickedWorldPos = m_pickResult.worldPos;

        // Refresh highlight set from window scan
        m_highlightedObjects.clear();
        for (auto& obj : m_pickResult.allHitObjects)
            m_highlightedObjects.insert(obj);
        m_highlightFramesRemaining = 90;   // ~1.5s at 60fps

        m_pickResultReady = false;
        emit viewport->pickResultReady();
    }

    item->update();
}

void GraphViewportRenderer::initGL() {
    m_gl = glk::gl();
    if (!m_gl) qFatal("GraphViewportRenderer: OpenGL 3.0 not available");
    qDebug() << "GraphViewportRenderer: OpenGL version"
             << (const char*)m_gl->glGetString(GL_VERSION);
    m_lineBuffer = std::make_unique<hdl_graph_slam::LineBuffer>();
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
    if (!graph) { return; }

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

    // Remove stale keyframe views
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

    // Sync edge views — create once per edge
    for (auto& edge : graph->graph->edges()) {
        long edge_id = edge->id();
        if (m_edgeViews.find(edge_id) == m_edgeViews.end()) {
            auto view = hdl_graph_slam::EdgeView::create(edge, *m_lineBuffer);
            if (view) {
                m_edgeViews[edge_id] = view;
                m_drawables.push_back(view);
            }
        }
    }
}

void GraphViewportRenderer::render() {
    if (!m_glInitialized) { initGL(); setupShaders(); }
    if (!m_rainbowShader) return;

    m_gl->glDepthMask(GL_TRUE);
    m_gl->glViewport(0, 0, m_fboSize.width(), m_fboSize.height());
    m_gl->glClearColor(0.051f, 0.059f, 0.071f, 1.0f);
    m_gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m_gl->glEnable(GL_DEPTH_TEST);

    m_rainbowShader->use();

    const bool pickThisFrame = m_pickRequested && m_infoAttachmentAdded;

    // Enable MRT ONLY on pick frames — normal frames let Qt manage draw buffers
    if (pickThisFrame) {
        GLenum drawBuffers[] = {GL_COLOR_ATTACHMENT0, GL_COLOR_ATTACHMENT1};
        m_gl->glDrawBuffers(2, drawBuffers);

        // Diagnostic: check FBO is still complete with both attachments active
        GLenum status = m_gl->glCheckFramebufferStatus(GL_FRAMEBUFFER);
        if (status != GL_FRAMEBUFFER_COMPLETE)
            qWarning() << "[PICK] FBO incomplete after MRT setup:" << Qt::hex << status;

        // Clear info buffer to all zeros → decodes to typeFlags=0 (no object).
        // GL_RGBA8 ⇒ must use glClearBufferfv, not glClearBufferiv.
        GLfloat clearInfo[] = {0.0f, 0.0f, 0.0f, 0.0f};
        m_gl->glClearBufferfv(GL_COLOR, 1, clearInfo);

        GLenum e = m_gl->glGetError();
        if (e != GL_NO_ERROR)
            qWarning() << "[PICK] MRT setup error:" << Qt::hex << e;
    }

    // Perspective projection
    float aspect = (float)m_fboSize.width() / std::max(m_fboSize.height(), 1);
    float fovY = 45.0f * 3.14159265f / 180.0f;
    float tanHalfFov = std::tan(fovY / 2.0f);
    Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
    proj(0, 0) = 1.0f / (aspect * tanHalfFov);
    proj(1, 1) = 1.0f / tanHalfFov;
    proj(2, 2) = -(1000.0f + 0.1f) / (1000.0f - 0.1f);
    proj(2, 3) = -(2.0f * 1000.0f * 0.1f) / (1000.0f - 0.1f);
    proj(3, 2) = -1.0f;
    m_projectionMatrix = proj;

    // Use camera from QML mouse interaction (defaults to z=-10 if untouched)
    m_rainbowShader->set_uniform("view_matrix", m_cameraView);
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

    // ---- Highlight decay ----
    if (m_highlightFramesRemaining > 0) {
        m_highlightFramesRemaining--;
        if (m_highlightFramesRemaining == 0)
            m_highlightedObjects.clear();
    }

    const bool hasHighlight = !m_highlightedObjects.empty();

    // Set up highlight flags before normal draw — drawables check them internally
    if (hasHighlight) {
        m_drawFlags.highlightPass = true;
        m_drawFlags.highlightedSet = &m_highlightedObjects;
    }

    if (!m_drawables.empty()) {
        m_lineBuffer->clear();

        m_rainbowShader->set_uniform("color_mode", 0);
        for (auto& d : m_drawables) {
            if (d && d->available())
                d->draw(m_drawFlags, *m_rainbowShader);
        }

        m_lineBuffer->draw(*m_rainbowShader);
    }

    // Coordinate axes at origin — always visible as orientation reference
    {
        Eigen::Matrix4f id = Eigen::Matrix4f::Identity();
        m_rainbowShader->set_uniform("model_matrix", id);
        m_rainbowShader->set_uniform("color_mode", 2);  // uses vert_color
        m_rainbowShader->set_uniform("info_values", Eigen::Vector4i(0,0,0,0));
        glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM)
            .draw(*m_rainbowShader);
    }

    if (m_drawables.empty()) {
        // Extra grid when no graph is loaded
        Eigen::Matrix4f id = Eigen::Matrix4f::Identity();
        m_rainbowShader->set_uniform("model_matrix", id);
        m_rainbowShader->set_uniform("color_mode", 1);
        Eigen::Vector4f grey(0.4f, 0.4f, 0.4f, 1.0f);
        m_rainbowShader->set_uniform("material_color", grey);
        glk::Primitives::instance()->primitive(glk::Primitives::GRID)
            .draw(*m_rainbowShader);
    }

    // Restore highlight flags
    if (hasHighlight) {
        m_drawFlags.highlightPass = false;
        m_drawFlags.highlightedSet = nullptr;
    }

    // FPS counter (throttled to 0.5s)
    m_frameCount++;

    // ---- GPU pick readback (only on pick frames) ----
    if (pickThisFrame) {
        doPickReadback(m_pickLogicalX, m_pickLogicalY, m_devicePixelRatio);
        m_pickRequested = false;
        m_pickResultReady = true;

        // Restore: only changed draw buffers on this frame, so restore now
        GLenum singleBuffer[] = {GL_COLOR_ATTACHMENT0};
        m_gl->glDrawBuffers(1, singleBuffer);
        m_gl->glReadBuffer(GL_COLOR_ATTACHMENT0);
    }
}

void GraphViewportRenderer::doPickReadback(float logicalX, float logicalY, float dpr) {
    m_pickResult = {};   // reset to no-hit

    // ---- Logical pixels → FBO device pixels (Y-flip) ----
    int fboX = static_cast<int>(std::round(logicalX * dpr));
    int fboY = m_fboSize.height() - 1
             - static_cast<int>(std::round(logicalY * dpr));

    if (fboX < 0 || fboY < 0 ||
        fboX >= m_fboSize.width() || fboY >= m_fboSize.height())
        return;

    constexpr int window = 5;
    const int size  = window * 2 + 1;
    const int readX = std::max(0, fboX - window);
    const int readY = std::max(0, fboY - window);
    const int readW = std::min(size, m_fboSize.width()  - readX);
    const int readH = std::min(size, m_fboSize.height() - readY);

    // ---- Read info attachment (RGBA8 → raw bytes, ~484 Bytes) ----
    std::vector<unsigned char> infoPixels(readW * readH * 4);
    m_gl->glReadBuffer(GL_COLOR_ATTACHMENT1);
    m_gl->glReadPixels(readX, readY, readW, readH,
                       GL_RGBA, GL_UNSIGNED_BYTE, infoPixels.data());

    // ---- Spiral search + decode ----
    // Shader packs: R=typeFlags.lo, G=typeFlags.hi, B=vertexId.lo, A=vertexId.hi
    // Background cleared to all zeros → typeFlags=0 → no hit
    auto decode = [](const unsigned char* p) -> std::pair<int, int> {
        int typeFlags = p[0] | (p[1] << 8);
        int vertexId  = p[2] | (p[3] << 8);
        if (vertexId >= 32768) vertexId -= 65536;   // sign-extend 16-bit → 32-bit
        return {typeFlags, vertexId};
    };

    // ---- Collect all pickable objects in scan window ----
    m_pickResult.allHitObjects.clear();
    for (int ly = 0; ly < readH; ly++) {
        for (int lx = 0; lx < readW; lx++) {
            auto [tf, vid] = decode(&infoPixels[(ly * readW + lx) * 4]);
            if (tf & (hdl_graph_slam::DrawableObject::VERTEX |
                      hdl_graph_slam::DrawableObject::EDGE))
                m_pickResult.allHitObjects.push_back({tf, vid});
        }
    }

    std::vector<Eigen::Vector2i> offsets;
    offsets.reserve(size * size);
    for (int dy = -window; dy <= window; dy++)
        for (int dx = -window; dx <= window; dx++)
            offsets.push_back(Eigen::Vector2i(dx, dy));
    std::sort(offsets.begin(), offsets.end(),
              [](const Eigen::Vector2i& a, const Eigen::Vector2i& b) {
                  return a.squaredNorm() < b.squaredNorm();
              });

    int hitLocalX = -1, hitLocalY = -1;
    for (const auto& offset : offsets) {
        int lx = (fboX - readX) + offset.x();
        int ly = (fboY - readY) + offset.y();
        if (lx < 0 || ly < 0 || lx >= readW || ly >= readH) continue;

        auto [typeFlags, vertexId] = decode(&infoPixels[(ly * readW + lx) * 4]);
        if (typeFlags & (hdl_graph_slam::DrawableObject::VERTEX |
                          hdl_graph_slam::DrawableObject::EDGE)) {
            m_pickResult.hit      = true;
            m_pickResult.vertexId = vertexId;
            hitLocalX = lx;
            hitLocalY = ly;
            break;
        }
    }

    if (!m_pickResult.hit) return;

    // ---- Read depth at the hit pixel (same frame, same pixel as the ID) ----
    int hitFboX = readX + hitLocalX;
    int hitFboY = readY + hitLocalY;
    float depth = 1.0f;
    m_gl->glReadBuffer(GL_NONE);
    m_gl->glReadPixels(hitFboX, hitFboY, 1, 1,
                       GL_DEPTH_COMPONENT, GL_FLOAT, &depth);

    // ---- Unproject: use the actual hit pixel, not the mouse center ----
    // FBO coords: Y=0 at bottom, NDC: Y=-1 at bottom — no flip needed
    float ndcX = (hitFboX + 0.5f) / m_fboSize.width()  * 2.0f - 1.0f;
    float ndcY = (hitFboY + 0.5f) / m_fboSize.height() * 2.0f - 1.0f;
    float clipZ = depth * 2.0f - 1.0f;   // [0,1] → [-1,1]

    Eigen::Vector4f clipPos(ndcX, ndcY, clipZ, 1.0f);
    Eigen::Matrix4f vp = m_projectionMatrix * m_cameraView;
    Eigen::Matrix4f invVP = vp.inverse();
    Eigen::Vector4f worldPos = invVP * clipPos;

    m_pickResult.worldPos = worldPos.head<3>() / worldPos.w();

    // Verify: unprojected position vs keyframe center
    if (m_renderGraph) {
        auto it = m_renderGraph->keyframes.find(m_pickResult.vertexId);
        if (it != m_renderGraph->keyframes.end() && it->second) {
            Eigen::Vector3d c = it->second->estimate().translation();
            qDebug() << "[PICK] worldPos" << m_pickResult.worldPos.x()
                     << m_pickResult.worldPos.y() << m_pickResult.worldPos.z()
                     << "center" << c.x() << c.y() << c.z()
                     << "dist" << (m_pickResult.worldPos - c.cast<float>()).norm();
        }
    }
}
