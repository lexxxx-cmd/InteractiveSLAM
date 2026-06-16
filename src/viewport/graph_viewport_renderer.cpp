#include "viewport/graph_viewport_renderer.hpp"
#include "viewport/graph_viewport.hpp"
#include "glk/primitives.hpp"
#include <QOpenGLContext>
#include <QDebug>
#include <cmath>
#include <algorithm>
#include <iostream>

GraphViewportRenderer::GraphViewportRenderer() = default;

GraphViewportRenderer::~GraphViewportRenderer() {
    cleanupGL();
}

void GraphViewportRenderer::cleanupGL() {
    if (!m_gl) return;
    m_rainbowShader.reset();  // deletes shader program
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
    item->update();  // keep rendering continuously
}

void GraphViewportRenderer::initGL() {
    m_gl = glk::gl();  // thread_local QOpenGLFunctions_3_0
    if (!m_gl) {
        qFatal("GraphViewportRenderer: OpenGL 3.0 not available");
    }
    const char* version = (const char*)m_gl->glGetString(GL_VERSION);
    qDebug() << "GraphViewportRenderer: OpenGL version" << version;
    m_glInitialized = true;
}

void GraphViewportRenderer::setupShaders() {
    m_rainbowShader = std::make_unique<glk::GLSLShader>();
    if (!m_rainbowShader->init(QStringLiteral(":/rainbow.vert"),
                                QStringLiteral(":/rainbow.frag"))) {
        qFatal("GraphViewportRenderer: failed to load rainbow shader");
    }
}

void GraphViewportRenderer::render() {
    if (!m_glInitialized) {
        initGL();
        setupShaders();
    }

    // ⚠ Force depth write — Qt Quick 2D may leave glDepthMask at GL_FALSE
    m_gl->glDepthMask(GL_TRUE);

    m_gl->glViewport(0, 0, m_viewportSize.width(), m_viewportSize.height());
    m_gl->glClearColor(0.051f, 0.059f, 0.071f, 1.0f);  // #0D0F12
    m_gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m_gl->glEnable(GL_DEPTH_TEST);

    m_rainbowShader->use();

    // Perspective projection (45° FOV, near=0.1, far=1000)
    float aspect = (float)m_viewportSize.width() / std::max(m_viewportSize.height(), 1);
    float fovY = 45.0f * 3.14159265f / 180.0f;
    float tanHalfFov = std::tan(fovY / 2.0f);

    Eigen::Matrix4f proj = Eigen::Matrix4f::Zero();
    proj(0, 0) = 1.0f / (aspect * tanHalfFov);
    proj(1, 1) = 1.0f / tanHalfFov;
    proj(2, 2) = -(1000.0f + 0.1f) / (1000.0f - 0.1f);
    proj(2, 3) = -(2.0f * 1000.0f * 0.1f) / (1000.0f - 0.1f);
    proj(3, 2) = -1.0f;

    // View: camera at (0, 0, 5), looking at origin
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    view(2, 3) = -5.0f;

    m_rainbowShader->set_uniform("view_matrix", view);
    m_rainbowShader->set_uniform("projection_matrix", proj);
    m_rainbowShader->set_uniform("z_range", Eigen::Vector2f(-1.5f, 5.0f));
    m_rainbowShader->set_uniform("z_clipping", int{0});
    m_rainbowShader->set_uniform("point_scale", 1.0f);
    m_rainbowShader->set_uniform("point_size", 50.0f);
    m_rainbowShader->set_uniform("keyframe_scale", 1.0f);
    m_rainbowShader->set_uniform("apply_keyframe_scale", false);
    m_rainbowShader->set_uniform("info_values", Eigen::Vector4i(0, 0, 0, 0));

    // Coordinate axes (solid white)
    m_rainbowShader->set_uniform("color_mode", 1);
    Eigen::Vector4f white(1.0f, 1.0f, 1.0f, 1.0f);
    m_rainbowShader->set_uniform("material_color", white);
    Eigen::Matrix4f model_m = Eigen::Matrix4f::Identity();
    m_rainbowShader->set_uniform("model_matrix", model_m);
    glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM)
        .draw(*m_rainbowShader);

    // Grid (solid grey)
    Eigen::Vector4f grey(0.4f, 0.4f, 0.4f, 1.0f);
    m_rainbowShader->set_uniform("material_color", grey);
    glk::Primitives::instance()->primitive(glk::Primitives::GRID)
        .draw(*m_rainbowShader);

    // Qt Scene Graph manages FBO binding — do NOT call bindDefault()
}
