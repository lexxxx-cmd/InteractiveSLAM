#include "viewport/graph_viewport_renderer.hpp"
#include "viewport/graph_viewport.hpp"
#include "glk/primitives.hpp"
#include <QOpenGLContext>
#include <QDebug>
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
    // Future: sync graph data, draw flags, pick requests, FPS from renderer
    (void)item;
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
    if (!m_rainbowShader->init(":/shaders/rainbow.vert", ":/shaders/rainbow.frag")) {
        qFatal("GraphViewportRenderer: failed to load rainbow shader");
    }
}

void GraphViewportRenderer::render() {
    if (!m_glInitialized) {
        initGL();
        setupShaders();
    }

    // Info FBO size sync placeholder — will be active in Phase I (picking)

    m_gl->glClearColor(0.051f, 0.059f, 0.071f, 1.0f);  // #0D0F12
    m_gl->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    m_gl->glEnable(GL_DEPTH_TEST);

    m_rainbowShader->use();

    // Default view: camera at (0,0,5) looking at origin
    Eigen::Matrix4f view = Eigen::Matrix4f::Identity();
    view(2, 3) = -5.0f;
    m_rainbowShader->set_uniform("view_matrix", view);
    m_rainbowShader->set_uniform("projection_matrix", Eigen::Matrix4f::Identity());
    m_rainbowShader->set_uniform("z_range", Eigen::Vector2f(-1.5f, 5.0f));
    m_rainbowShader->set_uniform("z_clipping", 0);
    m_rainbowShader->set_uniform("point_scale", 1.0f);
    m_rainbowShader->set_uniform("point_size", 50.0f);
    m_rainbowShader->set_uniform("keyframe_scale", 1.0f);
    m_rainbowShader->set_uniform("apply_keyframe_scale", false);
    m_rainbowShader->set_uniform("info_values", Eigen::Vector4i(0, 0, 0, 0));

    // Coordinate axes (color_mode=2: per-vertex color + info)
    m_rainbowShader->set_uniform("color_mode", 2);
    m_rainbowShader->set_uniform("model_matrix", Eigen::Matrix4f::Identity());
    glk::Primitives::instance()->primitive(glk::Primitives::COORDINATE_SYSTEM)
        .draw(*m_rainbowShader);

    // Grid (color_mode=1: solid color)
    m_rainbowShader->set_uniform("color_mode", 1);
    m_rainbowShader->set_uniform("material_color",
        Eigen::Vector4f(0.3f, 0.3f, 0.3f, 1.0f));
    glk::Primitives::instance()->primitive(glk::Primitives::GRID)
        .draw(*m_rainbowShader);

    // Qt Scene Graph manages FBO binding — do NOT call bindDefault()
}
