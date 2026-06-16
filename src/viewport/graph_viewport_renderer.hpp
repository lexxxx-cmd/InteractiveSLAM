#pragma once

#include <QQuickFramebufferObject>
#include <QOpenGLFramebufferObject>
#include <QOpenGLExtraFunctions>
#include <memory>
#include <Eigen/Core>
#include "glk/glsl_shader.hpp"
#include "glk/mesh.hpp"

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

    QOpenGLExtraFunctions* m_gl = nullptr;
    std::unique_ptr<glk::GLSLShader> m_rainbowShader;
    QSize m_viewportSize;
    bool m_glInitialized = false;
};
