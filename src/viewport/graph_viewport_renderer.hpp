#pragma once

#include <QQuickFramebufferObject>
#include <QOpenGLFramebufferObject>
#include <QOpenGLFunctions_3_0>
#include <memory>
#include <Eigen/Core>
#include "glk/glsl_shader.hpp"

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

    QOpenGLFunctions_3_0* m_gl = nullptr;
    std::unique_ptr<glk::GLSLShader> m_rainbowShader;
    QSize m_viewportSize;
    bool m_glInitialized = false;
};
