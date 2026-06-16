#pragma once

#include <QOpenGLFunctions_3_0>
#include <QOpenGLContext>

namespace glk {

/// Returns a QOpenGLFunctions_3_0 bound to the current OpenGL context.
/// Must only be called from the Scene Graph render thread (inside
/// QQuickFramebufferObject::Renderer::render() or synchronize()).
///
/// On first call per thread, creates and initializes the functions object.
/// The instance lives for the lifetime of the render thread.
inline QOpenGLFunctions_3_0* gl() {
    thread_local std::unique_ptr<QOpenGLFunctions_3_0> instance;
    if (!instance) {
        instance = std::make_unique<QOpenGLFunctions_3_0>();
        instance->initializeOpenGLFunctions();
    }
    return instance.get();
}

} // namespace glk
