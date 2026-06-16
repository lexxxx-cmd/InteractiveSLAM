#pragma once

#include <QOpenGLFunctions_3_0>
#include <QOpenGLContext>
#include <QOpenGLVersionFunctionsFactory>

namespace glk {

/// Returns a QOpenGLFunctions_3_0 bound to the current OpenGL context.
/// Must only be called from the Scene Graph render thread (inside
/// QQuickFramebufferObject::Renderer::render() or synchronize()).
///
/// Uses QOpenGLVersionFunctionsFactory (Qt 6 standard API) to obtain
/// versioned functions from the current context.
inline QOpenGLFunctions_3_0* gl() {
    thread_local QOpenGLFunctions_3_0* instance = nullptr;
    if (!instance) {
        auto* ctx = QOpenGLContext::currentContext();
        instance = QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_3_0>(ctx);
        Q_ASSERT(instance);
    }
    return instance;
}

} // namespace glk
