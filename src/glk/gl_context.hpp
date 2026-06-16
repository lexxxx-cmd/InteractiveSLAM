#pragma once

#include <QOpenGLExtraFunctions>
#include <QOpenGLContext>

namespace glk {

/// Returns QOpenGLExtraFunctions bound to the current OpenGL context.
/// Works with both desktop GL and GLES 3.0+ (including ANGLE on Windows).
/// Provides VAO, FBO, and all GL 3.0+ features via Qt's extension mechanism.
///
/// Must only be called from the Scene Graph render thread (inside
/// QQuickFramebufferObject::Renderer::render()).
inline QOpenGLExtraFunctions* gl() {
    thread_local QOpenGLExtraFunctions* instance = nullptr;
    if (!instance) {
        auto* ctx = QOpenGLContext::currentContext();
        instance = ctx->extraFunctions();
        Q_ASSERT(instance);
    }
    return instance;
}

} // namespace glk
