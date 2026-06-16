#pragma once

#include <QOpenGLFunctions_3_0>
#include <QOpenGLContext>

namespace glk {

/// Returns the QOpenGLFunctions_3_0 for the current OpenGL context.
/// Must only be called from the Scene Graph render thread (inside
/// QQuickFramebufferObject::Renderer::render() or synchronize()).
inline QOpenGLFunctions_3_0* gl() {
    auto* f = QOpenGLContext::currentContext()
        ->versionFunctions<QOpenGLFunctions_3_0>();
    Q_ASSERT(f);
    return f;
}

} // namespace glk
