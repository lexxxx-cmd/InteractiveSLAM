/**
 * @file StateEx.h
 * @brief OpenGL 状态管理扩展类的头文件
 *
 * 本文件定义了 StateEx 类，继承自 osg::State，
 * 用于扩展 OSG 的 OpenGL 状态管理，支持将 OSG 与 Qt 的 QPainter 2D 绘制混合使用。
 * 主要功能是保存和恢复默认帧缓冲区对象（FBO）的 ID，以便在 OSG 和 Qt 之间切换渲染目标。
 *
 * 参考: http://forum.openscenegraph.org/viewtopic.php?t=15627&view=previous
 */

#ifndef STATEEX_H
#define STATEEX_H

#include "Export"

#include <osg/State>

/**
 * @class StateEx
 * @brief OSG OpenGL 状态管理扩展类
 *
 * 继承自 osg::State，在标准状态管理基础上增加了对默认 FBO（帧缓冲区对象）ID 的跟踪。
 * 当 OSG 渲染与 Qt 的 QPainter 2D 绘制混合使用时，需要在两者之间切换帧缓冲区。
 * 此类通过存储 Qt 提供的默认 FBO ID，使得 OSG 渲染完成后能正确地将渲染目标恢复为 Qt 的默认帧缓冲区。
 */
class StateEx : public osg::State
{
public:
    StateEx() : defaultFbo(0) {}

    /**
     * @brief 设置默认帧缓冲区对象 ID
     * @param fbo Qt 提供的默认帧缓冲区对象 ID（通常来自 QOpenGLWidget::defaultFramebufferObject()）
     */
    inline void setDefaultFbo(GLuint fbo)
    {
        defaultFbo = fbo;
    }

    /**
     * @brief 获取默认帧缓冲区对象 ID
     * @return 当前保存的默认 FBO ID
     */
    inline GLuint getDefaultFbo() const
    {
        return defaultFbo;
    }

protected:
    /** @brief 保存的默认帧缓冲区对象 ID，用于在 OSG 渲染后将渲染目标恢复为 Qt 的默认帧缓冲区 */
    GLuint defaultFbo;
};

#endif // STATEEX_H
