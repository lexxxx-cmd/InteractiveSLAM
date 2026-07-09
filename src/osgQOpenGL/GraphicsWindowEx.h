/**
 * @file GraphicsWindowEx.h
 * @brief 扩展的 OSG 图形窗口类的头文件
 *
 * 本文件定义了 GraphicsWindowEx 类，继承自 osgViewer::GraphicsWindow，
 * 用于在 OSG 与 Qt 集成时提供一个"虚拟"的图形窗口实现。
 * 当使用 QOpenGLWidget 或 QOpenGLWindow 作为渲染表面时，
 * OpenGL 上下文由 Qt 管理，OSG 不需要实际创建和管理窗口。
 * 因此 GraphicsWindowEx 的大部分方法都是空实现或返回默认值。
 *
 * 核心功能：
 * 1. 使用 StateEx 替代默认的 osg::State，支持 Qt 默认 FBO 的追踪
 * 2. 所有窗口管理操作均为空实现（有效、已实现、上下文切换等均返回 true）
 * 3. 管理上下文 ID 的创建和引用计数
 *
 * 参考: http://forum.openscenegraph.org/viewtopic.php?t=15627&view=previous
 */

#ifndef GRAPHICSWINDOWEX_H
#define GRAPHICSWINDOWEX_H

#include "Export"

#include <osgViewer/GraphicsWindow>

/**
 * @class GraphicsWindowEx
 * @brief 扩展的 OSG 图形窗口类，用于 OSG 与 Qt 集成
 *
 * 此类为 OSG 提供一个虚拟的 GraphicsWindow 接口实现。
 * 在实际的 OSG+Qt 集成场景中，OpenGL 渲染上下文完全由 Qt 管理，
 * OSG 不需要真正地创建窗口或管理上下文切换。
 *
 * 因此所有窗口相关的方法（valid、realize、makeCurrent、swapBuffers 等）
 * 均为空实现或直接返回 true，假设图形上下文始终有效且处于当前状态。
 *
 * 关键特性：在 init() 方法中创建 StateEx 实例，支持追踪 Qt 的默认 FBO ID。
 */
class GraphicsWindowEx : public osgViewer::GraphicsWindow
{
public:
    /**
     * @brief 使用 Traits 构造图形窗口
     * @param traits 图形上下文特性参数（包含 x、y、width、height 等）
     */
    GraphicsWindowEx(osg::GraphicsContext::Traits* traits);

    /**
     * @brief 使用坐标和尺寸构造图形窗口
     * @param x      窗口左上角 x 坐标
     * @param y      窗口左上角 y 坐标
     * @param width  窗口宽度
     * @param height 窗口高度
     */
    GraphicsWindowEx(int x, int y, int width, int height);

    /**
     * @brief 初始化图形窗口
     *
     * 创建 StateEx 实例替换默认的 osg::State，设置图形上下文。
     * 如果共享了上下文，则使用共享上下文的 ContextID；否则创建新的 ContextID。
     */
    void init();

    /** @brief 判断是否为同类型对象 */
    virtual bool isSameKindAs(const osg::Object* object) const
    {
        return dynamic_cast<const GraphicsWindowEx*>(object) != 0;
    }

    /** @brief 返回库名称 */
    virtual const char* libraryName() const
    {
        return "";
    }

    /** @brief 返回类名称 */
    virtual const char* className() const
    {
        return "GraphicsWindowEx";
    }

    // **********************************************************************
    // 虚拟实现：以下方法均为空实现或返回默认值
    // 因为 OpenGL 上下文完全由 Qt 管理，OSG 不需要实际操作窗口
    // **********************************************************************

    /** @brief 始终返回 true，假设图形上下文始终有效 */
    virtual bool valid() const { return true; }

    /** @brief 空实现，返回 true */
    virtual bool realizeImplementation() { return true; }

    /** @brief 始终返回 true，假设始终处于已实现状态 */
    virtual bool isRealizedImplementation() const { return true; }

    /** @brief 空实现，不需要关闭任何窗口资源 */
    virtual void closeImplementation() {}

    /** @brief 始终返回 true，假设上下文始终是当前上下文 */
    virtual bool makeCurrentImplementation() { return true; }

    /** @brief 始终返回 true，假设不需要释放上下文 */
    virtual bool releaseContextImplementation() { return true; }

    /** @brief 空实现，交换缓冲区由 Qt 管理 */
    virtual void swapBuffersImplementation() {}

    /** @brief 空实现，焦点由 Qt 管理 */
    virtual void grabFocus() {}

    /** @brief 空实现 */
    virtual void grabFocusIfPointerInWindow() {}

    /** @brief 空实现，窗口层级由 Qt 管理 */
    virtual void raiseWindow() {}
};

#endif // GRAPHICSWINDOWEX_H
