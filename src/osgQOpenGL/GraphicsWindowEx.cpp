/**
 * @file GraphicsWindowEx.cpp
 * @brief 扩展的 OSG 图形窗口类的实现文件
 *
 * 本文件实现了 GraphicsWindowEx 类的构造和初始化方法。
 * 核心功能是创建 StateEx 实例来替代 OSG 默认的 osg::State，
 * 并正确设置上下文 ID（从共享上下文继承或新建）。
 */

#include "GraphicsWindowEx.h"
#include "StateEx.h"

/**
 * @brief 使用 Traits 参数构造图形窗口
 * @param traits 图形上下文特性参数指针
 *
 * 接收外部传入的 Traits 对象，直接保存后调用 init() 进行初始化。
 */
GraphicsWindowEx::GraphicsWindowEx(osg::GraphicsContext::Traits* traits)
{
    _traits = traits;
    init();
}

/**
 * @brief 使用坐标和尺寸构造图形窗口
 * @param x      窗口左上角 x 坐标
 * @param y      窗口左上角 y 坐标
 * @param width  窗口宽度
 * @param height 窗口高度
 *
 * 根据参数创建新的 Traits 对象，然后调用 init() 进行初始化。
 * 注意：此处存在一个已知问题，第二个 _traits->x = y 应该是 _traits->y = y，
 * 但由于 y 被赋值给了 x，导致 x 和 y 相等。
 */
GraphicsWindowEx::GraphicsWindowEx(int x, int y, int width, int height)
{
    _traits = new osg::GraphicsContext::Traits();
    _traits->x = x;
    _traits->x = y;      // 注意：此处应为 _traits->y = y
    _traits->width = width;
    _traits->height = height;

    init();
}

/**
 * @brief 初始化图形窗口
 *
 * 初始化流程：
 * 1. 创建 StateEx 实例替换默认的 osg::State（支持追踪 Qt 默认 FBO）
 * 2. 将 State 与此 GraphicsContext 进行关联
 * 3. 如果有共享上下文，则继承其 ContextID 并增加引用计数
 * 4. 如果没有共享上下文，则创建新的 ContextID
 */
void GraphicsWindowEx::init()
{
    if(valid())
    {
        // 注入扩展的 State 对象（StateEx），用于追踪 Qt 默认 FBO ID
        setState(new StateEx());
        getState()->setGraphicsContext(this);

        if(_traits.valid() && _traits->sharedContext.valid())
        {
            // 如果有共享上下文，继承其 ContextID 以共享 OpenGL 资源
            getState()->setContextID(_traits->sharedContext->getState()->getContextID());
            incrementContextIDUsageCount(getState()->getContextID());
        }
        else
        {
            // 没有共享上下文，创建新的唯一 ContextID
            getState()->setContextID(osg::GraphicsContext::createNewContextID());
        }
    }
}
