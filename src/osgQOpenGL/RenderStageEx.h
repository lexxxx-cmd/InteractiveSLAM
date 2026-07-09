/**
 * @file RenderStageEx.h
 * @brief 自定义渲染阶段扩展类的头文件
 *
 * 本文件定义了 RenderStageEx 类，继承自 osgUtil::RenderStage，
 * 用于扩展 OSG 的渲染阶段（RenderStage）行为。
 * 主要目的是在渲染过程中注入自定义逻辑，支持将 OSG 渲染与 Qt 的 QPainter 2D 绘制混合使用。
 * 通过在 drawInner 方法中拦截渲染调用，确保 OSG 渲染到 Qt 提供的帧缓冲区中。
 *
 * 参考: http://forum.openscenegraph.org/viewtopic.php?t=15627&view=previous
 */

#ifndef RENDERSTAGEEX_H
#define RENDERSTAGEEX_H

#include "Export"

#include <osgUtil/RenderStage>

/**
 * @class RenderStageEx
 * @brief 自定义渲染阶段扩展类
 *
 * 继承自 osgUtil::RenderStage，重写 drawInner 方法以支持 OSG 与 Qt 的混合渲染。
 * 在渲染内部绘制时，它会检查是否使用了帧缓冲区对象（FBO），
 * 如果是，则将渲染目标绑定到 Qt 的默认 FBO（通过 StateEx 获取），
 * 从而确保 OSG 场景正确地渲染到 Qt 窗口表面上。
 */
class RenderStageEx : public osgUtil::RenderStage
{
public:
    /**
     * @brief 重写内部绘制方法
     * @param renderInfo 渲染信息对象，包含当前状态和上下文信息
     * @param previous   上一个渲染叶节点指针，用于绘制顺序跟踪
     * @param doCopyTexture 引用标志，指示是否需要将渲染结果复制到关联纹理
     *
     * 此方法在默认的 RenderStage 绘制基础上，增加了对 Qt 默认 FBO 的绑定逻辑，
     * 使得 OSG 渲染能够正确地与 Qt 的 QPainter 2D 绘制协同工作。
     */
    virtual void drawInner(osg::RenderInfo& renderInfo,
                           osgUtil::RenderLeaf*& previous, bool& doCopyTexture);
};

#endif // RENDERSTAGEEX_H
