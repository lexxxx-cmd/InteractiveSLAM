/**
 * @file CullVisitorEx.h
 * @brief 自定义裁剪访问器扩展类的头文件
 *
 * 本文件定义了 CullVisitorEx 类，继承自 osgUtil::CullVisitor，
 * 用于扩展 OSG 的裁剪遍历（Cull Traversal）行为。
 * 主要目的是当遇到 Camera 节点时，创建自定义的渲染阶段（RenderStageEx）
 * 来替代默认的 RenderStage，以便支持 OSG 与 Qt 的 QPainter 2D 绘制混合使用。
 *
 * 参考: http://forum.openscenegraph.org/viewtopic.php?t=15627&view=previous
 */

#ifndef CULLVISITOREX_H
#define CULLVISITOREX_H

#include "Export"

#include <osgUtil/CullVisitor>

/**
 * @class CullVisitorEx
 * @brief 自定义裁剪访问器扩展类
 *
 * 继承自 osgUtil::CullVisitor，重写 apply(osg::Camera& camera) 方法。
 * 在场景图的裁剪遍历阶段，当遇到 Camera 节点时，此类的 apply 方法会：
 * 1. 创建 RenderStageEx 实例（而非默认的 RenderStage）
 * 2. 将 Camera 的渲染状态（清除颜色、视口、绘制缓冲等）配置到新的渲染阶段
 * 3. 根据渲染顺序（PRE_RENDER / POST_RENDER）将渲染阶段添加到依赖列表中
 *
 * 这是 OSG 与 Qt 集成方案中的关键组件，确保 OSG 渲染到正确的帧缓冲区目标。
 */
class CullVisitorEx : public osgUtil::CullVisitor
{
public:
    META_NodeVisitor(Ex, CullVisitorEx)

    CullVisitorEx() {}
    CullVisitorEx(const CullVisitorEx& cv) : osgUtil::CullVisitor(cv) { }

    /** @brief 克隆当前访问器 */
    CullVisitorEx* clone() const
    {
        return new CullVisitorEx(*this);
    }

    /**
     * @brief 应用裁剪访问到 Camera 节点
     * @param camera 需要处理的 Camera 节点
     *
     * 重写父类的 apply 方法，为 Camera 节点创建使用 RenderStageEx 的渲染阶段，
     * 并正确处理渲染状态的继承、视口设置、投影和模型视图矩阵的压栈/出栈等操作。
     */
    virtual void apply(osg::Camera& camera);
};

#endif // CULLVISITOREX_H
