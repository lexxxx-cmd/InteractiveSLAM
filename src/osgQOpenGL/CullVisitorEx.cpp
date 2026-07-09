/**
 * @file CullVisitorEx.cpp
 * @brief 自定义裁剪访问器扩展类的实现文件
 *
 * 本文件实现了 CullVisitorEx::apply(osg::Camera& camera) 方法，
 * 以及辅助类 RenderStageCacheEx（用于缓存裁剪访问器对应的渲染阶段对象）。
 *
 * 核心功能：
 * 1. 重写 apply(Camera) 方法，为 Camera 节点创建使用 RenderStageEx 的渲染阶段
 * 2. 提供 RenderStageCacheEx 缓存机制，避免重复创建渲染阶段对象
 * 3. 正确处理 Camera 的渲染状态继承（清除设置、视口、绘制/读取缓冲区等）
 * 4. 根据渲染顺序（PRE_RENDER / POST_RENDER）将子渲染阶段注册到依赖列表中
 *
 * 参考: http://forum.openscenegraph.org/viewtopic.php?t=15627&view=previous
 */

#include "CullVisitorEx.h"
#include "RenderStageEx.h"

/**
 * @class RenderStageCacheEx
 * @brief 渲染阶段缓存类，用于缓存裁剪访问器（CullVisitor）对应的渲染阶段对象
 *
 * 继承自 osg::Object 和 osg::Observer。
 * 当多个 Camera 节点需要进行裁剪遍历时，每个裁剪访问器会对应一个渲染阶段对象。
 * 此类管理这些映射关系，并在访问器被销毁时自动清理对应条目。
 * 同时提供 OpenGL 对象缓冲区的 resize 和 release 支持。
 */
class RenderStageCacheEx : public osg::Object, public osg::Observer
{
public:

    /** @brief 渲染阶段映射表类型：CullVisitor -> RenderStage */
    typedef std::map<osgUtil::CullVisitor*, osg::ref_ptr<osgUtil::RenderStage> >
    RenderStageMap;

    RenderStageCacheEx() {}
    RenderStageCacheEx(const RenderStageCacheEx&, const osg::CopyOp&) {}

    /** @brief 析构时移除所有观察者 */
    virtual ~RenderStageCacheEx()
    {
        for(RenderStageMap::iterator itr = _renderStageMap.begin();
            itr != _renderStageMap.end();
            ++itr)
        {
            itr->first->removeObserver(this);
        }
    }

    META_Object(Ex, RenderStageCacheEx)

    /**
     * @brief 观察者回调：当被观察对象被删除时，自动从映射表中移除对应条目
     * @param object 被删除对象的指针
     */
    virtual void objectDeleted(void* object)
    {
        osg::Referenced* ref = reinterpret_cast<osg::Referenced*>(object);
        osgUtil::CullVisitor* cv = dynamic_cast<osgUtil::CullVisitor*>(ref);

        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);

        RenderStageMap::iterator itr = _renderStageMap.find(cv);

        if(itr != _renderStageMap.end())
        {
            _renderStageMap.erase(itr);
        }
    }

    /**
     * @brief 设置裁剪访问器对应的渲染阶段
     * @param cv 裁剪访问器指针
     * @param rs 渲染阶段指针
     *
     * 如果映射不存在，则新建条目并向访问器注册观察者；
     * 如果已存在，则更新对应的渲染阶段对象。
     */
    void setRenderStage(osgUtil::CullVisitor* cv, osgUtil::RenderStage* rs)
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
        RenderStageMap::iterator itr = _renderStageMap.find(cv);

        if(itr == _renderStageMap.end())
        {
            _renderStageMap[cv] = rs;
            cv->addObserver(this);
        }
        else
        {
            itr->second = rs;
        }
    }

    /**
     * @brief 获取裁剪访问器对应的渲染阶段
     * @param cv 裁剪访问器指针
     * @return 对应的渲染阶段指针，如果不存在则返回 0
     */
    osgUtil::RenderStage* getRenderStage(osgUtil::CullVisitor* cv)
    {
        OpenThreads::ScopedLock<OpenThreads::Mutex> lock(_mutex);
        RenderStageMap::iterator itr = _renderStageMap.find(cv);

        if(itr != _renderStageMap.end())
        {
            return itr->second.get();
        }
        else
        {
            return 0;
        }
    }

    /**
     * @brief 调整所有渲染阶段的 OpenGL 对象缓冲区大小
     * @param maxSize 新的最大缓冲区大小
     */
    virtual void resizeGLObjectBuffers(unsigned int maxSize)
    {
        for(RenderStageMap::const_iterator itr = _renderStageMap.begin();
            itr != _renderStageMap.end();
            ++itr)
        {
            itr->second->resizeGLObjectBuffers(maxSize);
        }
    }

    /**
     * @brief 释放所有渲染阶段关联的 OpenGL 对象
     * @param state 指定图形上下文的状态对象，为 0 时释放所有上下文的对象
     */
    virtual void releaseGLObjects(osg::State* state = 0) const
    {
        for(RenderStageMap::const_iterator itr = _renderStageMap.begin();
            itr != _renderStageMap.end();
            ++itr)
        {
            itr->second->releaseGLObjects(state);
        }
    }

    /** @brief 线程安全互斥锁 */
    OpenThreads::Mutex  _mutex;

    /** @brief 裁剪访问器到渲染阶段的映射表 */
    RenderStageMap      _renderStageMap;
};

/**
 * @brief 对 Camera 节点应用裁剪遍历
 * @param camera 需要处理的 Camera 节点
 *
 * 此方法重写了 osgUtil::CullVisitor 的 apply(Camera&) 方法。
 * 处理流程如下：
 *
 * 1. 状态管理：推送 Camera 的 StateSet，保存并继承裁剪设置
 * 2. 矩阵处理：根据 Camera 的参考框架（相对/绝对）和变换顺序，计算投影和模型视图矩阵
 * 3. 远近平面管理：保存当前计算的近远平面值和候选映射
 * 4. 渲染阶段创建：
 *    - 嵌套渲染（NESTED_RENDER）：直接遍历子节点
 *    - 其他渲染顺序：创建/复用 RenderStageEx 实例
 *      - 配置清除状态（颜色、深度、模板、累积）
 *      - 配置颜色掩码、视口、绘制/读取缓冲区
 *      - 设置初始视图矩阵和继承的位置状态容器
 *      - 执行子节点遍历
 *      - 根据渲染顺序（PRE_RENDER / POST_RENDER）将阶段添加到依赖列表
 * 5. 恢复操作：弹出矩阵栈，恢复近远平面值、裁剪设置和遍历掩码
 */
void CullVisitorEx::apply(osg::Camera& camera)
{

    // **********************************************************************
    // 以下代码源自 OSG RenderStage 类的 apply 实现

    // 推送 Camera 节点的状态集到渲染图状态栈
    osg::StateSet* node_state = camera.getStateSet();
    if(node_state) pushStateSet(node_state);

    // **********************************************************************
    // 保存并继承裁剪设置

    // 保存当前裁剪设置
    CullSettings saved_cull_settings(*this);

    // 从 Camera 获取裁剪设置
    setCullSettings(camera);
    // 从上层继承裁剪设置（使用 Camera 的继承掩码控制哪些设置需要继承）
    inheritCullSettings(saved_cull_settings, camera.getInheritanceMask());

    // **********************************************************************
    // 设置裁剪掩码

    unsigned int savedTraversalMask = getTraversalMask();
    bool mustSetCullMask = (camera.getInheritanceMask() &
                            osg::CullSettings::CULL_MASK) == 0;

    if(mustSetCullMask) setTraversalMask(camera.getCullMask());

    // **********************************************************************
    // 计算投影和模型视图矩阵

    osg::RefMatrix& originalModelView = *getModelViewMatrix();

    osg::RefMatrix* projection = 0;
    osg::RefMatrix* modelview = 0;

    // 根据 Camera 的参考框架类型处理矩阵
    if(camera.getReferenceFrame() == osg::Transform::RELATIVE_RF)
    {
        // 相对参考框架：将 Camera 矩阵与当前矩阵相乘
        if(camera.getTransformOrder() == osg::Camera::POST_MULTIPLY)
        {
            projection = createOrReuseMatrix(*getProjectionMatrix() *
                                             camera.getProjectionMatrix());
            modelview = createOrReuseMatrix(*getModelViewMatrix() * camera.getViewMatrix());
        }
        else // 前置乘法
        {
            projection = createOrReuseMatrix(camera.getProjectionMatrix() *
                                             (*getProjectionMatrix()));
            modelview = createOrReuseMatrix(camera.getViewMatrix() *
                                            (*getModelViewMatrix()));
        }
    }
    else
    {
        // 绝对参考框架：直接使用 Camera 的矩阵，忽略当前矩阵
        projection = createOrReuseMatrix(camera.getProjectionMatrix());
        modelview = createOrReuseMatrix(camera.getViewMatrix());
    }

    // **********************************************************************
    // 推送视口，保存并重置近远平面值

    if(camera.getViewport()) pushViewport(camera.getViewport());

    // 记录当前的近远平面值
    value_type previous_znear = _computed_znear;
    value_type previous_zfar = _computed_zfar;

    // 保存当前近平面和远平面候选映射，并清空以便重新计算
    DistanceMatrixDrawableMap  previousNearPlaneCandidateMap;
    previousNearPlaneCandidateMap.swap(_nearPlaneCandidateMap);

    DistanceMatrixDrawableMap  previousFarPlaneCandidateMap;
    previousFarPlaneCandidateMap.swap(_farPlaneCandidateMap);

    _computed_znear = FLT_MAX;
    _computed_zfar = -FLT_MAX;

    // 推送投影和模型视图矩阵到栈
    pushProjectionMatrix(projection);
    pushModelViewMatrix(modelview, camera.getReferenceFrame());

    // **********************************************************************
    // 扩展部分：为 Camera 创建自定义渲染阶段

    if(camera.getRenderOrder() == osg::Camera::NESTED_RENDER)
    {
        // 嵌套渲染：不创建独立渲染阶段，直接遍历子节点
        handle_cull_callbacks_and_traverse(camera);
    }
    else
    {
        // 获取当前渲染阶段作为父阶段
        osgUtil::RenderStage* prevRenderStage = getCurrentRenderBin()->getStage();
        // 从 Camera 的渲染缓存中获取或创建 RenderStageCacheEx
        osg::ref_ptr<RenderStageCacheEx> rsCache = dynamic_cast<RenderStageCacheEx*>
                                                   (camera.getRenderingCache());

        if(!rsCache)
        {
            rsCache = new RenderStageCacheEx();
            camera.setRenderingCache(rsCache);
        }

        // 获取当前裁剪访问器对应的渲染阶段
        osg::ref_ptr<osgUtil::RenderStage> rtts = rsCache->getRenderStage(this);

        if(!rtts)
        {
            // 创建新的 RenderStageEx 实例
            OpenThreads::ScopedLock<OpenThreads::Mutex> lock(*
                                                             (camera.getDataChangeMutex()));

            rtts = new RenderStageEx();
            rsCache->setRenderStage(this, rtts.get());

            rtts->setCamera(&camera);

            // 设置绘制缓冲区：如果继承掩码设置了 DRAW_BUFFER 则从父阶段继承
            if(camera.getInheritanceMask() & DRAW_BUFFER)
            {
                rtts->setDrawBuffer(prevRenderStage->getDrawBuffer(),
                                    prevRenderStage->getDrawBufferApplyMask());
            }
            else
            {
                rtts->setDrawBuffer(camera.getDrawBuffer());
            }

            // 设置读取缓冲区：如果继承掩码设置了 READ_BUFFER 则从父阶段继承
            if(camera.getInheritanceMask() & READ_BUFFER)
            {
                rtts->setReadBuffer(prevRenderStage->getReadBuffer(),
                                    prevRenderStage->getReadBufferApplyMask());
            }
            else
            {
                rtts->setReadBuffer(camera.getReadBuffer());
            }
        }
        else
        {
            // 复用已有的渲染阶段，先重置清除上一帧的内容
            rtts->reset();
        }

        // **********************************************************************
        // 配置渲染阶段的渲染状态

        // 清除设置：深度、累积缓冲、模板缓冲
        rtts->setClearDepth(camera.getClearDepth());
        rtts->setClearAccum(camera.getClearAccum());
        rtts->setClearStencil(camera.getClearStencil());
        // 清除掩码和清除颜色：根据继承掩码决定从 Camera 还是父阶段获取
        rtts->setClearMask((camera.getInheritanceMask() & CLEAR_MASK) ?
                           prevRenderStage->getClearMask() : camera.getClearMask());
        rtts->setClearColor((camera.getInheritanceMask() & CLEAR_COLOR) ?
                            prevRenderStage->getClearColor() : camera.getClearColor());

        // 颜色掩码
        osg::ColorMask* colorMask = camera.getColorMask() != 0 ? camera.getColorMask() :
                                    prevRenderStage->getColorMask();
        rtts->setColorMask(colorMask);

        // 视口设置
        osg::Viewport* viewport = camera.getViewport() != 0 ? camera.getViewport() :
                                  prevRenderStage->getViewport();
        rtts->setViewport(viewport);

        // 设置初始视图矩阵
        rtts->setInitialViewMatrix(modelview);

        // 继承父阶段的位置状态容器（用于光照、裁剪平面等位置相关状态）
        osg::Matrix inheritedMVtolocalMV;
        inheritedMVtolocalMV.invert(originalModelView);
        inheritedMVtolocalMV.postMult(*getModelViewMatrix());
        rtts->setInheritedPositionalStateContainerMatrix(inheritedMVtolocalMV);
        rtts->setInheritedPositionalStateContainer(
            prevRenderStage->getPositionalStateContainer());

        // 记录当前渲染 bin，切换到新创建的渲染阶段后执行子节点遍历
        osgUtil::RenderBin* previousRenderBin = getCurrentRenderBin();
        setCurrentRenderBin(rtts.get());

        // 遍历子图（执行子节点的裁剪回调并遍历）
        {
            handle_cull_callbacks_and_traverse(camera);
        }

        // 恢复之前的渲染 bin
        setCurrentRenderBin(previousRenderBin);

        // 如果子图所有节点都被小特征裁剪或超出 LOD 范围时，渲染阶段为空
        if(rtts->getStateGraphList().size() == 0
           && rtts->getRenderBinList().size() == 0)
        {
            // 子图已被完全裁剪（空节点）
        }

        // 将渲染阶段添加到当前阶段的依赖列表
        switch(camera.getRenderOrder())
        {
        case osg::Camera::PRE_RENDER:
            // 预渲染阶段：在主渲染之前执行（常用于阴影贴图、反射纹理等）
            getCurrentRenderBin()->getStage()->addPreRenderStage(rtts.get(),
                                                                 camera.getRenderOrderNum());
            break;

        default:
            // 后渲染阶段：在主渲染之后执行（常用于后处理效果）
            getCurrentRenderBin()->getStage()->addPostRenderStage(rtts.get(),
                                                                  camera.getRenderOrderNum());
            break;
        }
    }

    // **********************************************************************
    // 恢复操作

    // 恢复之前的模型视图矩阵
    popModelViewMatrix();

    // 恢复之前的投影矩阵
    popProjectionMatrix();

    // 恢复原始近远平面值
    _computed_znear = previous_znear;
    _computed_zfar = previous_zfar;

    // 恢复近平面和远平面候选映射
    previousNearPlaneCandidateMap.swap(_nearPlaneCandidateMap);
    previousFarPlaneCandidateMap.swap(_farPlaneCandidateMap);

    // 弹出视口
    if(camera.getViewport()) popViewport();

    // 恢复遍历掩码设置
    if(mustSetCullMask) setTraversalMask(savedTraversalMask);

    // 恢复裁剪设置
    setCullSettings(saved_cull_settings);

    // 弹出节点状态集
    if(node_state) popStateSet();
}
