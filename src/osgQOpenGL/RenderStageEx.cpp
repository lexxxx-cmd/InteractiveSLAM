/**
 * @file RenderStageEx.cpp
 * @brief 自定义渲染阶段扩展类的实现文件
 *
 * 本文件实现了 RenderStageEx::drawInner 方法，扩展了 OSG 标准的渲染阶段绘制流程。
 * 核心功能是在绘制前将帧缓冲区绑定到 Qt 的默认 FBO，确保 OSG 场景渲染到正确的目标上。
 * 同时保留了 OSG 标准渲染阶段中的纹理拷贝、FBO 离屏渲染、mipmap 生成等完整功能。
 *
 * 注意：当前实现中所有功能代码被包含在 #if 0 块中（已禁用），
 * 仅作为参考代码保留。实际运行时使用的是父类 RenderStage 的默认 drawInner 实现。
 * 如果要启用此扩展实现，需要将 #if 0 改为 #if 1。
 *
 * 参考: http://forum.openscenegraph.org/viewtopic.php?t=15627&view=previous
 */

#include "RenderStageEx.h"
#include "StateEx.h"

/**
 * @brief 重写内部绘制方法，支持 OSG 与 Qt 混合渲染
 *
 * 此扩展方法的核心差异在于：
 * 当未使用自定义 FBO 时，通过 StateEx 获取 Qt 窗口的默认 FBO ID，
 * 并将渲染绑定到该默认 FBO，从而使得 OSG 内容直接渲染到 Qt 表面。
 * 其余逻辑（多渲染目标、抗锯齿消除、纹理拷贝、像素读取、mipmap 生成等）
 * 与 OSG 标准 RenderStage::drawInner 实现一致。
 *
 * @param renderInfo   渲染信息对象，包含当前状态和图形上下文
 * @param previous     上一个渲染叶节点引用，用于绘制顺序管理
 * @param doCopyTexture 是否执行纹理拷贝的标志引用
 */
void RenderStageEx::drawInner(osg::RenderInfo& renderInfo,
                              osgUtil::RenderLeaf*& previous, bool& doCopyTexture)
{
#if 0
    // **********************************************************************
    // 以下代码继承自 OSG RenderStage 类的 drawInner 实现，并进行了扩展

    // 内部辅助子函数：将读取帧缓冲区绑定为单采样 FBO
    struct SubFunc
    {
        /** @brief 应用读取 FBO 绑定 */
        static void applyReadFBO(bool& apply_read_fbo,
                                 const osg::FrameBufferObject* read_fbo, osg::State& state)
        {
            if(read_fbo->isMultisample())
            {
                OSG_WARN << "Attempting to read from a"
                         " multisampled framebuffer object. Set a resolve"
                         " framebuffer on the RenderStage to fix this." << std::endl;
            }

            if(apply_read_fbo)
            {
                // 绑定单采样 FBO 用于读取
                read_fbo->apply(state, osg::FrameBufferObject::READ_FRAMEBUFFER);
                apply_read_fbo = false;
            }
        }
    };

    // **********************************************************************
    // 扩展部分：获取状态和 FBO 扩展，并绑定到 Qt 默认 FBO

    osg::State& state = *renderInfo.getState();
    osg::GLExtensions* fbo_ext = state.get<osg::GLExtensions>();
    bool using_multiple_render_targets = false;

    if(fbo_ext)
    {
        if(_fbo.valid())
        {
            // 检查是否使用了多渲染目标（MRT）
            using_multiple_render_targets = _fbo->hasMultipleRenderingTargets();

            if(!using_multiple_render_targets)
            {
#if !defined(OSG_GLES1_AVAILABLE) && !defined(OSG_GLES2_AVAILABLE)

                // 设置绘制缓冲区和读取缓冲区
                if(getDrawBufferApplyMask())
                    glDrawBuffer(_drawBuffer);

                if(getReadBufferApplyMask())
                    glReadBuffer(_readBuffer);

#endif
            }
        }
        else
        {
            // 关键扩展：当没有自定义 FBO 时，绑定到 Qt 的默认帧缓冲区
            // 这使得 OSG 内容直接渲染到 QOpenGLWidget/QOpenGLWindow 的表面
            fbo_ext->glBindFramebuffer(osg::FrameBufferObject::READ_DRAW_FRAMEBUFFER,
                                       static_cast<StateEx*>(&state)->getDefaultFbo());
        }
    }

    // 执行实际的渲染 bin 绘制
    RenderBin::draw(renderInfo, previous);

    // **********************************************************************
    // 以下为 OSG 标准 RenderStage::drawInner 的后续处理

    // 检查 OpenGL 错误
    if(state.getCheckForGLErrors() != osg::State::NEVER_CHECK_GL_ERRORS)
    {
        if(state.checkGLErrors("after RenderBin::draw(..)"))
        {
            if(fbo_ext)
            {
                GLenum fbstatus = fbo_ext->glCheckFramebufferStatus(GL_FRAMEBUFFER_EXT);

                if(fbstatus != GL_FRAMEBUFFER_COMPLETE_EXT)
                {
                    OSG_NOTICE << "RenderStage::drawInner(,) FBO status = 0x" << std::hex <<
                               fbstatus << std::dec << std::endl;
                }
            }
        }
    }

    // 处理抗锯齿消除（resolve）FBO
    const osg::FrameBufferObject* read_fbo = fbo_ext ? _fbo.get() : 0;
    bool apply_read_fbo = false;

    if(fbo_ext && _resolveFbo.valid() && fbo_ext->glBlitFramebuffer)
    {
        GLbitfield blitMask = 0;
        bool needToBlitColorBuffers = false;

        // 确定需要拷贝的缓冲区类型（深度、模板、颜色等）
        for(osg::FrameBufferObject::AttachmentMap::const_iterator
            it = _resolveFbo->getAttachmentMap().begin(),
            end = _resolveFbo->getAttachmentMap().end(); it != end; ++it)
        {
            switch(it->first)
            {
            case osg::Camera::DEPTH_BUFFER:
                blitMask |= GL_DEPTH_BUFFER_BIT;
                break;

            case osg::Camera::STENCIL_BUFFER:
                blitMask |= GL_STENCIL_BUFFER_BIT;
                break;

            case osg::Camera::PACKED_DEPTH_STENCIL_BUFFER:
                blitMask |= GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT;
                break;

            case osg::Camera::COLOR_BUFFER:
                blitMask |= GL_COLOR_BUFFER_BIT;
                break;

            default:
                needToBlitColorBuffers = true;
                break;
            }
        }

        // 绑定源 FBO 和目标 FBO 以执行 blit 操作
        _fbo->apply(state, osg::FrameBufferObject::READ_FRAMEBUFFER);
        _resolveFbo->apply(state, osg::FrameBufferObject::DRAW_FRAMEBUFFER);

        if(blitMask)
        {
            // 执行 blit 操作将数据从多采样 FBO 拷贝到单采样 FBO
            // 注意：在 Nvidia 175.16 Windows 驱动下，如果读取 FBO 是多采样的，
            // 则尺寸参数会被忽略，始终拷贝整个帧缓冲区
            fbo_ext->glBlitFramebuffer(
                static_cast<GLint>(_viewport->x()), static_cast<GLint>(_viewport->y()),
                static_cast<GLint>(_viewport->x() + _viewport->width()),
                static_cast<GLint>(_viewport->y() + _viewport->height()),
                static_cast<GLint>(_viewport->x()), static_cast<GLint>(_viewport->y()),
                static_cast<GLint>(_viewport->x() + _viewport->width()),
                static_cast<GLint>(_viewport->y() + _viewport->height()),
                blitMask, GL_NEAREST);
        }

#if !defined(OSG_GLES1_AVAILABLE) && !defined(OSG_GLES2_AVAILABLE)
        // 处理多颜色缓冲区附件的情况
        if(needToBlitColorBuffers)
        {
            for(osg::FrameBufferObject::AttachmentMap::const_iterator
                it = _resolveFbo->getAttachmentMap().begin(),
                end = _resolveFbo->getAttachmentMap().end(); it != end; ++it)
            {
                osg::Camera::BufferComponent attachment = it->first;

                if(attachment >= osg::Camera::COLOR_BUFFER0)
                {
                    glReadBuffer(GL_COLOR_ATTACHMENT0_EXT + (attachment -
                                                             osg::Camera::COLOR_BUFFER0));
                    glDrawBuffer(GL_COLOR_ATTACHMENT0_EXT + (attachment -
                                                             osg::Camera::COLOR_BUFFER0));
                    fbo_ext->glBlitFramebuffer(
                        static_cast<GLint>(_viewport->x()), static_cast<GLint>(_viewport->y()),
                        static_cast<GLint>(_viewport->x() + _viewport->width()),
                        static_cast<GLint>(_viewport->y() + _viewport->height()),
                        static_cast<GLint>(_viewport->x()), static_cast<GLint>(_viewport->y()),
                        static_cast<GLint>(_viewport->x() + _viewport->width()),
                        static_cast<GLint>(_viewport->y() + _viewport->height()),
                        GL_COLOR_BUFFER_BIT, GL_NEAREST);
                }
            }
        }
#endif
        apply_read_fbo = true;
        read_fbo = _resolveFbo.get();
        using_multiple_render_targets = read_fbo->hasMultipleRenderingTargets();
    }

    // 将渲染结果拷贝到关联纹理
    if(doCopyTexture)
    {
        if(read_fbo) SubFunc::applyReadFBO(apply_read_fbo, read_fbo, state);
        copyTexture(renderInfo);
    }

    // 处理缓冲区附件图像的像素读取（用于离屏渲染后读取像素数据）
    std::map< osg::Camera::BufferComponent, Attachment>::const_iterator itr;
    for(itr = _bufferAttachmentMap.begin();
        itr != _bufferAttachmentMap.end();
        ++itr)
    {
        if(itr->second._image.valid())
        {
            if(read_fbo) SubFunc::applyReadFBO(apply_read_fbo, read_fbo, state);

#if !defined(OSG_GLES1_AVAILABLE) && !defined(OSG_GLES2_AVAILABLE)
            if(using_multiple_render_targets)
            {
                int attachment = itr->first;

                if(attachment == osg::Camera::DEPTH_BUFFER
                   || attachment == osg::Camera::STENCIL_BUFFER)
                {
                    // 对于深度/模板缓冲区，假设使用第一个渲染目标
                    glReadBuffer(read_fbo->getMultipleRenderingTargets()[0]);
                }
                else
                {
                    glReadBuffer(GL_COLOR_ATTACHMENT0_EXT + (attachment -
                                                             osg::Camera::COLOR_BUFFER0));
                }
            }
            else
            {
                if(_readBuffer != GL_NONE)
                {
                    glReadBuffer(_readBuffer);
                }
            }
#endif
            // 读取像素数据到图像对象
            GLenum pixelFormat = itr->second._image->getPixelFormat();
            if(pixelFormat == 0) pixelFormat = _imageReadPixelFormat;
            if(pixelFormat == 0) pixelFormat = GL_RGB;

            GLenum dataType = itr->second._image->getDataType();
            if(dataType == 0) dataType = _imageReadPixelDataType;
            if(dataType == 0) dataType = GL_UNSIGNED_BYTE;

            itr->second._image->readPixels(static_cast<int>(_viewport->x()),
                                           static_cast<int>(_viewport->y()),
                                           static_cast<int>(_viewport->width()),
                                           static_cast<int>(_viewport->height()),
                                           pixelFormat, dataType);
        }
    }

    // 渲染完成后，可选择关闭 FBO，恢复到系统默认帧缓冲区
    if(fbo_ext)
    {
        if(getDisableFboAfterRender())
        {
            GLuint fboId = state.getGraphicsContext() ?
                           state.getGraphicsContext()->getDefaultFboId() : 0;
            fbo_ext->glBindFramebuffer(GL_FRAMEBUFFER_EXT, fboId);
        }

        doCopyTexture = true;
    }

    // 如果需要，生成 mipmap 纹理层级
    if(fbo_ext && _camera.valid())
    {
        const osg::Camera::BufferAttachmentMap& bufferAttachments =
            _camera->getBufferAttachmentMap();

        for(osg::Camera::BufferAttachmentMap::const_iterator itr =
                bufferAttachments.begin();
            itr != bufferAttachments.end();
            ++itr)
        {
            if(itr->second._texture.valid() && itr->second._mipMapGeneration)
            {
                state.setActiveTextureUnit(0);
                state.applyTextureAttribute(0, itr->second._texture.get());
                fbo_ext->glGenerateMipmap(itr->second._texture->getTextureTarget());
            }
        }
    }

#endif  // #if 0 — 当前禁用此扩展实现，使用父类默认实现
}
