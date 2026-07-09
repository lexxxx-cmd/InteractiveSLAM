/**
 * @file OSGRenderer.h
 * @brief OSG 渲染器的头文件
 *
 * 本文件定义了 OSGRenderer 类，继承自 QObject 和 osgViewer::Viewer，
 * 作为 OSG 与 Qt 集成的核心渲染管理类。
 *
 * 主要功能：
 * 1. 管理 OSG 场景的初始化、帧更新和渲染循环
 * 2. 将 Qt 输入事件（键盘、鼠标、滚轮）转换为 OSG 事件
 * 3. 使用 QTimer 驱动 OSG 的帧更新
 * 4. 管理嵌入式图形窗口（GraphicsWindowEmbedded）和视口
 * 5. 支持按需渲染（ON_DEMAND）和持续渲染两种模式
 * 6. 帧率限制，避免过度占用 CPU
 */


// Copyright (C) 2017 Mike Krus
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License as
// published by the Free Software Foundation; either version 2 of the
// License, or (at your option) any later version.
//
// This program is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.

#ifndef OSGRENDERER_H
#define OSGRENDERER_H

#include "Export"

#include <QObject>

#include <osgViewer/Viewer>

class QInputEvent;
class QKeyEvent;
class QMouseEvent;
class QWheelEvent;
namespace eveBIM
{
    class ViewerWidget;
}

/**
 * @class OSGRenderer
 * @brief OSG 渲染器类，继承自 QObject 和 osgViewer::Viewer
 *
 * 此类是 OSG 与 Qt 集成的核心桥梁。它同时继承 QObject（Qt 对象模型）
 * 和 osgViewer::Viewer（OSG 视图控制器），实现以下功能：
 *
 * - 渲染管理：初始化 OSG 场景、配置相机和视口、驱动帧循环
 * - 事件处理：将 Qt 的输入事件转换为 OSG 标准事件并发送到事件队列
 * - 帧率控制：通过 QTimer 驱动帧更新，支持最大帧率限制
 * - 按需渲染：仅在场景需要更新时渲染，降低空闲时的 CPU 占用
 * - Qt 集成：与 osgQOpenGLWidget 或 osgQOpenGLWindow 配合使用
 *
 * 使用方式：
 * 1. 创建 OSGRenderer 实例，将 QOpenGLWidget/QOpenGLWindow 设为父对象
 * 2. 调用 setupOSG() 初始化 OSG 场景
 * 3. 通过 Qt 事件系统自动接收输入事件
 * 4. 通过 QTimer 定时器自动驱动帧更新
 */
class OSGRenderer : public QObject, public osgViewer::Viewer
{
    /** @brief OSG 是否已初始化 */
    bool                                       m_osgInitialized {false};

    /** @brief 嵌入式 OSG 图形窗口，嵌入到 Qt 的 OpenGL 表面中 */
    osg::ref_ptr<osgViewer::GraphicsWindow>    m_osgWinEmb;

    /** @brief 窗口缩放因子（用于高 DPI 屏幕，通常为 devicePixelRatio） */
    float                                      m_windowScale {1.0f};

    /** @brief 是否持续更新渲染（true=持续渲染，false=按需渲染） */
    bool                                       m_continuousUpdate {true};

    /** @brief Qt 定时器 ID，用于驱动帧循环 */
    int                                        _timerId{0};

    /** @brief 上一帧的开始时间，用于帧率计算 */
    osg::Timer                                 _lastFrameStartTime;

    /** @brief 应用是否即将退出 */
    bool                                       _applicationAboutToQuit {false};

    /** @brief OSG 是否请求渲染下一帧 */
    bool                                       _osgWantsToRenderFrame{true};

    Q_OBJECT

    friend class eveBIM::ViewerWidget;

public:

    /**
     * @brief 构造函数（无参数解析器）
     * @param parent 父 QObject 对象，通常为 QOpenGLWidget/QOpenGLWindow
     */
    explicit OSGRenderer(QObject* parent = nullptr);

    /**
     * @brief 构造函数（带参数解析器）
     * @param arguments OSG 命令行参数解析器
     * @param parent    父 QObject 对象
     */
    explicit OSGRenderer(osg::ArgumentParser* arguments, QObject* parent = nullptr);

    /** @brief 析构函数 */
    ~OSGRenderer() override;

    /** @brief 获取持续更新模式状态 */
    bool continuousUpdate() const { return m_continuousUpdate; }

    /** @brief 设置持续更新模式 */
    void setContinuousUpdate(bool continuousUpdate) { m_continuousUpdate = continuousUpdate; }

    // Qt 输入事件处理（将 Qt 事件转发到 OSG 事件队列）
    virtual void keyPressEvent(QKeyEvent* event);
    virtual void keyReleaseEvent(QKeyEvent* event);
    virtual void mousePressEvent(QMouseEvent* event);
    virtual void mouseReleaseEvent(QMouseEvent* event);
    virtual void mouseDoubleClickEvent(QMouseEvent* event);
    virtual void mouseMoveEvent(QMouseEvent* event);
    virtual void wheelEvent(QWheelEvent* event);

    /**
     * @brief 调整窗口大小
     * @param windowWidth  窗口宽度（Qt 像素）
     * @param windowHeight 窗口高度（Qt 像素）
     * @param windowScale  缩放因子（高 DPI 缩放）
     *
     * 更新 OSG 视口和图形窗口尺寸，响应 Qt 的 resize 事件。
     */
    virtual void resize(int windowWidth, int windowHeight, float windowScale);

    /**
     * @brief 初始化 OSG 渲染系统
     * @param windowWidth  窗口宽度
     * @param windowHeight 窗口高度
     * @param windowScale  缩放因子
     *
     * 创建嵌入式图形窗口、配置相机视口、设置线程模型为单线程、
     * 禁用 Escape 退出快捷键、启动 Qt 定时器驱动帧循环。
     */
    void setupOSG(int windowWidth, int windowHeight, float windowScale);

    /** @brief 重写 osgViewer::Viewer 的检查是否需要渲染帧的方法 */
    virtual bool checkNeedToDoFrame() override;

    /** @brief 重写 osgViewer::ViewerBase 的帧更新方法 */
    void frame(double simulationTime = USE_REFERENCE_TIME) override;

    /** @brief 重写 osgViewer::Viewer 的请求重绘方法 */
    void requestRedraw() override;

    /** @brief 重写 osgViewer::Viewer 的事件检查方法 */
    bool checkEvents() override;

    /**
     * @brief 请求渲染一帧（通知父 Widget/Window 更新）
     *
     * 获取父对象，如果是 osgQOpenGLWindow 或 osgQOpenGLWidget，
     * 则设置 _osgWantsToRenderFrame 标志并调用 update() 触发重绘。
     */
    void update();

protected:
    /**
     * @brief Qt 定时器事件处理
     * @param event 定时器事件
     *
     * 在持续渲染模式下或场景需要更新时调用 update() 触发渲染。
     */
    void timerEvent(QTimerEvent* event) override;

    /**
     * @brief 设置键盘修饰键（Shift/Ctrl/Alt）到 OSG 事件状态
     * @param event Qt 输入事件
     */
    void setKeyboardModifiers(QInputEvent* event);
};

#endif // OSGRENDERER_H
