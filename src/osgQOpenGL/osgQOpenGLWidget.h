/**
 * @file osgQOpenGLWidget.h
 * @brief 基于 QOpenGLWidget 的 OSG 集成部件的头文件
 *
 * 本文件定义了 osgQOpenGLWidget 类，继承自 QOpenGLWidget 和 QOpenGLFunctions，
 * 是实现 OSG（OpenSceneGraph）与 Qt 集成的核心部件之一。
 *
 * 主要功能：
 * 1. 提供一个 Qt 部件，将 OSG 场景渲染嵌入到 QOpenGLWidget 的 OpenGL 表面中
 * 2. 管理 OSGRenderer 渲染器的生命周期（创建、初始化、销毁）
 * 3. 将 Qt 的输入事件（键盘、鼠标、滚轮）转发到 OSG 渲染器
 * 4. 支持全屏模式切换（按 F 键）
 * 5. 管理线程安全的读写互斥锁，保护 OSG 场景图的并发访问
 * 6. 处理高 DPI 屏幕的缩放
 *
 * 使用 osgQOpenGLWidget 还是 osgQOpenGLWindow 的选择：
 * - osgQOpenGLWidget：作为 QWidget 嵌入到现有 Qt 布局中，适合需要与其他控件混排的场景
 * - osgQOpenGLWindow：作为独立 QWindow，性能更好，适合全屏或独占窗口的场景
 */

#ifndef OSGQOPENGLWIDGET_H
#define OSGQOPENGLWIDGET_H

#ifdef __APPLE__
#   define __glext_h_
#   include <QtGui/qopengl.h>
#   undef __glext_h_
#   include <QtGui/qopenglext.h>
#endif

#include "Export"
#include <OpenThreads/ReadWriteMutex>

#ifdef WIN32
//#define __gl_h_
#include <osg/GL>
#endif

#include <osg/ArgumentParser>

#include <QtOpenGLWidgets/QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QReadWriteLock>

class OSGRenderer;

namespace osgViewer
{
    class Viewer;
}

/**
 * @class osgQOpenGLWidget
 * @brief 基于 QOpenGLWidget 的 OSG 集成部件
 *
 * 继承自 QOpenGLWidget，将 OSG 渲染集成到 Qt 的 OpenGL 部件框架中。
 * 提供完整的 OSG 场景显示功能，同时支持 Qt 的事件系统和布局管理。
 *
 * 核心职责：
 * - 创建和管理 OSGRenderer 实例
 * - 在 initializeGL 中初始化 OpenGL 函数和 OSG 渲染器
 * - 在 paintGL 中执行 OSG 帧更新
 * - 将所有输入事件转发给 OSGRenderer 处理
 * - 通过信号 initialized() 通知外部初始化完成
 * - 按 F 键切换到全屏模式（支持多屏选择）
 *
 * 线程安全：
 * - 使用 OpenThreads::ReadWriteMutex 保护 OSG 场景图的访问
 * - paintGL 中获取读锁，允许并发的场景图读取，防止写入冲突
 */
class osgQOpenGLWidget : public QOpenGLWidget,
    protected QOpenGLFunctions
{
    Q_OBJECT

protected:
    /** @brief OSG 渲染器指针 */
    OSGRenderer* m_renderer {nullptr};

    /** @brief 标志：OSG 是否需要渲染下一帧 */
    bool _osgWantsToRenderFrame{true};

    /** @brief OSG 场景图的读写互斥锁，用于线程安全访问 */
    OpenThreads::ReadWriteMutex _osgMutex;

    /** @brief 可选的 OSG 命令行参数解析器 */
    osg::ArgumentParser* _arguments {nullptr};

    /** @brief 是否为第一帧（用于首次设置默认 FBO） */
    bool _isFirstFrame {true};

    friend class OSGRenderer;

public:
    /**
     * @brief 构造函数
     * @param parent 父 Qt 部件
     */
    osgQOpenGLWidget(QWidget* parent = nullptr);

    /**
     * @brief 构造函数（带参数解析器）
     * @param arguments OSG 命令行参数解析器
     * @param parent    父 Qt 部件
     */
    osgQOpenGLWidget(osg::ArgumentParser* arguments, QWidget* parent = nullptr);

    /** @brief 析构函数 */
    virtual ~osgQOpenGLWidget();

    /**
     * @brief 获取 OSG 视图器（Viewer）
     * @return osgViewer::Viewer 指针
     */
    virtual osgViewer::Viewer* getOsgViewer();

    /**
     * @brief 获取场景图互斥锁
     * @return 读写互斥锁指针
     */
    virtual OpenThreads::ReadWriteMutex* mutex();

    /**
     * @brief 默认大小提示
     * @return QSize(640, 480)
     */
    QSize sizeHint() const override { return QSize(640,480); }

signals:
    /** @brief 初始化完成信号 */
    void initialized();

protected:
    /**
     * @brief 初始化 OpenGL（重写 QOpenGLWidget）
     *
     * 初始化 OpenGL 函数、创建渲染器、发送 initialized 信号。
     */
    void initializeGL() override;

    /**
     * @brief 窗口大小改变时调整渲染尺寸
     * @param w 新宽度（Qt 逻辑像素）
     * @param h 新高度（Qt 逻辑像素）
     */
    void resizeGL(int w, int h) override;

    /**
     * @brief 绘制 OpenGL 内容（重写 QOpenGLWidget）
     *
     * 获取读锁后执行 OSG 帧更新。
     * 第一帧时设置默认 FBO ID 以确保渲染到正确的帧缓冲区。
     */
    void paintGL() override;

    /**
     * @brief 设置默认显示参数（创建渲染器前调用）
     *
     * 配置 OSG 的显示设置，如启用 Optimus、禁用立体渲染等。
     */
    virtual void setDefaultDisplaySettings();

    // 键盘事件转发
    void keyPressEvent(QKeyEvent* event) override;
    void keyReleaseEvent(QKeyEvent* event) override;

    // 鼠标事件转发
    void mousePressEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;
    void mouseDoubleClickEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;

    /**
     * @brief 事件处理（重写 QWidget）
     *
     * 捕获设备像素比变化与屏幕内部变化事件，主动重同步 OSG 视口，
     * 避免最大化/还原或跨屏移动后视口与 FBO 尺寸不匹配。
     */
    bool event(QEvent* event) override;

    /**
     * @brief 创建渲染器
     *
     * 设置默认显示设置，创建 OSGRenderer 实例，
     * 调用 setupOSG 完成渲染器初始化。
     */
    void createRenderer();

private:
};

#endif // OSGQOPENGLWIDGET_H
