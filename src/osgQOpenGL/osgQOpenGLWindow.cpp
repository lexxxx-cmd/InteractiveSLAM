/**
 * @file osgQOpenGLWindow.cpp
 * @brief 基于 QOpenGLWindow 的 OSG 集成窗口的实现文件
 *
 * 本文件实现了 osgQOpenGLWindow 类的所有方法。
 * 核心功能是在 QOpenGLWindow 中集成 OSG 渲染，并通过 QWidget 包装器
 * 使其可以嵌入到 Qt 的布局系统中。
 *
 * 关键流程：
 * 1. initializeGL -> createRenderer -> setupOSG：创建并初始化 OSG 渲染器
 * 2. paintGL -> frame：驱动 OSG 渲染循环
 * 3. resizeGL -> resize：响应窗口大小变化
 * 4. 事件处理：将所有 Qt 输入事件转发到 OSGRenderer
 */

#include "osgQOpenGLWindow.h"
#include "OSGRenderer.h"

#include <osgViewer/Viewer>
#include <osg/GL>

#include <QApplication>
#include <QKeyEvent>
#include <QInputDialog>
#include <QLayout>
#include <QMainWindow>
#include <QScreen>
#include <QWindow>

/**
 * @brief 构造函数
 * @param parent 父 Qt 部件
 *
 * 以 NoPartialUpdate 模式创建 QOpenGLWindow（禁用部分更新，每次重绘整个窗口）。
 * 同时创建 QWidget 包装器（通过 QWidget::createWindowContainer），
 * 使得此 QWindow 可以像普通 QWidget 一样嵌入到 Qt 布局管理器中。
 */
osgQOpenGLWindow::osgQOpenGLWindow(QWidget* parent)
    : QOpenGLWindow(QOpenGLWindow::NoPartialUpdate, nullptr)
{
    // 创建 QWidget 包装器，使得 OSG 窗口可以嵌入到 Qt 布局中
    _widget = QWidget::createWindowContainer(this);
}

/**
 * @brief 析构函数
 */
osgQOpenGLWindow::~osgQOpenGLWindow()
{
}

/**
 * @brief 获取 OSG 视图器
 * @return osgViewer::Viewer 指针（实际是 OSGRenderer 实例）
 */
osgViewer::Viewer* osgQOpenGLWindow::getOsgViewer()
{
    return m_renderer;
}

/**
 * @brief 获取场景图读写互斥锁
 * @return 读写互斥锁指针
 */
OpenThreads::ReadWriteMutex* osgQOpenGLWindow::mutex()
{
    return &_osgMutex;
}

/**
 * @brief 初始化 OpenGL（重写 QOpenGLWindow）
 *
 * 初始化流程：
 * 1. 解析当前 OpenGL 上下文中的函数（initializeOpenGLFunctions）
 * 2. 创建 OSG 渲染器（createRenderer）
 * 3. 发送 initialized() 信号通知外部
 */
void osgQOpenGLWindow::initializeGL()
{
    initializeOpenGLFunctions();
    createRenderer();
    emit initialized();
}

/**
 * @brief 窗口大小改变时的处理
 * @param w 新宽度（Qt 逻辑像素）
 * @param h 新高度（Qt 逻辑像素）
 *
 * 获取屏幕的像素比（devicePixelRatio）并传递给渲染器，
 * 以支持高 DPI 屏幕的正确渲染。
 */
void osgQOpenGLWindow::resizeGL(int w, int h)
{
    Q_ASSERT(m_renderer);
    double pixelRatio = screen()->devicePixelRatio();
    m_renderer->resize(w, h, pixelRatio);
}

/**
 * @brief 绘制 OpenGL 内容（重写 QOpenGLWindow）
 *
 * 绘制流程：
 * 1. 获取 OSG 场景图的读锁（允许并发读取，防止写入冲突）
 * 2. 第一帧时：获取 Qt 的默认 FBO ID 并设置到 OSG 图形上下文中，
 *    确保 OSG 渲染输出到正确的帧缓冲区
 * 3. 调用 m_renderer->frame() 执行一帧的 OSG 渲染
 */
void osgQOpenGLWindow::paintGL()
{
    OpenThreads::ScopedReadLock locker(_osgMutex);
    if (_isFirstFrame) {
        _isFirstFrame = false;
        // 设置 OSG 的默认 FBO ID 为 Qt 提供的默认帧缓冲区，
        // 确保 OSG 渲染到 Qt 的 OpenGL 表面上
        m_renderer->getCamera()->getGraphicsContext()->setDefaultFboId(
            defaultFramebufferObject());
    }
    m_renderer->frame();
}

/**
 * @brief 键盘按下事件处理
 * @param event Qt 键盘事件
 * 将事件转发到 OSG 渲染器处理。
 */
void osgQOpenGLWindow::keyPressEvent(QKeyEvent* event)
{
    Q_ASSERT(m_renderer);
    m_renderer->keyPressEvent(event);
}

/**
 * @brief 键盘释放事件处理
 * @param event Qt 键盘事件
 * 将事件转发到 OSG 渲染器处理。
 */
void osgQOpenGLWindow::keyReleaseEvent(QKeyEvent* event)
{
    Q_ASSERT(m_renderer);
    m_renderer->keyReleaseEvent(event);
}

/**
 * @brief 鼠标按下事件处理
 * @param event Qt 鼠标事件
 * 将事件转发到 OSG 渲染器处理。
 */
void osgQOpenGLWindow::mousePressEvent(QMouseEvent* event)
{
    Q_ASSERT(m_renderer);
    m_renderer->mousePressEvent(event);
}

/**
 * @brief 鼠标释放事件处理
 * @param event Qt 鼠标事件
 * 将事件转发到 OSG 渲染器处理。
 */
void osgQOpenGLWindow::mouseReleaseEvent(QMouseEvent* event)
{
    Q_ASSERT(m_renderer);
    m_renderer->mouseReleaseEvent(event);
}

/**
 * @brief 鼠标双击事件处理
 * @param event Qt 鼠标事件
 * 将事件转发到 OSG 渲染器处理。
 */
void osgQOpenGLWindow::mouseDoubleClickEvent(QMouseEvent* event)
{
    Q_ASSERT(m_renderer);
    m_renderer->mouseDoubleClickEvent(event);
}

/**
 * @brief 鼠标移动事件处理
 * @param event Qt 鼠标事件
 * 将事件转发到 OSG 渲染器处理。
 */
void osgQOpenGLWindow::mouseMoveEvent(QMouseEvent* event)
{
    Q_ASSERT(m_renderer);
    m_renderer->mouseMoveEvent(event);
}

/**
 * @brief 鼠标滚轮事件处理
 * @param event Qt 滚轮事件
 * 将事件转发到 OSG 渲染器处理。
 */
void osgQOpenGLWindow::wheelEvent(QWheelEvent* event)
{
    Q_ASSERT(m_renderer);
    m_renderer->wheelEvent(event);
}

/**
 * @brief 设置默认显示参数
 *
 * 在创建 OSG 渲染器之前调用。
 * 配置 OSG 的全局显示设置：
 * - 启用 Nvidia Optimus（双显卡切换支持）
 * - 禁用立体渲染模式
 */
void osgQOpenGLWindow::setDefaultDisplaySettings()
{
    osg::DisplaySettings* ds = osg::DisplaySettings::instance().get();
    ds->setNvOptimusEnablement(1);
    ds->setStereo(false);
}

/**
 * @brief 创建 OSG 渲染器
 *
 * 创建流程：
 * 1. 调用 setDefaultDisplaySettings() 配置全局显示参数
 * 2. 创建 OSGRenderer 实例，将当前窗口设为其父对象
 * 3. 获取当前屏幕的像素比
 * 4. 调用 renderer->setupOSG() 完成 OSG 系统初始化
 *
 * 注意：与 osgQOpenGLWidget 不同，osgQOpenGLWindow 不支持
 * 通过 ArgumentParser 定制 OSG 渲染器。
 */
void osgQOpenGLWindow::createRenderer()
{
    // 在创建 OSG 视图器之前设置默认显示参数
    setDefaultDisplaySettings();

    // 创建渲染器实例，将当前窗口设为其父对象
    m_renderer = new OSGRenderer(this);

    // 获取屏幕像素比（用于高 DPI 缩放）
    double pixelRatio = screen()->devicePixelRatio();
    m_renderer->setupOSG(width(), height(), pixelRatio);
}
