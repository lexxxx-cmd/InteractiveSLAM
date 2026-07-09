/**
 * @file osgQOpenGLWidget.cpp
 * @brief 基于 QOpenGLWidget 的 OSG 集成部件的实现文件
 *
 * 本文件实现了 osgQOpenGLWidget 类的所有方法。
 * 核心功能是将 OSG 渲染嵌入到 Qt 的 QOpenGLWidget 中，实现 Qt 与 OSG 的深度集成。
 *
 * 关键流程：
 * 1. initializeGL -> createRenderer -> setupOSG：创建并初始化 OSG 渲染器
 * 2. paintGL -> frame：驱动 OSG 渲染循环
 * 3. resizeGL -> resize：响应窗口大小变化
 * 4. 事件处理：将所有 Qt 输入事件转发到 OSGRenderer
 * 5. F 键全屏切换：支持多屏幕选择的全屏/窗口模式切换
 */

#include "osgQOpenGLWidget.h"
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
 * @brief 默认构造函数
 * @param parent 父 Qt 部件
 */
osgQOpenGLWidget::osgQOpenGLWidget(QWidget* parent)
    : QOpenGLWidget(parent)
{
}

/**
 * @brief 带参数解析器的构造函数
 * @param arguments OSG 命令行参数解析器
 * @param parent    父 Qt 部件
 */
osgQOpenGLWidget::osgQOpenGLWidget(osg::ArgumentParser* arguments,
                                   QWidget* parent) :
    QOpenGLWidget(parent),
    _arguments(arguments)
{
}

/**
 * @brief 析构函数
 */
osgQOpenGLWidget::~osgQOpenGLWidget()
{
}

/**
 * @brief 获取 OSG 视图器
 * @return osgViewer::Viewer 指针（实际是 OSGRenderer 实例）
 */
osgViewer::Viewer* osgQOpenGLWidget::getOsgViewer()
{
    return m_renderer;
}

/**
 * @brief 获取场景图读写互斥锁
 * @return 读写互斥锁指针
 */
OpenThreads::ReadWriteMutex* osgQOpenGLWidget::mutex()
{
    return &_osgMutex;
}

/**
 * @brief 初始化 OpenGL（重写 QOpenGLWidget）
 *
 * 初始化流程：
 * 1. 解析当前 OpenGL 上下文中的函数（initializeOpenGLFunctions）
 * 2. 创建 OSG 渲染器（createRenderer）
 * 3. 发送 initialized() 信号通知外部
 */
void osgQOpenGLWidget::initializeGL()
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
void osgQOpenGLWidget::resizeGL(int w, int h)
{
    Q_ASSERT(m_renderer);
    QScreen* screen = windowHandle()
                      && windowHandle()->screen() ? windowHandle()->screen() :
                      qApp->screens().front();
    m_renderer->resize(w, h, screen->devicePixelRatio());
}

/**
 * @brief 绘制 OpenGL 内容（重写 QOpenGLWidget）
 *
 * 绘制流程：
 * 1. 获取 OSG 场景图的读锁（允许并发读取，防止写入冲突）
 * 2. 第一帧时：获取 Qt 的默认 FBO ID 并设置到 OSG 图形上下文中，
 *    确保 OSG 渲染输出到正确的帧缓冲区
 * 3. 调用 m_renderer->frame() 执行一帧的 OSG 渲染
 */
void osgQOpenGLWidget::paintGL()
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
 *
 * 特殊处理：
 * - 按 F 键：在全屏和窗口模式之间切换
 *   - 如果父部件是 QMainWindow，进入全屏模式
 *     - 多屏环境下弹出屏幕选择对话框
 *     - 设置 1px 边框
 *   - 如果已经全屏，则恢复到窗口模式
 * - 其他键：转发到 OSG 渲染器，同时 event->ignore() 允许事件继续传播，
 *   确保 Qt 的快捷键（如 Ctrl+O、Ctrl+S 等）能正常触发
 */
void osgQOpenGLWidget::keyPressEvent(QKeyEvent* event)
{
    Q_ASSERT(m_renderer);

    if(event->key() == Qt::Key_F)
    {
        // 保存当前大小和边距，用于从全屏恢复
        static QSize g;
        static QMargins sMargins;

        if(parent() && parent()->isWidgetType())
        {
            // 进入全屏模式
            QMainWindow* _mainwindow = dynamic_cast<QMainWindow*>(parent());

            if(_mainwindow)
            {
                // 保存当前尺寸和边距以便恢复
                g = size();
                if(layout())
                    sMargins = layout()->contentsMargins();

                bool ok = true;

                // 多屏环境：弹出屏幕选择对话框
                if(qApp->screens().size() > 1)
                {
                    // 构建屏幕选择列表
                    QMap<QString, QScreen*> screens;
                    int screenNumber = 0;

                    for(QScreen* screen : qApp->screens())
                    {
                        QString name = screen->name();

                        if(name.isEmpty())
                        {
                            name = tr("Screen %1").arg(screenNumber);
                        }

                        name += " (" + QString::number(screen->size().width()) + "x" +
                                QString::number(screen->size().height()) + ")";
                        screens[name] = screen;
                        ++screenNumber;
                    }

                    // 弹出选择对话框
                    QString selected = QInputDialog::getItem(this,
                                                             tr("Choose fullscreen target screen"),
                                                             tr("Screen"), screens.keys(), 0, false, &ok);

                    if(ok && !selected.isEmpty())
                    {
                        // 切换到选中的屏幕
                        context()->setScreen(screens[selected]);
                        move(screens[selected]->geometry().x(),
                             screens[selected]->geometry().y());
                        resize(screens[selected]->geometry().width(),
                               screens[selected]->geometry().height());
                    }
                }

                if(ok)
                {
                    // 全屏模式下设置 1px 边框
                    if(layout())
                        layout()->setContentsMargins(1, 1, 1, 1);

                    // 从父部件中分离并全屏显示
                    setParent(0);
                    showFullScreen();
                }
            }
        }
        else
        {
            // 从全屏恢复到窗口模式
            showNormal();
            setMinimumSize(g);

            // 重新将部件设置为 QMainWindow 的中央部件
            QMainWindow* _mainwindow = dynamic_cast<QMainWindow*>(parent());
            if(_mainwindow){
                _mainwindow->setCentralWidget(this);
            }

            // 恢复边距
            if(layout())
                layout()->setContentsMargins(sMargins);

            qApp->processEvents();
            setMinimumSize(QSize(1, 1));
        }
    }
    else
    {
        // 非 F 键：转发到 OSG 渲染器
        m_renderer->keyPressEvent(event);

        // 调用 event->ignore() 让事件继续传播到父部件，
        // 从而允许 Qt 的 QAction 快捷键（如 Ctrl+O、Ctrl+S、R 等）
        // 即使在 OSG 视口拥有键盘焦点时也能正常触发
        event->ignore();
    }
}

/**
 * @brief 键盘释放事件处理
 * @param event Qt 键盘事件
 * 将事件转发到 OSG 渲染器处理。
 */
void osgQOpenGLWidget::keyReleaseEvent(QKeyEvent* event)
{
    Q_ASSERT(m_renderer);
    m_renderer->keyReleaseEvent(event);
}

/**
 * @brief 鼠标按下事件处理
 * @param event Qt 鼠标事件
 * 将事件转发到 OSG 渲染器处理。
 */
void osgQOpenGLWidget::mousePressEvent(QMouseEvent* event)
{
    Q_ASSERT(m_renderer);
    m_renderer->mousePressEvent(event);
}

/**
 * @brief 鼠标释放事件处理
 * @param event Qt 鼠标事件
 * 将事件转发到 OSG 渲染器处理。
 */
void osgQOpenGLWidget::mouseReleaseEvent(QMouseEvent* event)
{
    Q_ASSERT(m_renderer);
    m_renderer->mouseReleaseEvent(event);
}

/**
 * @brief 鼠标双击事件处理
 * @param event Qt 鼠标事件
 * 将事件转发到 OSG 渲染器处理。
 */
void osgQOpenGLWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
    Q_ASSERT(m_renderer);
    m_renderer->mouseDoubleClickEvent(event);
}

/**
 * @brief 鼠标移动事件处理
 * @param event Qt 鼠标事件
 * 将事件转发到 OSG 渲染器处理。
 */
void osgQOpenGLWidget::mouseMoveEvent(QMouseEvent* event)
{
    Q_ASSERT(m_renderer);
    m_renderer->mouseMoveEvent(event);
}

/**
 * @brief 鼠标滚轮事件处理
 * @param event Qt 滚轮事件
 * 将事件转发到 OSG 渲染器处理。
 */
void osgQOpenGLWidget::wheelEvent(QWheelEvent* event)
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
void osgQOpenGLWidget::setDefaultDisplaySettings()
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
 * 2. 根据是否提供了参数解析器，创建对应的 OSGRenderer 实例
 * 3. 获取当前屏幕的像素比
 * 4. 调用 renderer->setupOSG() 完成 OSG 系统初始化
 *
 * 注意：OSGRenderer 的父对象设置为 this（osgQOpenGLWidget），
 * 这样渲染器的生命周期由 Qt 的父子对象机制自动管理。
 */
void osgQOpenGLWidget::createRenderer()
{
    // 在创建 OSG 视图器之前设置默认显示参数
    setDefaultDisplaySettings();

    // 创建渲染器实例
    if (!_arguments) {
        m_renderer = new OSGRenderer(this);
    } else {
        m_renderer = new OSGRenderer(_arguments, this);
    }

    // 获取屏幕像素比（用于高 DPI 缩放）
    QScreen* screen = windowHandle()
                      && windowHandle()->screen() ? windowHandle()->screen() :
                      qApp->screens().front();
    m_renderer->setupOSG(width(), height(), screen->devicePixelRatio());
}
