/**
 * @file osgQOpenGLWindow.h
 * @brief 基于 QOpenGLWindow 的 OSG 集成窗口的头文件
 *
 * 本文件定义了 osgQOpenGLWindow 类，继承自 QOpenGLWindow 和 QOpenGLFunctions，
 * 是实现 OSG（OpenSceneGraph）与 Qt 集成的另一种方式。
 *
 * 与 osgQOpenGLWidget 的区别：
 * - osgQOpenGLWidget：基于 QOpenGLWidget，可嵌入到 Qt 布局系统中
 * - osgQOpenGLWindow：基于 QOpenGLWindow，是一个顶级窗口，性能更好
 *   （无需通过 QWidget 的渲染路径，减少了额外的合成开销）
 *
 * 主要功能：
 * 1. 在 QOpenGLWindow 中集成 OSG 渲染
 * 2. 通过 QWidget::createWindowContainer 封装为 QWidget，方便嵌入现有布局
 * 3. 管理 OSGRenderer 渲染器的生命周期
 * 4. 将 Qt 的输入事件转发到 OSG 渲染器
 * 5. 处理高 DPI 屏幕的缩放
 * 6. 管理线程安全的读写互斥锁
 */

#ifndef OSGQOPENGLWINDOW_H
#define OSGQOPENGLWINDOW_H

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

#include <QOpenGLWindow>
#include <QOpenGLFunctions>
#include <QReadWriteLock>

class OSGRenderer;
class QWidget;

namespace osgViewer
{
    class Viewer;
}

/**
 * @class osgQOpenGLWindow
 * @brief 基于 QOpenGLWindow 的 OSG 集成窗口
 *
 * 继承自 QOpenGLWindow，提供 OSG 渲染的顶级窗口集成。
 * 内部创建一个 QWidget 包装器（通过 createWindowContainer），
 * 使其可以作为子部件嵌入到 Qt 布局中。
 *
 * 核心职责：
 * - 创建和管理 OSGRenderer 实例
 * - 在 initializeGL 中初始化 OpenGL 函数和 OSG 渲染器
 * - 在 paintGL 中执行 OSG 帧更新
 * - 将所有输入事件转发给 OSGRenderer 处理
 * - 通过信号 initialized() 通知外部初始化完成
 * - 提供 asWidget() 方法获取包装的 QWidget，便于布局集成
 *
 * 线程安全：
 * - 使用 OpenThreads::ReadWriteMutex 保护 OSG 场景图的访问
 * - paintGL 中获取读锁，允许并发的场景图读取，防止写入冲突
 */
class osgQOpenGLWindow : public QOpenGLWindow,
    protected QOpenGLFunctions
{
    Q_OBJECT

protected:
    /** @brief OSG 渲染器指针 */
    OSGRenderer* m_renderer {nullptr};

    /** @brief 标志：OSG 是否需要渲染下一帧 */
    bool _osgWantsToRenderFrame{true};

    /** @brief OSG 场景图的读写互斥锁 */
    OpenThreads::ReadWriteMutex _osgMutex;

    /** @brief 是否为第一帧（用于首次设置默认 FBO） */
    bool _isFirstFrame {true};

    friend class OSGRenderer;

    /** @brief QWidget 包装器（通过 createWindowContainer 创建），用于嵌入布局 */
    QWidget* _widget = nullptr;

public:
    /**
     * @brief 构造函数
     * @param parent 父 Qt 部件
     *
     * 构造时创建工作包装器 QWidget（通过 createWindowContainer），
     * 使得此 OpenGL 窗口可以嵌入到 Qt 的布局管理器中。
     */
    osgQOpenGLWindow(QWidget* parent = nullptr);

    /** @brief 析构函数 */
    virtual ~osgQOpenGLWindow();

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
     * @brief 获取包装的 QWidget
     * @return QWidget 指针，可用于嵌入到布局中
     */
    QWidget* asWidget()
    {
        return _widget;
    }

signals:
    /** @brief 初始化完成信号 */
    void initialized();

protected:
    /**
     * @brief 初始化 OpenGL（重写 QOpenGLWindow）
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
     * @brief 绘制 OpenGL 内容（重写 QOpenGLWindow）
     *
     * 获取读锁后执行 OSG 帧更新。
     * 第一帧时设置默认 FBO ID。
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
     * @brief 创建渲染器
     *
     * 设置默认显示设置，创建 OSGRenderer 实例，
     * 调用 setupOSG 完成渲染器初始化。
     */
    void createRenderer();
};

#endif // OSGQOPENGLWINDOW_H
