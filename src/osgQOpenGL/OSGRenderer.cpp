/**
 * @file OSGRenderer.cpp
 * @brief OSG 渲染器的实现文件
 *
 * 本文件实现了 OSGRenderer 类的所有方法，包括：
 * 1. QtKeyboardMap 键盘映射工具类（将 Qt 键码映射为 OSG 键码）
 * 2. 事件处理（键盘、鼠标、滚轮事件从 Qt 到 OSG 的转换）
 * 3. 帧更新管理（setupOSG、frame、checkNeedToDoFrame、timerEvent）
 * 4. 窗口大小管理和渲染请求
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

#include "OSGRenderer.h"

#include "osgQOpenGLWindow.h"
#include "osgQOpenGLWidget.h"

//#include <osgQOpenGL/CullVisitorEx>
//#include <osgQOpenGL/GraphicsWindowEx>

#include <QApplication>
#include <QScreen>
#include <QOpenGLContext>
#include <QOpenGLFunctions>
#include <QOpenGLShaderProgram>
#include <QOpenGLVertexArrayObject>

#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>

#include <QThread>

namespace
{

    /**
     * @class QtKeyboardMap
     * @brief Qt 键码到 OSG 键码的映射工具类
     *
     * 将 Qt 键盘事件中的键码（Qt::Key_*）映射为 OSG 事件系统使用的键码
     * （osgGA::GUIEventAdapter::KEY_*）。
     * 对于未在映射表中的键，尝试使用事件文本的 ASCII 码作为键码。
     */
    class QtKeyboardMap
    {
    public:
        QtKeyboardMap()
        {
            // 功能键映射
            mKeyMap[Qt::Key_Escape     ] = osgGA::GUIEventAdapter::KEY_Escape;
            mKeyMap[Qt::Key_Delete     ] = osgGA::GUIEventAdapter::KEY_Delete;
            mKeyMap[Qt::Key_Home       ] = osgGA::GUIEventAdapter::KEY_Home;
            mKeyMap[Qt::Key_Enter      ] = osgGA::GUIEventAdapter::KEY_KP_Enter;
            mKeyMap[Qt::Key_End        ] = osgGA::GUIEventAdapter::KEY_End;
            mKeyMap[Qt::Key_Return     ] = osgGA::GUIEventAdapter::KEY_Return;
            mKeyMap[Qt::Key_PageUp     ] = osgGA::GUIEventAdapter::KEY_Page_Up;
            mKeyMap[Qt::Key_PageDown   ] = osgGA::GUIEventAdapter::KEY_Page_Down;
            mKeyMap[Qt::Key_Left       ] = osgGA::GUIEventAdapter::KEY_Left;
            mKeyMap[Qt::Key_Right      ] = osgGA::GUIEventAdapter::KEY_Right;
            mKeyMap[Qt::Key_Up         ] = osgGA::GUIEventAdapter::KEY_Up;
            mKeyMap[Qt::Key_Down       ] = osgGA::GUIEventAdapter::KEY_Down;
            mKeyMap[Qt::Key_Backspace  ] = osgGA::GUIEventAdapter::KEY_BackSpace;
            mKeyMap[Qt::Key_Tab        ] = osgGA::GUIEventAdapter::KEY_Tab;
            mKeyMap[Qt::Key_Space      ] = osgGA::GUIEventAdapter::KEY_Space;
            mKeyMap[Qt::Key_Delete     ] = osgGA::GUIEventAdapter::KEY_Delete;

            // 修饰键映射
            mKeyMap[Qt::Key_Alt        ] = osgGA::GUIEventAdapter::KEY_Alt_L;
            mKeyMap[Qt::Key_Shift      ] = osgGA::GUIEventAdapter::KEY_Shift_L;
            mKeyMap[Qt::Key_Control    ] = osgGA::GUIEventAdapter::KEY_Control_L;
            mKeyMap[Qt::Key_Meta       ] = osgGA::GUIEventAdapter::KEY_Meta_L;

            // F1~F20 功能键映射
            mKeyMap[Qt::Key_F1         ] = osgGA::GUIEventAdapter::KEY_F1;
            mKeyMap[Qt::Key_F2         ] = osgGA::GUIEventAdapter::KEY_F2;
            mKeyMap[Qt::Key_F3         ] = osgGA::GUIEventAdapter::KEY_F3;
            mKeyMap[Qt::Key_F4         ] = osgGA::GUIEventAdapter::KEY_F4;
            mKeyMap[Qt::Key_F5         ] = osgGA::GUIEventAdapter::KEY_F5;
            mKeyMap[Qt::Key_F6         ] = osgGA::GUIEventAdapter::KEY_F6;
            mKeyMap[Qt::Key_F7         ] = osgGA::GUIEventAdapter::KEY_F7;
            mKeyMap[Qt::Key_F8         ] = osgGA::GUIEventAdapter::KEY_F8;
            mKeyMap[Qt::Key_F9         ] = osgGA::GUIEventAdapter::KEY_F9;
            mKeyMap[Qt::Key_F10        ] = osgGA::GUIEventAdapter::KEY_F10;
            mKeyMap[Qt::Key_F11        ] = osgGA::GUIEventAdapter::KEY_F11;
            mKeyMap[Qt::Key_F12        ] = osgGA::GUIEventAdapter::KEY_F12;
            mKeyMap[Qt::Key_F13        ] = osgGA::GUIEventAdapter::KEY_F13;
            mKeyMap[Qt::Key_F14        ] = osgGA::GUIEventAdapter::KEY_F14;
            mKeyMap[Qt::Key_F15        ] = osgGA::GUIEventAdapter::KEY_F15;
            mKeyMap[Qt::Key_F16        ] = osgGA::GUIEventAdapter::KEY_F16;
            mKeyMap[Qt::Key_F17        ] = osgGA::GUIEventAdapter::KEY_F17;
            mKeyMap[Qt::Key_F18        ] = osgGA::GUIEventAdapter::KEY_F18;
            mKeyMap[Qt::Key_F19        ] = osgGA::GUIEventAdapter::KEY_F19;
            mKeyMap[Qt::Key_F20        ] = osgGA::GUIEventAdapter::KEY_F20;

            // 符号键映射
            mKeyMap[Qt::Key_hyphen     ] = '-';
            mKeyMap[Qt::Key_Equal      ] = '=';
            mKeyMap[Qt::Key_division   ] = osgGA::GUIEventAdapter::KEY_KP_Divide;
            mKeyMap[Qt::Key_multiply   ] = osgGA::GUIEventAdapter::KEY_KP_Multiply;
            mKeyMap[Qt::Key_Minus      ] = '-';
            mKeyMap[Qt::Key_Plus       ] = '+';
            mKeyMap[Qt::Key_Insert     ] = osgGA::GUIEventAdapter::KEY_KP_Insert;
        }

        ~QtKeyboardMap() {}

        /**
         * @brief 将 Qt 键码映射为 OSG 键码
         * @param event Qt 键盘事件
         * @return 对应的 OSG 键码
         *
         * 如果在映射表中找到匹配项则返回 OSG 键码，
         * 否则尝试取事件文本的第一个字符的 ASCII 码作为键码。
         */
        int remapKey(QKeyEvent* event)
        {
            KeyMap::iterator itr = mKeyMap.find(event->key());

            if(itr == mKeyMap.end())
            {
                return int(*(event->text().toLatin1().data()));
            }
            else
                return itr->second;
        }

    private:
        /** @brief 键码映射表：Qt 键码 -> OSG 键码 */
        typedef std::map<unsigned int, int> KeyMap;
        KeyMap mKeyMap;
    };

    /** @brief 全局 Qt 键码映射器单例 */
    static QtKeyboardMap s_QtKeyboardMap;
} // namespace

/**
 * @brief 无参数解析器的构造函数
 * @param parent 父 QObject 对象
 *
 * 直接初始化 QObject 和 osgViewer::Viewer 基类。
 */
OSGRenderer::OSGRenderer(QObject* parent)
    : QObject(parent), osgViewer::Viewer()
{
}

/**
 * @brief 带参数解析器的构造函数
 * @param arguments OSG 命令行参数解析器
 * @param parent    父 QObject 对象
 *
 * 将 arguments 传递给 osgViewer::Viewer 的构造函数进行初始化。
 */
OSGRenderer::OSGRenderer(osg::ArgumentParser* arguments, QObject* parent)
    : QObject(parent), osgViewer::Viewer(*arguments)
{
}

/**
 * @brief 析构函数
 */
OSGRenderer::~OSGRenderer()
{
}

/**
 * @brief 请求父窗口/部件渲染一帧
 *
 * 通过 dynamic_cast 判断父对象的实际类型（osgQOpenGLWindow 或 osgQOpenGLWidget），
 * 设置 _osgWantsToRenderFrame 标志为 true，然后调用父对象的 update()
 * 方法触发 Qt 的 repaint/paintGL 流程。
 */
void OSGRenderer::update()
{
    osgQOpenGLWindow* osgWidgetRendered = dynamic_cast<osgQOpenGLWindow*>(parent());

    if(osgWidgetRendered != nullptr)
    {
        osgWidgetRendered->_osgWantsToRenderFrame = true;
        osgWidgetRendered->update();
    }
    else
    {
        osgQOpenGLWidget* osgWidget = dynamic_cast<osgQOpenGLWidget*>(parent());
        osgWidget->_osgWantsToRenderFrame = true;
        osgWidget->update();
    }
}

/**
 * @brief 调整渲染窗口大小
 * @param windowWidth  窗口宽度（Qt 逻辑像素）
 * @param windowHeight 窗口高度（Qt 逻辑像素）
 * @param windowScale  窗口缩放因子（高 DPI 缩放）
 *
 * 更新 OSG 的相机视口、嵌入式图形窗口尺寸和事件队列中的窗口尺寸。
 * 所有尺寸需乘以 windowScale 以适配高 DPI 屏幕（如 Retina 显示屏）。
 */
void OSGRenderer::resize(int windowWidth, int windowHeight, float windowScale)
{
    if(!m_osgInitialized)
        return;

    m_windowScale = windowScale;

    // 更新相机视口（像素坐标乘以缩放因子）
    _camera->setViewport(new osg::Viewport(0, 0, windowWidth * windowScale,
                                           windowHeight * windowScale));

    // 更新嵌入式图形窗口尺寸
    m_osgWinEmb->resized(0, 0,
                         windowWidth * windowScale,
                         windowHeight * windowScale);
    // 更新事件队列中的窗口尺寸
    m_osgWinEmb->getEventQueue()->windowResize(0, 0,
                                               windowWidth * windowScale,
                                               windowHeight * windowScale);

    update();
}

/**
 * @brief 初始化 OSG 渲染系统
 * @param windowWidth  窗口宽度
 * @param windowHeight 窗口高度
 * @param windowScale  窗口缩放因子
 *
 * 初始化流程：
 * 1. 创建嵌入式图形窗口（GraphicsWindowEmbedded），设置尺寸
 * 2. 同步事件队列的窗口矩形
 * 3. 创建并设置相机视口，关联图形上下文
 * 4. 禁用 Escape 键退出（避免用户按 Escape 时退出程序）
 * 5. 设置在帧结束时是否释放上下文（设为 false，由 Qt 管理）
 * 6. 设置线程模型为单线程（Qt 的 OpenGL 上下文通常只能在主线程使用）
 * 7. 启动 Qt 定时器（10ms 间隔）来驱动帧循环
 */
void OSGRenderer::setupOSG(int windowWidth, int windowHeight, float windowScale)
{
    m_osgInitialized = true;
    m_windowScale = windowScale;

    // 创建嵌入式图形窗口（不实际创建 OS 窗口，而是嵌入到 Qt 的 OpenGL 表面）
    m_osgWinEmb = new osgViewer::GraphicsWindowEmbedded(0, 0,
                                                        windowWidth * windowScale, windowHeight * windowScale);

    // 确保事件队列的窗口矩形与图形上下文一致
    m_osgWinEmb->getEventQueue()->syncWindowRectangleWithGraphicsContext();

    // 设置相机视口和图形上下文
    _camera->setViewport(new osg::Viewport(0, 0, windowWidth * windowScale,
                                           windowHeight * windowScale));
    _camera->setGraphicsContext(m_osgWinEmb.get());

    // 禁用 Escape 键退出功能（OSG 默认按下 Escape 会设置 done 标志退出主循环）
    setKeyEventSetsDone(0);

    // 不在帧结束时释放 OpenGL 上下文（由 Qt 管理上下文的生命周期）
    setReleaseContextAtEndOfFrameHint(false);

    // 设置为单线程模式（因为 Qt 的 OpenGL 上下文通常只在主线程中有效）
    setThreadingModel(osgViewer::Viewer::SingleThreaded);

    // 启动 10ms 精确定时器驱动帧循环
    _timerId = startTimer(10, Qt::PreciseTimer);
    _lastFrameStartTime.setStartTick(0);
}

/**
 * @brief 设置键盘修饰键状态（Shift、Ctrl、Alt）
 * @param event Qt 输入事件
 *
 * 从 Qt 事件中提取 Shift、Ctrl、Alt 修饰键的状态，
 * 转换为 OSG 的修饰键掩码并设置到嵌入式窗口的事件状态中。
 */
void OSGRenderer::setKeyboardModifiers(QInputEvent* event)
{
    unsigned int modkey = event->modifiers() & (Qt::ShiftModifier |
                                                Qt::ControlModifier |
                                                Qt::AltModifier);
    unsigned int mask = 0;

    if(modkey & Qt::ShiftModifier) mask |= osgGA::GUIEventAdapter::MODKEY_SHIFT;
    if(modkey & Qt::ControlModifier) mask |= osgGA::GUIEventAdapter::MODKEY_CTRL;
    if(modkey & Qt::AltModifier) mask |= osgGA::GUIEventAdapter::MODKEY_ALT;

    m_osgWinEmb->getEventQueue()->getCurrentEventState()->setModKeyMask(mask);
}

/**
 * @brief 键盘按键按下事件处理
 * @param event Qt 键盘事件
 *
 * 将 Qt 的键盘按下事件转换为 OSG 事件并发送到嵌入式窗口的事件队列。
 */
void OSGRenderer::keyPressEvent(QKeyEvent* event)
{
    setKeyboardModifiers(event);
    int value = s_QtKeyboardMap.remapKey(event);
    m_osgWinEmb->getEventQueue()->keyPress(value);
}

/**
 * @brief 键盘按键释放事件处理
 * @param event Qt 键盘事件
 *
 * 将 Qt 的键盘释放事件转换为 OSG 事件。
 * 如果是自动重复事件（长按触发），则忽略该事件。
 */
void OSGRenderer::keyReleaseEvent(QKeyEvent* event)
{
    if(event->isAutoRepeat())
    {
        event->ignore();
    }
    else
    {
        setKeyboardModifiers(event);
        int value = s_QtKeyboardMap.remapKey(event);
        m_osgWinEmb->getEventQueue()->keyRelease(value);
    }
}

/**
 * @brief 鼠标按键按下事件处理
 * @param event Qt 鼠标事件
 *
 * 将 Qt 鼠标按钮转换为 OSG 按钮编号（左键=1，中键=2，右键=3），
 * 转换坐标（乘以缩放因子），然后发送到 OSG 事件队列。
 */
void OSGRenderer::mousePressEvent(QMouseEvent* event)
{
    int button = 0;

    switch(event->button())
    {
    case Qt::LeftButton:   button = 1; break;
    case Qt::MiddleButton: button = 2; break;
    case Qt::RightButton:  button = 3; break;
    default:               button = 0; break;
    }

    setKeyboardModifiers(event);
    m_osgWinEmb->getEventQueue()->mouseButtonPress(event->x() * m_windowScale,
                                                   event->y() * m_windowScale, button);
}

/**
 * @brief 鼠标按键释放事件处理
 * @param event Qt 鼠标事件
 */
void OSGRenderer::mouseReleaseEvent(QMouseEvent* event)
{
    int button = 0;

    switch(event->button())
    {
    case Qt::LeftButton:   button = 1; break;
    case Qt::MiddleButton: button = 2; break;
    case Qt::RightButton:  button = 3; break;
    default:               button = 0; break;
    }

    setKeyboardModifiers(event);
    m_osgWinEmb->getEventQueue()->mouseButtonRelease(event->x() * m_windowScale,
                                                     event->y() * m_windowScale, button);
}

/**
 * @brief 鼠标双击事件处理
 * @param event Qt 鼠标事件
 */
void OSGRenderer::mouseDoubleClickEvent(QMouseEvent* event)
{
    int button = 0;

    switch(event->button())
    {
    case Qt::LeftButton:   button = 1; break;
    case Qt::MiddleButton: button = 2; break;
    case Qt::RightButton:  button = 3; break;
    default:               button = 0; break;
    }

    setKeyboardModifiers(event);
    m_osgWinEmb->getEventQueue()->mouseDoubleButtonPress(event->x() * m_windowScale,
                                                         event->y() * m_windowScale, button);
}

/**
 * @brief 鼠标移动事件处理
 * @param event Qt 鼠标事件
 */
void OSGRenderer::mouseMoveEvent(QMouseEvent* event)
{
    setKeyboardModifiers(event);
    m_osgWinEmb->getEventQueue()->mouseMotion(event->x() * m_windowScale,
                                              event->y() * m_windowScale);
}

/**
 * @brief 鼠标滚轮事件处理
 * @param event Qt 滚轮事件
 *
 * 首先更新鼠标位置到当前位置，然后根据滚轮的滚动方向
 * 发送 SCROLL_UP/SCROLL_DOWN（垂直滚动）或
 * SCROLL_RIGHT/SCROLL_LEFT（水平滚动）事件。
 */
void OSGRenderer::wheelEvent(QWheelEvent* event)
{
    setKeyboardModifiers(event);
    QPointF pos = event->position();
    float x = pos.x() * m_windowScale;
    float y = pos.y() * m_windowScale;

    // 先更新鼠标位置
    m_osgWinEmb->getEventQueue()->mouseMotion(x, y);

    QPoint delta = event->angleDelta();
    if (!delta.isNull())
    {
        // 垂直方向滚动（上下）
        if (delta.y() != 0)
        {
            m_osgWinEmb->getEventQueue()->mouseScroll(
                delta.y() > 0 ? osgGA::GUIEventAdapter::SCROLL_UP
                : osgGA::GUIEventAdapter::SCROLL_DOWN);
        }
        // 水平方向滚动（左右）
        else if (delta.x() != 0)
        {
            m_osgWinEmb->getEventQueue()->mouseScroll(
                delta.x() > 0 ? osgGA::GUIEventAdapter::SCROLL_RIGHT
                : osgGA::GUIEventAdapter::SCROLL_LEFT);
        }
    }
}

/**
 * @brief 检查是否有待处理的事件
 * @return true 有事件待处理，false 无事件
 *
 * 检查所有已注册的事件源和所有窗口是否有待处理的事件。
 * 重写自 osgViewer::Viewer。
 */
bool OSGRenderer::checkEvents()
{
    // 检查所有已附加的事件源
    for(Devices::iterator eitr = _eventSources.begin();
        eitr != _eventSources.end();
        ++eitr)
    {
        osgGA::Device* es = eitr->get();
        if(es->getCapabilities() & osgGA::Device::RECEIVE_EVENTS)
        {
            if(es->checkEvents())
                return true;
        }
    }

    // 检查所有窗口的事件
    Windows windows;
    getWindows(windows);
    for(Windows::iterator witr = windows.begin();
        witr != windows.end();
        ++witr)
    {
        if((*witr)->checkEvents())
            return true;
    }

    return false;
}

/**
 * @brief 检查是否需要渲染下一帧
 * @return true 需要渲染，false 不需要
 *
 * 重写 osgViewer::Viewer::checkNeedToDoFrame() 方法，
 * 在默认检查逻辑基础上增加了更全面的场景更新判断。
 *
 * 检查条件：
 * 1. 是否有事件处理器请求了重绘（_requestRedraw）
 * 2. 是否请求了持续更新（_requestContinousUpdate）
 * 3. 场景图是否需要更新（更新回调或场景修改）
 * 4. 数据库分页器（DatabasePager）是否需要更新
 * 5. 图像分页器（ImagePager）是否需要更新
 * 6. 场景是否需要重新绘制
 * 7. 是否有待处理的事件
 */
bool OSGRenderer::checkNeedToDoFrame()
{
    // 检查是否有事件处理器请求了重绘
    if(_requestRedraw)
        return true;

    // 检查是否请求了持续更新
    if(_requestContinousUpdate)
        return true;

    // 检查场景图是否需要更新（相机更新回调或场景修改）
    if(requiresUpdateSceneGraph())
        return true;

    // 检查数据库分页器是否需要更新场景
    if(getDatabasePager()->requiresUpdateSceneGraph())
        return true;

    // 检查图像分页器是否需要更新场景
    if(getImagePager()->requiresUpdateSceneGraph())
        return true;

    // 检查场景是否需要重绘
    if(requiresRedraw())
        return true;

    // 检查是否有事件需要处理
    if(checkEvents())
        return true;

    // 再次检查重绘和持续更新请求（可能在事件处理过程中被设置）
    if(_requestRedraw)
        return true;

    if(_requestContinousUpdate)
        return true;

    return false;
}

/**
 * @brief 执行一帧的渲染循环
 * @param simulationTime 仿真时间（默认 USE_REFERENCE_TIME 使用系统时间）
 *
 * 重写 osgViewer::Viewer::frame() 方法，在调用父类 frame 前
 * 加入帧率限制和 CPU 负载控制逻辑。
 *
 * 帧率限制：
 * - 如果设置了最大帧率（runMaxFrameRate > 0），计算上一帧耗时，
 *   如果小于最小帧间隔则 sleep 等待
 *
 * CPU 负载控制（按需渲染模式）：
 * - 在 ON_DEMAND 模式下，如果两帧间隔小于 10ms 则 sleep 等待，
 *   避免在不需要渲染时过度占用 CPU
 *
 * 此方法通常由 osgQOpenGLWidget::paintGL() 或 osgQOpenGLWindow::paintGL() 调用。
 */
void OSGRenderer::frame(double simulationTime)
{
    // 帧率限制：如果设置了最大帧率，确保帧间隔不小于 1/最大帧率
    if(getRunMaxFrameRate() > 0.0)
    {
        double dt = _lastFrameStartTime.time_s();
        double minFrameTime = 1.0 / getRunMaxFrameRate();

        if(dt < minFrameTime)
            QThread::usleep(static_cast<unsigned int>(1000000.0 * (minFrameTime - dt)));
    }

    // 按需渲染模式下的 CPU 负载控制
    if(getRunFrameScheme() == osgViewer::ViewerBase::ON_DEMAND)
    {
        double dt = _lastFrameStartTime.time_s();
        if(dt < 0.01)  // 如果上一帧在 10ms 内完成
        {
            // sleep 剩余时间，避免忙等导致 CPU 占用过高
            OpenThreads::Thread::microSleep(static_cast<unsigned int>(1000000.0 * (0.01 - dt)));
        }
    }

    // 记录当前帧开始时间
    _lastFrameStartTime.setStartTick();

    // 调用父类的帧更新方法（执行事件遍历 -> 更新遍历 -> 渲染遍历）
    osgViewer::Viewer::frame(simulationTime);
}

/**
 * @brief 请求重绘
 *
 * 委托给 osgViewer::Viewer::requestRedraw()。
 * 此方法会在 checkNeedToDoFrame() 中被检查。
 */
void OSGRenderer::requestRedraw()
{
    osgViewer::Viewer::requestRedraw();
}

/**
 * @brief Qt 定时器事件处理
 * @param event 定时器事件（未使用）
 *
 * 由 Qt 定时器触发（10ms 间隔），驱动 OSG 帧更新。
 *
 * 行为逻辑：
 * - 如果应用即将退出，直接返回
 * - 如果是持续渲染模式（CONTINUOUS），直接调用 update()
 * - 如果是按需渲染模式（ON_DEMAND），仅在 checkNeedToDoFrame()
 *   返回 true 时调用 update()，避免空闲时不必要的渲染
 */
void OSGRenderer::timerEvent(QTimerEvent* /*event*/)
{
    // 应用即将退出，不再触发渲染
    if(_applicationAboutToQuit)
    {
        return;
    }

    // 在持续渲染模式或场景需要更新时，触发渲染
    if(getRunFrameScheme() != osgViewer::ViewerBase::ON_DEMAND ||
       checkNeedToDoFrame())
    {
        update();
    }
}
