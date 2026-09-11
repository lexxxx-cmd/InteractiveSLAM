// ============================================================================
// FirstPersonManipulator.h
// 第一人称相机操作器
//
// 功能：FPS 风格的相机控制——鼠标左键拖拽转动视角（yaw/pitch），
//       WASD 沿水平面行走，滚轮调节行走速度。
//       由 ViewportWidget 在 Shift 切换时挂到 viewer 上，
//       退出时经 getPose() 把当前位姿交还给轨迹球操作器无缝衔接。
//
// 设计要点：
//   - 只处理鼠标拖拽/滚轮/FRAME 事件，WASD 键盘状态由 ViewportWidget
//     的 Qt 事件过滤器直接调 setKey() 写入（osgQOpenGLWidget 无键盘焦点，
//     OSG 事件队列的键盘路径不可靠）；
//   - 世界 Z 轴向上（与场景地面网格/坐标轴一致），行走始终保持在
//     水平面内，视线俯仰独立于行走方向（FPS 惯例）；
//   - 行走速度默认按场景包围球半径设定，滚轮实时增减。
// ============================================================================

#pragma once

#include <osgGA/CameraManipulator>

#include <algorithm>
#include <cmath>

/**
 * @brief 第一人称相机操作器
 */
class FirstPersonManipulator : public osgGA::CameraManipulator {
public:
    FirstPersonManipulator() = default;
    FirstPersonManipulator(const FirstPersonManipulator& fm,
                           const osg::CopyOp& copyOp = osg::CopyOp::SHALLOW_COPY)
        : osgGA::CameraManipulator(fm, copyOp) {}

    META_Object(osgGA, FirstPersonManipulator);

    /**
     * @brief 从当前相机位姿启动（进入第一人称模式时调用）
     * @param eye      相机世界坐标
     * @param dir      视线方向（会被归一化，不能与世界上向平行）
     * @param speed    行走速度（米/秒）
     */
    void startFrom(const osg::Vec3d& eye, osg::Vec3d dir, double speed) {
        m_eye = eye;
        dir.normalize();
        m_pitchDeg = std::asin(std::clamp(dir.z(), -1.0, 1.0)) * 57.29577951308232;
        m_yawDeg   = std::atan2(dir.x(), dir.y()) * 57.29577951308232;
        m_speed    = speed;
        m_lastFrameTime = -1.0;
        clearKeys();
    }

    /** @brief 取当前位姿（退出第一人称模式时交给轨迹球操作器） */
    void getPose(osg::Vec3d& eye, osg::Vec3d& dir) const {
        eye = m_eye;
        dir = direction();
    }

    /** @brief 写入单个行走键状态（由 ViewportWidget 的 Qt 键盘事件驱动） */
    void setKey(char key, bool pressed) {
        switch (key) {
        case 'W': m_keyW = pressed; break;
        case 'A': m_keyA = pressed; break;
        case 'S': m_keyS = pressed; break;
        case 'D': m_keyD = pressed; break;
        case 'Q': m_keyUp = pressed; break;    // Q 上升
        case 'E': m_keyDown = pressed; break;  // E 下降
        default: break;
        }
    }

    /** @brief 行走速度（米/秒） */
    void setWalkSpeed(double speed) { m_speed = std::max(0.1, speed); }
    double walkSpeed() const { return m_speed; }

    // --- osgGA::CameraManipulator 接口 ---
    void setByMatrix(const osg::Matrixd& matrix) override {
        m_eye = matrix.getTrans();
        // OSG 矩阵为行向量约定（v * M）：相机世界矩阵旋转部分
        // 第 2 行 = -视线（相机看向自身 -Z），取负得到视线方向
        osg::Vec3d dir(-matrix(2, 0), -matrix(2, 1), -matrix(2, 2));
        dir.normalize();
        m_pitchDeg = std::asin(std::clamp(dir.z(), -1.0, 1.0)) * 57.29577951308232;
        m_yawDeg   = std::atan2(dir.x(), dir.y()) * 57.29577951308232;
    }

    void setByInverseMatrix(const osg::Matrixd& invmatrix) override {
        setByMatrix(osg::Matrixd::inverse(invmatrix));
    }

    osg::Matrixd getMatrix() const override {
        return osg::Matrixd::inverse(getInverseMatrix());
    }

    osg::Matrixd getInverseMatrix() const override {
        const osg::Vec3d up(0.0, 0.0, 1.0);
        return osg::Matrix::lookAt(m_eye, m_eye + direction(), up);
    }

    /** @brief 每帧按当前按键状态移动相机（dt 由 FRAME 事件时间差计算） */
    void updateCamera(osg::Camera& camera) override {
        camera.setViewMatrix(getInverseMatrix());
    }

protected:
    bool handle(const osgGA::GUIEventAdapter& ea,
                osgGA::GUIActionAdapter& /*us*/) override {
        switch (ea.getEventType()) {
        case osgGA::GUIEventAdapter::PUSH:
            if (ea.getButtonMask() & osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {
                m_dragging = true;
                m_lastX = ea.getX();
                m_lastY = ea.getY();
                return true;
            }
            break;

        case osgGA::GUIEventAdapter::DRAG:
            if (m_dragging) {
                const double dx = ea.getX() - m_lastX;
                const double dy = ea.getY() - m_lastY;
                m_lastX = ea.getX();
                m_lastY = ea.getY();
                constexpr double kSens = 0.18;   // 度/像素
                m_yawDeg  += dx * kSens;         // 鼠标右移 → 视线右转
                m_pitchDeg = std::clamp(m_pitchDeg + dy * kSens,
                                        -85.0, 85.0);
                return true;
            }
            break;

        case osgGA::GUIEventAdapter::RELEASE:
            if (m_dragging) { m_dragging = false; return true; }
            break;

        case osgGA::GUIEventAdapter::SCROLL:
            // 滚轮调节行走速度（放大 1.25× / 缩小 0.8×）
            if (ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_UP)
                setWalkSpeed(m_speed * 1.25);
            else if (ea.getScrollingMotion() == osgGA::GUIEventAdapter::SCROLL_DOWN)
                setWalkSpeed(m_speed * 0.8);
            return true;

        case osgGA::GUIEventAdapter::FRAME: {
            const double t = ea.getTime();
            double dt = 0.0;
            if (m_lastFrameTime > 0.0 && t > m_lastFrameTime) dt = t - m_lastFrameTime;
            m_lastFrameTime = t;
            if (dt > 0.0 && dt < 0.5) move(dt);
            return false;  // FRAME 不消费，放行给其他处理器
        }
        default:
            break;
        }
        return false;
    }

private:
    /** @brief 由 yaw/pitch 计算视线方向（世界 Z 向上，yaw=0 朝 +Y） */
    osg::Vec3d direction() const {
        const double yaw   = m_yawDeg  * 0.017453292519943295;
        const double pitch = m_pitchDeg * 0.017453292519943295;
        return osg::Vec3d(std::sin(yaw) * std::cos(pitch),
                          std::cos(yaw) * std::cos(pitch),
                          std::sin(pitch));
    }

    /** @brief 按按键状态在水平面内行走（速度恒定，方向为按键组合的单位向量） */
    void move(double dt) {
        const double yaw = m_yawDeg * 0.017453292519943295;
        const osg::Vec3d fwdH(std::sin(yaw), std::cos(yaw), 0.0);   // 视线水平投影
        const osg::Vec3d right(std::cos(yaw), -std::sin(yaw), 0.0); // 视线右侧

        osg::Vec3d delta;
        if (m_keyW) delta += fwdH;
        if (m_keyS) delta -= fwdH;
        if (m_keyD) delta += right;
        if (m_keyA) delta -= right;
        if (m_keyUp)   delta += osg::Vec3d(0.0, 0.0, 1.0);  // Q 上升
        if (m_keyDown) delta -= osg::Vec3d(0.0, 0.0, 1.0);  // E 下降
        if (delta.length2() > 0.0) {
            delta.normalize();  // OSG Vec3 无 getNormalized()，原地归一化
            m_eye += delta * (m_speed * dt);
        }
    }

    void clearKeys() {
        m_keyW = m_keyA = m_keyS = m_keyD = false;
        m_keyUp = m_keyDown = false;
    }

    osg::Vec3d m_eye{0.0, 0.0, 0.0};
    double m_yawDeg   = 0.0;    ///< 偏航角（绕世界 Z，0 = 朝 +Y）
    double m_pitchDeg = 0.0;    ///< 俯仰角（±85° 限制，避免翻转）
    double m_speed    = 5.0;    ///< 行走速度（米/秒）
    bool   m_keyW = false, m_keyA = false, m_keyS = false, m_keyD = false;
        bool   m_keyUp = false, m_keyDown = false;  // Q 上升 / E 下降
    bool   m_dragging = false;
    float  m_lastX = 0.0f, m_lastY = 0.0f;
    double m_lastFrameTime = -1.0;
};
