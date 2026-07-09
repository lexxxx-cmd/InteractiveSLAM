/**
 * @file OverlayPanelWidget.h
 * @brief 叠加面板容器头文件
 *
 * OverlayPanelWidget 是一个可拖动的浮动面板，悬浮于 3D 视口之上。
 * 它将任意内容 QWidget 包装在深色半透明的面板框架中，
 * 提供标题栏（可拖动）和关闭按钮。
 *
 * 使用示例：
 *   OverlayPanelWidget* panel = new OverlayPanelWidget("Rendering", renderWidget);
 *   viewport->registerOverlay(panel);
 */

#pragma once

#include <QFrame>
#include <QPoint>

class QLabel;
class QPushButton;
class QVBoxLayout;

/**
 * @brief 可拖动的浮动叠加面板
 *
 * 继承自 QFrame，样式为深色半透明背景、圆角边框。
 * 包含一个标题栏（可拖动）和一个内容区域。
 * 当用户点击关闭按钮时，发射 closeRequested 信号。
 *
 * 拖动功能：
 * - 仅在标题栏区域按下左键开始拖动
 * - 拖动过程中位置被限制在父部件边界内
 * - 释放左键结束拖动
 */
class OverlayPanelWidget : public QFrame {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param title   面板标题（显示在标题栏中）
     * @param content 面板内容部件（将被嵌入内容区域）
     * @param parent  父级 Qt 组件
     */
    explicit OverlayPanelWidget(const QString& title, QWidget* content,
                                QWidget* parent = nullptr);

    /**
     * @brief 设置内容部件
     *
     * 将给定部件重新设置父级到面板的内容区域。
     * 如果之前已有内容部件，将被移除。
     * @param content 新的内容部件
     */
    void setContentWidget(QWidget* content);

    QString title() const;  ///< 获取面板标题

signals:
    void closeRequested();  ///< 用户点击关闭按钮时发出

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    void applyStyle();  ///< 应用 CSS 样式表

    QLabel* m_titleLabel = nullptr;      ///< 标题标签
    QPushButton* m_closeBtn = nullptr;   ///< 关闭按钮（显示"✕"）
    QWidget* m_titleBar = nullptr;       ///< 标题栏容器
    QWidget* m_contentArea = nullptr;     ///< 内容区域容器
    QVBoxLayout* m_mainLayout = nullptr;  ///< 主布局

    bool m_dragging = false;            ///< 是否正在拖动
    QPoint m_dragStartPos;              ///< 拖动起始偏移（部件位置到鼠标全局位置的偏移量）
};
