/**
 * @file OverlayPanelWidget.cpp
 * @brief 叠加面板容器实现
 *
 * 实现可拖动的浮动叠加面板，包括：
 * - 深色半透明圆角框架样式
 * - 标题栏 + 内容区的布局
 * - 鼠标拖动定位（限制在父部件边界内）
 * - 关闭按钮的信号发射
 * -  CSS 样式表（Qt StyleSheet）美化
 */

#include "ui/OverlayPanelWidget.h"

#include <QLabel>
#include <QMouseEvent>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------

/**
 * @brief 构造函数
 *
 * 创建面板框架、标题栏（含标题标签和关闭按钮）、内容区布局，
 * 应用暗色风格样式表，并安装阴影效果。
 *
 * @param title   面板标题文字
 * @param content 要嵌入面板的内容部件
 * @param parent  父级部件
 */
OverlayPanelWidget::OverlayPanelWidget(const QString& title, QWidget* content,
                                       QWidget* parent)
    : QFrame(parent) {

    setObjectName("OverlayPanelWidget");
    setMinimumWidth(280);
    setMaximumWidth(380);
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

    // 框架外观
    setFrameShape(QFrame::StyledPanel);
    setFrameShadow(QFrame::Raised);

    // ---- 主布局 ----
    m_mainLayout = new QVBoxLayout(this);
    m_mainLayout->setContentsMargins(0, 0, 0, 0);
    m_mainLayout->setSpacing(0);

    // ---- 标题栏 ----
    m_titleBar = new QWidget(this);
    m_titleBar->setObjectName("OverlayTitleBar");
    auto* titleLayout = new QHBoxLayout(m_titleBar);
    titleLayout->setContentsMargins(8, 5, 4, 5);
    titleLayout->setSpacing(4);

    // 标题标签
    m_titleLabel = new QLabel(title, m_titleBar);
    m_titleLabel->setObjectName("OverlayTitleLabel");
    titleLayout->addWidget(m_titleLabel, 1);

    // 关闭按钮（✕）
    m_closeBtn = new QPushButton(QStringLiteral("✕"), m_titleBar);
    m_closeBtn->setObjectName("OverlayCloseButton");
    m_closeBtn->setFixedSize(20, 20);
    m_closeBtn->setCursor(Qt::ArrowCursor);
    titleLayout->addWidget(m_closeBtn);

    m_mainLayout->addWidget(m_titleBar);

    // ---- 内容区域 ----
    m_contentArea = new QWidget(this);
    m_contentArea->setObjectName("OverlayContentArea");
    auto* contentLayout = new QVBoxLayout(m_contentArea);
    contentLayout->setContentsMargins(6, 6, 6, 6);
    contentLayout->setSpacing(0);
    m_mainLayout->addWidget(m_contentArea, 1);

    // ---- 应用样式（在所有子部件创建后） ----
    applyStyle();

    // ---- 连接关闭按钮信号 ----
    connect(m_closeBtn, &QPushButton::clicked,
            this, &OverlayPanelWidget::closeRequested);

    // ---- 安装内容部件（如果提供） ----
    if (content) {
        setContentWidget(content);
    }

}

// ---------------------------------------------------------------------------
// 样式
// ---------------------------------------------------------------------------

/**
 * @brief 应用 CSS 风格样式
 *
 * 所有子控件样式（标题栏、标题标签、关闭按钮、内容区域）
 * 由全局 QSS 通过 objectName 选择器统一处理。
 * 此处仅设置框架级别样式作为兜底。
 */
void OverlayPanelWidget::applyStyle() {
    setStyleSheet(QStringLiteral(R"(
        #OverlayPanelWidget {
            background: rgba(26, 26, 31, 0.95);
            border: 1px solid #2a2a35;
            border-radius: 6px;
        }
    )"));
}

// ---------------------------------------------------------------------------
// 内容管理
// ---------------------------------------------------------------------------

/**
 * @brief 设置内容部件
 *
 * 移除之前的内容部件，将新部件嵌入内容区域。
 * @param content 新的内容部件
 */
void OverlayPanelWidget::setContentWidget(QWidget* content) {
    if (!content) return;

    // 移除之前的内容部件
    QLayout* cl = m_contentArea->layout();
    if (cl) {
        QLayoutItem* item;
        while ((item = cl->takeAt(0)) != nullptr) {
            if (item->widget()) {
                item->widget()->setParent(nullptr);
            }
            delete item;
        }
    }

    // 将新部件重新设置父级到内容区域
    content->setParent(m_contentArea);
    cl->addWidget(content);
}

QString OverlayPanelWidget::title() const {
    return m_titleLabel ? m_titleLabel->text() : QString();
}

// ---------------------------------------------------------------------------
// 拖动 — 标题栏鼠标事件
// ---------------------------------------------------------------------------

/**
 * @brief 鼠标按下事件
 *
 * 当在标题栏区域按下左键时开始拖动模式。
 */
void OverlayPanelWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        // 仅在标题栏区域开始拖动
        if (m_titleBar && event->position().y() <= m_titleBar->height()) {
            m_dragging = true;
            // 存储部件位置到鼠标全局位置的偏移量
            m_dragStartPos = event->globalPosition().toPoint() -
                             mapToParent(QPoint(0, 0));
            setCursor(Qt::ClosedHandCursor);
            event->accept();
            return;
        }
    }
    QFrame::mousePressEvent(event);
}

/**
 * @brief 鼠标移动事件
 *
 * 拖动模式下，根据鼠标移动更新面板位置，并限制在父部件边界内。
 */
void OverlayPanelWidget::mouseMoveEvent(QMouseEvent* event) {
    if (m_dragging) {
        QPoint newParentPos = event->globalPosition().toPoint() - m_dragStartPos;

        // 限制在父部件边界内
        if (parentWidget()) {
            int pw = parentWidget()->width();
            int ph = parentWidget()->height();
            newParentPos.setX(std::max(0, std::min(newParentPos.x(), pw - width())));
            newParentPos.setY(std::max(0, std::min(newParentPos.y(), ph - height())));
        }

        move(newParentPos);
        event->accept();
        return;
    }
    QFrame::mouseMoveEvent(event);
}

/**
 * @brief 鼠标释放事件
 *
 * 结束拖动模式，恢复光标样式。
 */
void OverlayPanelWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (m_dragging && event->button() == Qt::LeftButton) {
        m_dragging = false;
        setCursor(Qt::ArrowCursor);
        event->accept();
        return;
    }
    QFrame::mouseReleaseEvent(event);
}
