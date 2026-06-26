#include "ui/OverlayPanelWidget.h"

#include <QLabel>
#include <QMouseEvent>
#include <QPushButton>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGraphicsDropShadowEffect>

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

OverlayPanelWidget::OverlayPanelWidget(const QString& title, QWidget* content,
                                       QWidget* parent)
    : QFrame(parent) {

    setObjectName("OverlayPanelWidget");
    setMinimumWidth(280);
    setMaximumWidth(380);
    setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Maximum);

    // Frame appearance
    setFrameShape(QFrame::StyledPanel);
    setFrameShadow(QFrame::Raised);

    // ── Main layout ──────────────────────────────────────────────────
    m_mainLayout = new QVBoxLayout(this);
    m_mainLayout->setContentsMargins(0, 0, 0, 0);
    m_mainLayout->setSpacing(0);

    // ── Title bar ────────────────────────────────────────────────────
    m_titleBar = new QWidget(this);
    m_titleBar->setObjectName("OverlayTitleBar");
    auto* titleLayout = new QHBoxLayout(m_titleBar);
    titleLayout->setContentsMargins(8, 5, 4, 5);
    titleLayout->setSpacing(4);

    m_titleLabel = new QLabel(title, m_titleBar);
    m_titleLabel->setObjectName("OverlayTitleLabel");
    titleLayout->addWidget(m_titleLabel, 1);

    m_closeBtn = new QPushButton(QStringLiteral("✕"), m_titleBar);  // ✕
    m_closeBtn->setObjectName("OverlayCloseButton");
    m_closeBtn->setFixedSize(20, 20);
    m_closeBtn->setCursor(Qt::ArrowCursor);
    titleLayout->addWidget(m_closeBtn);

    m_mainLayout->addWidget(m_titleBar);

    // ── Content area ─────────────────────────────────────────────────
    m_contentArea = new QWidget(this);
    m_contentArea->setObjectName("OverlayContentArea");
    auto* contentLayout = new QVBoxLayout(m_contentArea);
    contentLayout->setContentsMargins(6, 6, 6, 6);
    contentLayout->setSpacing(0);
    m_mainLayout->addWidget(m_contentArea, 1);

    // ── Apply styles (after all children exist) ──────────────────────
    applyStyle();

    // ── Connect close button ─────────────────────────────────────────
    connect(m_closeBtn, &QPushButton::clicked,
            this, &OverlayPanelWidget::closeRequested);

    // ── Install content if provided ──────────────────────────────────
    if (content) {
        setContentWidget(content);
    }

    // ── Drop shadow for depth ────────────────────────────────────────
    auto* shadow = new QGraphicsDropShadowEffect(this);
    shadow->setBlurRadius(16);
    shadow->setOffset(0, 4);
    shadow->setColor(QColor(0, 0, 0, 120));
    setGraphicsEffect(shadow);
}

// ---------------------------------------------------------------------------
// Style
// ---------------------------------------------------------------------------

void OverlayPanelWidget::applyStyle() {
    // Frame — dark semi-transparent, rounded corners
    setStyleSheet(QStringLiteral(R"(
        #OverlayPanelWidget {
            background: rgba(28, 28, 34, 235);
            border: 1px solid rgba(72, 72, 82, 200);
            border-radius: 6px;
        }
    )"));

    m_titleLabel->setStyleSheet(QStringLiteral(R"(
        #OverlayTitleLabel {
            background: transparent;
            color: #cccccc;
            font-weight: bold;
            font-size: 12px;
        }
    )"));

    m_titleBar->setStyleSheet(QStringLiteral(R"(
        #OverlayTitleBar {
            background: rgba(20, 20, 26, 220);
            border-top-left-radius: 5px;
            border-top-right-radius: 5px;
        }
    )"));

    m_closeBtn->setStyleSheet(QStringLiteral(R"(
        #OverlayCloseButton {
            background: transparent;
            color: #888888;
            border: none;
            padding: 0px;
            font-size: 12px;
        }
        #OverlayCloseButton:hover {
            color: #ffffff;
            background: rgba(200, 60, 60, 200);
            border-radius: 3px;
        }
    )"));

    m_contentArea->setStyleSheet(QStringLiteral(R"(
        #OverlayContentArea {
            background: transparent;
        }
    )"));
}

// ---------------------------------------------------------------------------
// Content management
// ---------------------------------------------------------------------------

void OverlayPanelWidget::setContentWidget(QWidget* content) {
    if (!content) return;

    // Remove any previously-set content
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

    // Reparent into content area
    content->setParent(m_contentArea);
    cl->addWidget(content);
}

QString OverlayPanelWidget::title() const {
    return m_titleLabel ? m_titleLabel->text() : QString();
}

// ---------------------------------------------------------------------------
// Drag — mouse events on title bar
// ---------------------------------------------------------------------------

void OverlayPanelWidget::mousePressEvent(QMouseEvent* event) {
    if (event->button() == Qt::LeftButton) {
        // Only start drag from title bar area
        if (m_titleBar && event->position().y() <= m_titleBar->height()) {
            m_dragging = true;
            // Store offset from widget position to global mouse position
            m_dragStartPos = event->globalPosition().toPoint() -
                             mapToParent(QPoint(0, 0));
            setCursor(Qt::ClosedHandCursor);
            event->accept();
            return;
        }
    }
    QFrame::mousePressEvent(event);
}

void OverlayPanelWidget::mouseMoveEvent(QMouseEvent* event) {
    if (m_dragging) {
        QPoint newParentPos = event->globalPosition().toPoint() - m_dragStartPos;

        // Clamp to parent bounds
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

void OverlayPanelWidget::mouseReleaseEvent(QMouseEvent* event) {
    if (m_dragging && event->button() == Qt::LeftButton) {
        m_dragging = false;
        setCursor(Qt::ArrowCursor);
        event->accept();
        return;
    }
    QFrame::mouseReleaseEvent(event);
}
