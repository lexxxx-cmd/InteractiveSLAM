#pragma once

#include <QFrame>
#include <QPoint>

class QLabel;
class QPushButton;
class QVBoxLayout;

/// @brief A draggable overlay panel that floats over the 3D viewport.
///        Wraps any content QWidget in a dark semi-transparent frame with
///        a title bar for dragging and a close button.
class OverlayPanelWidget : public QFrame {
    Q_OBJECT

public:
    explicit OverlayPanelWidget(const QString& title, QWidget* content,
                                QWidget* parent = nullptr);

    /// Reparent the given widget into this overlay's content area.
    void setContentWidget(QWidget* content);

    QString title() const;

signals:
    void closeRequested();

protected:
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    void applyStyle();

    QLabel* m_titleLabel = nullptr;
    QPushButton* m_closeBtn = nullptr;
    QWidget* m_titleBar = nullptr;
    QWidget* m_contentArea = nullptr;
    QVBoxLayout* m_mainLayout = nullptr;

    bool m_dragging = false;
    QPoint m_dragStartPos;  // offset from widget pos to mouse global pos at drag start
};
