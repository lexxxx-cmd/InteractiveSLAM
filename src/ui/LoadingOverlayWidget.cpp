/**
 * @file LoadingOverlayWidget.cpp
 * @brief 加载遮罩进度覆盖层实现
 *
 * 实现全屏加载遮罩，包括：
 * - 雾化半透明背景 + 暗色圆角卡片（与 OverlayPanelWidget 同风格）
 * - 自绘 spinner：QConicalGradient 渐隐尾迹旋转圆弧（无 QMovie/GIF）
 * - 进度条：确定模式按比例填充 + 计数文本；忙碌模式来回滑动动画块
 * - 父窗口 Resize/Show 事件过滤，保证遮罩铺满、卡片始终居中
 */

#include "ui/LoadingOverlayWidget.h"

#include <QBrush>
#include <QEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QTimer>

// ---------------------------------------------------------------------------
// 常量（布局 / 配色 / 动画节奏）
// ---------------------------------------------------------------------------

namespace {
const int kTimerIntervalMs = 16;      ///< 动画帧间隔（约 60fps）
const int kSpinnerPeriodMs = 1200;    ///< spinner 旋转一圈的周期（毫秒）
const int kBusyPeriodMs = 1600;       ///< 忙碌块来回滑动一个周期（毫秒）
const int kArcSpanDeg = 270;          ///< spinner 圆弧跨度（度）
const double kSpinnerRadius = 24.0;   ///< spinner 半径（直径 48）
const double kArcPenWidth = 4.0;      ///< spinner 弧线线宽
const double kCardRadius = 10.0;      ///< 卡片圆角半径
const double kCardW = 360.0;          ///< 卡片宽度
const double kCardH = 220.0;          ///< 卡片高度
const double kBarHeight = 6.0;        ///< 进度条高度
const double kBusyBlockRatio = 0.3;   ///< 忙碌块宽度占进度条宽度比例

const QColor kMaskColor(0, 0, 0, 110);           ///< 整窗雾化遮罩色
const QColor kCardBg(26, 26, 31, 242);           ///< 卡片背景（≈0.95 不透明度）
const QColor kCardBorder(255, 255, 255, 25);     ///< 卡片 1px 边框
const QColor kAccent(79, 156, 255);              ///< 主题亮蓝（#4f9cff）
const QColor kAccentFaded(79, 156, 255, 0);      ///< 主题亮蓝（全透明，渐隐尾迹端）
const QColor kTitleColor(232, 232, 238);         ///< 标题文字色
const QColor kTextColor(150, 150, 158);          ///< 描述文字色（灰）
const QColor kProgressTextColor(140, 140, 148);  ///< 计数文字色（小、灰）
const QColor kBarTrack(255, 255, 255, 28);       ///< 进度条轨道色
}  // namespace

// ---------------------------------------------------------------------------
// 构造
// ---------------------------------------------------------------------------

/**
 * @brief 构造函数
 *
 * 创建动画定时器（仅可见时由 showOverlay 启动），安装父窗口事件
 * 过滤器以跟踪 Resize/Show，初始隐藏。
 *
 * @param parent 父级 Qt 组件（通常为 MainWindow）
 */
LoadingOverlayWidget::LoadingOverlayWidget(QWidget* parent)
    : QWidget(parent) {
    // 动画定时器：仅遮罩可见期间运行（showOverlay/hideOverlay 控制），
    // 避免隐藏时后台空转重绘
    m_timer = new QTimer(this);
    m_timer->setInterval(kTimerIntervalMs);
    connect(m_timer, &QTimer::timeout, this, [this]() { update(); });

    // 监听父窗口尺寸/显示变化（含窗口缩放、DPI 变化），
    // 保持遮罩铺满整个主窗口、卡片始终居中
    if (parentWidget()) {
        parentWidget()->installEventFilter(this);
        setGeometry(parentWidget()->rect());
    }

    // 初始隐藏，由加载生命周期（onLoadingStarted/Succeeded/Failed）控制显示
    hide();
}

// ---------------------------------------------------------------------------
// 公开 API — 显示 / 隐藏 / 内容更新
// ---------------------------------------------------------------------------

/** @brief 显示遮罩并启动动画（同步几何 → show → raise → 启动定时器） */
void LoadingOverlayWidget::showOverlay() {
    if (parentWidget()) {
        setGeometry(parentWidget()->rect());  // 立即对齐父窗口当前几何
    }
    m_elapsed.restart();
    m_timer->start();
    show();
    raise();  // 提到主窗口所有子控件最上层
}

/** @brief 隐藏遮罩并停止动画（停止定时器 → hide） */
void LoadingOverlayWidget::hideOverlay() {
    m_timer->stop();
    hide();
}

/** @brief 设置卡片标题（空字符串时不绘制） */
void LoadingOverlayWidget::setTitle(const QString& title) {
    if (m_title == title) return;
    m_title = title;
    update();
}

/** @brief 设置卡片描述文本（空字符串时不绘制） */
void LoadingOverlayWidget::setText(const QString& text) {
    if (m_text == text) return;
    m_text = text;
    update();
}

/**
 * @brief 设置确定进度
 *
 * maximum > 0 时进度条按 current/maximum 比例填充并显示计数文本；
 * maximum <= 0 时自动切换为忙碌模式（防御，正常由调用方分流）。
 */
void LoadingOverlayWidget::setProgress(int current, int maximum) {
    if (maximum <= 0) {
        setIndeterminate();
        return;
    }
    m_indeterminate = false;
    m_current = current;
    m_maximum = maximum;
    update();
}

/** @brief 切换为忙碌（indeterminate）模式：进度条为来回滑动的动画块 */
void LoadingOverlayWidget::setIndeterminate() {
    m_indeterminate = true;
    update();
}

// ---------------------------------------------------------------------------
// 事件 — 绘制 / 父窗口几何跟踪
// ---------------------------------------------------------------------------

/**
 * @brief 绘制遮罩
 *
 * 自下而上：整窗半透明黑雾 → 居中圆角卡片 → spinner → 标题 →
 * 描述文本 → 进度条。空标题/空文本不绘制。
 */
void LoadingOverlayWidget::paintEvent(QPaintEvent* /*event*/) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    // ---- 1. 整窗半透明黑雾（雾化遮罩） ----
    painter.fillRect(rect(), kMaskColor);

    // ---- 2. 居中圆角卡片（OverlayPanelWidget 同风格暗色面板） ----
    QRectF cardRect(QPointF((width() - kCardW) / 2.0, (height() - kCardH) / 2.0),
                    QSizeF(kCardW, kCardH));
    painter.setPen(QPen(kCardBorder, 1));
    painter.setBrush(kCardBg);
    painter.drawRoundedRect(cardRect, kCardRadius, kCardRadius);

    const qreal top = cardRect.top();
    const qreal left = cardRect.left();

    // ---- 3. 自绘 spinner（水平居中，直径 48） ----
    drawSpinner(painter, QPointF(cardRect.center().x(), top + 48), kSpinnerRadius);

    // ---- 4. 标题（较大字号、加粗；空则不绘制） ----
    if (!m_title.isEmpty()) {
        painter.setFont(scaledFont(1.2, true));
        painter.setPen(kTitleColor);
        painter.drawText(QRectF(left, top + 84, kCardW, 26),
                         Qt::AlignHCenter | Qt::AlignVCenter, m_title);
    }

    // ---- 5. 描述文本（较小、灰色；空则不绘制） ----
    if (!m_text.isEmpty()) {
        painter.setFont(scaledFont(0.95));
        painter.setPen(kTextColor);
        painter.drawText(QRectF(left, top + 114, kCardW, 20),
                         Qt::AlignHCenter | Qt::AlignVCenter, m_text);
    }

    // ---- 6. 进度条（卡片内宽，高 6） ----
    drawProgressBar(painter, QRectF(left + 28, top + 156, kCardW - 56, kBarHeight));
}

/**
 * @brief 父窗口事件过滤
 *
 * 父窗口缩放/显示（含 DPI 变化）时同步几何，保证遮罩始终铺满。
 */
bool LoadingOverlayWidget::eventFilter(QObject* watched, QEvent* event) {
    if (watched == parentWidget() &&
        (event->type() == QEvent::Resize || event->type() == QEvent::Show)) {
        if (parentWidget()) setGeometry(parentWidget()->rect());
    }
    return QWidget::eventFilter(watched, event);
}

// ---------------------------------------------------------------------------
// 私有 — spinner / 进度条 / 字体
// ---------------------------------------------------------------------------

/**
 * @brief 绘制带渐隐尾迹的旋转圆弧 spinner
 *
 * 旋转角由动画时钟驱动（每 kSpinnerPeriodMs 毫秒一圈）；画笔使用
 * 锥形渐变（尾迹端透明 → 前端亮蓝），配合 270° 圆弧形成渐隐尾迹。
 */
void LoadingOverlayWidget::drawSpinner(QPainter& painter,
                                       const QPointF& center, double radius) const {
    const qreal startDeg =
        (m_elapsed.elapsed() % kSpinnerPeriodMs) * 360.0 / kSpinnerPeriodMs;

    // 锥形渐变：弧的尾迹端（起点）透明，扫过 270° 到前端变为亮蓝
    QConicalGradient gradient(center, startDeg);
    gradient.setColorAt(0.0, kAccentFaded);
    gradient.setColorAt(kArcSpanDeg / 360.0, kAccent);
    gradient.setColorAt(1.0, kAccentFaded);

    painter.setPen(QPen(QBrush(gradient), kArcPenWidth,
                        Qt::SolidLine, Qt::RoundCap));
    painter.setBrush(Qt::NoBrush);
    painter.drawArc(QRectF(center.x() - radius, center.y() - radius,
                           radius * 2.0, radius * 2.0),
                    qRound(startDeg * 16), kArcSpanDeg * 16);
}

/**
 * @brief 绘制进度条
 *
 * 确定模式：按 current/maximum 比例填充并显示 "current / maximum" 文本；
 * 忙碌模式：约 30% 宽的亮块在槽内来回滑动（三角波，同一动画时钟驱动）。
 */
void LoadingOverlayWidget::drawProgressBar(QPainter& painter,
                                           const QRectF& barRect) const {
    painter.setPen(Qt::NoPen);

    // 轨道（暗色半透明圆角槽）
    painter.setBrush(kBarTrack);
    painter.drawRoundedRect(barRect, kBarHeight / 2, kBarHeight / 2);

    if (!m_indeterminate && m_maximum > 0) {
        // ---- 确定模式：按比例填充 ----
        const qreal ratio = qBound(0.0, qreal(m_current) / qreal(m_maximum), 1.0);
        if (ratio > 0.0) {
            QRectF fillRect = barRect;
            fillRect.setWidth(barRect.width() * ratio);
            // 圆角半径按填充宽度收缩，避免极小宽度时形状异常
            const qreal r = qMin(kBarHeight / 2, fillRect.width() / 2);
            painter.setBrush(kAccent);
            painter.drawRoundedRect(fillRect, r, r);
        }
        // ---- 计数文本（条下方居中） ----
        painter.setPen(kProgressTextColor);
        painter.setFont(scaledFont(0.85));
        painter.drawText(QRectF(barRect.left(), barRect.bottom() + 6,
                                barRect.width(), 16),
                         Qt::AlignHCenter | Qt::AlignVCenter,
                         tr("%1 / %2").arg(m_current).arg(m_maximum));
    } else {
        // ---- 忙碌模式：来回滑动的动画块 ----
        const qreal phase =
            (m_elapsed.elapsed() % kBusyPeriodMs) / qreal(kBusyPeriodMs);
        // 三角波：0 → 1 → 0
        const qreal tri = (phase < 0.5) ? phase * 2.0 : 2.0 - phase * 2.0;
        const qreal blockW = barRect.width() * kBusyBlockRatio;
        const qreal x = barRect.left() + (barRect.width() - blockW) * tri;
        painter.setBrush(kAccent);
        painter.drawRoundedRect(QRectF(x, barRect.top(), blockW, barRect.height()),
                                kBarHeight / 2, kBarHeight / 2);
    }
}

/**
 * @brief 基于控件字体按比例缩放字号
 *
 * @param scale 字号缩放比例（1.0 = 原尺寸）
 * @param bold  是否加粗
 */
QFont LoadingOverlayWidget::scaledFont(qreal scale, bool bold) const {
    QFont f = font();
    if (f.pointSizeF() <= 0) f.setPointSizeF(9.0);  // 像素字体时回退基准
    f.setPointSizeF(f.pointSizeF() * scale);
    f.setBold(bold);
    return f;
}
