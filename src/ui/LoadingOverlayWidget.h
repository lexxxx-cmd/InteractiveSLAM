/**
 * @file LoadingOverlayWidget.h
 * @brief 加载遮罩进度覆盖层头文件
 *
 * LoadingOverlayWidget 是铺满主窗口的全屏加载遮罩：
 * 半透明黑雾 + 居中暗色卡片（自绘 spinner 转圈 + 标题/描述 + 进度条）。
 * 显示期间位于主窗口最上层，拦截鼠标/键盘事件，冻结底层交互。
 *
 * 进度数据由后端 ProgressReporter 的信号驱动（跨线程转发已由后端完成）：
 *   - setTitle() / setText()：更新卡片文案
 *   - setProgress(current, maximum)：确定进度条 + 计数文本
 *   - setIndeterminate()：忙碌模式（来回滑动的动画块）
 *
 * 使用示例：
 *   m_loadingOverlay = new LoadingOverlayWidget(mainWindow);  // 初始隐藏
 *   m_loadingOverlay->setIndeterminate();
 *   m_loadingOverlay->showOverlay();   // 加载开始
 *   ...
 *   m_loadingOverlay->hideOverlay();   // 加载结束
 */

#pragma once

#include <QElapsedTimer>
#include <QFont>
#include <QString>
#include <QWidget>

class QPainter;
class QTimer;

/**
 * @brief 全屏加载遮罩进度对话框
 *
 * 继承自 QWidget，作为主窗口的直接子控件铺满整个窗口区域：
 * - paintEvent 自绘雾化背景、圆角卡片、带渐隐尾迹的旋转圆弧 spinner、
 *   进度条（确定/忙碌两种模式）
 * - 动画由约 60fps 的 QTimer 驱动，仅在遮罩可见期间运行
 * - 通过事件过滤器监听父窗口 Resize/Show，窗口缩放/DPI 变化时
 *   保持遮罩铺满、卡片始终居中
 */
class LoadingOverlayWidget : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param parent 父级 Qt 组件（通常为 MainWindow，遮罩将铺满其整个区域）
     */
    explicit LoadingOverlayWidget(QWidget* parent = nullptr);

    /** @brief 显示遮罩并启动动画（同步几何 → show → raise → 启动定时器） */
    void showOverlay();

    /** @brief 隐藏遮罩并停止动画（停止定时器 → hide，避免后台空转） */
    void hideOverlay();

    /** @brief 设置卡片标题（空字符串时不绘制） */
    void setTitle(const QString& title);

    /** @brief 设置卡片描述文本（空字符串时不绘制） */
    void setText(const QString& text);

    /**
     * @brief 设置确定进度
     *
     * maximum > 0 时进度条按 current/maximum 比例填充并显示计数文本；
     * maximum <= 0 时自动切换为忙碌模式。
     *
     * @param current 当前进度值
     * @param maximum 进度最大值
     */
    void setProgress(int current, int maximum);

    /** @brief 切换为忙碌（indeterminate）模式：进度条为来回滑动的动画块 */
    void setIndeterminate();

protected:
    void paintEvent(QPaintEvent* event) override;  ///< 绘制雾化遮罩、卡片与动画
    bool eventFilter(QObject* watched, QEvent* event) override;  ///< 监听父窗口几何变化

private:
    /** @brief 绘制带渐隐尾迹的旋转圆弧 spinner */
    void drawSpinner(QPainter& painter, const QPointF& center, double radius) const;

    /** @brief 绘制进度条（确定模式按比例填充，忙碌模式为滑动动画块） */
    void drawProgressBar(QPainter& painter, const QRectF& barRect) const;

    /** @brief 基于控件字体按比例缩放字号（像素字体时回退 9pt 基准） */
    QFont scaledFont(qreal scale, bool bold = false) const;

    QTimer* m_timer = nullptr;      ///< 动画定时器（约 60fps，仅遮罩可见时运行）
    QElapsedTimer m_elapsed;        ///< 动画时钟（spinner 旋转角 / 忙碌块位置）
    QString m_title;                ///< 卡片标题（空则不绘制）
    QString m_text;                 ///< 卡片描述文本（空则不绘制）
    bool m_indeterminate = true;    ///< 是否忙碌模式（初始忙碌，直到后端报告具体进度）
    int m_current = 0;              ///< 当前进度值（确定模式）
    int m_maximum = 0;              ///< 进度最大值（确定模式，<=0 视为忙碌）
};
