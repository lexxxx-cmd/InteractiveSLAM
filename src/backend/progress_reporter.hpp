#pragma once

#include <QObject>
#include <QString>
#include <QThread>
#include <string>
#include "data/hdl_graph_slam/progress_interface.hpp"

/**
 * @brief 线程安全的进度报告器 —— 将 hdl_graph_slam::ProgressInterface 桥接到 Qt 信号系统。
 *
 * 核心职责：
 *   1. 实现 ProgressInterface 抽象接口，供后端加载代码调用
 *   2. 自动检测调用线程，将工作线程的进度更新跨线程转发到主线程
 *   3. 通过 Qt 信号通知前端 UI 更新进度显示
 *
 * 线程安全机制：
 *   - 每个 set_* 方法首先检测调用线程
 *   - 如果从工作线程调用，通过 QMetaObject::invokeMethod 使用
 *     Qt::QueuedConnection 将调用重新分发到主线程的事件循环
 *   - 如果从主线程调用，直接发射信号
 */
class ProgressReporter : public QObject, public hdl_graph_slam::ProgressInterface {
    Q_OBJECT
public:
    /**
     * @brief 构造函数
     * @param parent Qt 父对象
     */
    explicit ProgressReporter(QObject* parent = nullptr);

    // ========== ProgressInterface 接口实现（可从任何线程安全调用） ==========

    /** @brief 设置进度标题（如 "加载点云"、"优化图" 等） */
    void set_title(const std::string& title) override;

    /** @brief 设置进度文本描述（如 "正在处理文件 3/10"） */
    void set_text(const std::string& text) override;

    /** @brief 设置进度最大值（如总文件数） */
    void set_maximum(int max) override;

    /** @brief 设置当前进度值 */
    void set_current(int current) override;

    /** @brief 进度值递增 1 */
    void increment() override;

    /**
     * @brief 重置进度状态到构造初值（仅主线程调用）
     *
     * 置 m_current = 0、m_maximum = 100，与构造函数一致。
     * 只清状态、不发射信号：采用"清零 + 后续信号自然覆盖"语义——
     * 新一轮加载开始（emit loadingStarted 之前）由 GraphManager 同步调用，
     * 若上一轮工作线程还有未消费的 queued 进度调用晚于本 reset 到达，
     * 残留旧值会被新图首批信号冲掉；不 emit 可避免遮罩在 indeterminate
     * 忙碌态下闪现 0/100 确定模式。
     */
    void reset();

signals:
    /** @brief 进度标题变化信号 */
    void titleChanged(const QString& title);
    /** @brief 进度文本描述变化信号 */
    void textChanged(const QString& text);
    /** @brief 进度值变化信号（当前值，最大值） */
    void progressChanged(int current, int maximum);

private:
    /**
     * @brief 确保状态更新在主线程中执行
     *
     * 此方法的实体会检测当前线程，如果不在主线程则通过
     * invokeMethod 重新分发。目前各 set_* 方法各自实现了
     * 这一逻辑，此方法作为文档性辅助。
     */
    void ensureMainThread();

    int m_maximum = 100;  ///< 进度最大值（默认为 100）
    int m_current = 0;    ///< 当前进度值（从 0 开始）
};

