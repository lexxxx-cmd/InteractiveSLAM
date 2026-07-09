// ============================================================================
// progress_reporter.cpp
// 进度报告器实现文件
//
// 本文件实现了 ProgressReporter 类的所有功能，包括：
//   - 跨线程安全的进度报告接口
//   - 工作线程到主线程的自动消息转发
//   - 进度变化信号的发射
// ============================================================================

#include "backend/progress_reporter.hpp"
#include <QMetaObject>

// ============================================================================
// 构造函数
// ============================================================================

/**
 * @brief 构造函数：初始化进度范围为 0~100，当前进度为 0
 * @param parent Qt 父对象
 */
ProgressReporter::ProgressReporter(QObject* parent)
    : QObject(parent), m_maximum(100), m_current(0) {}

// ============================================================================
// 线程检测辅助方法
// ============================================================================

/**
 * @brief 检查并确保操作在主线程中执行
 *
 * 此方法目前作为文档性辅助，各 set_* 方法各自实现了线程检测逻辑。
 * 原因是 ProgressInterface 的不同方法具有不同的参数签名，
 * 无法通过统一的 invokeMethod 字符串调用分发。
 * 因此每个 set_* 方法各自通过 lambda 闭包进行跨线程调用。
 */
void ProgressReporter::ensureMainThread() {
    if (QThread::currentThread() != this->thread()) {
        // 从工作线程调用 —— 通过事件循环重新分发到主线程
        // 具体实现在各 set_* 方法中
    }
}

// ============================================================================
// 进度接口实现（ProgressInterface）
//
// 每个 set_* 方法都遵循同样的跨线程调用模式：
//   1. 检测当前线程是否与 QObject 所属线程一致
//   2. 如果不一致，通过 QMetaObject::invokeMethod 使用
//      Qt::QueuedConnection 将 lambda 重新排队到主线程事件循环
//   3. 如果一致（已在主线程），直接发射信号
// ============================================================================

/**
 * @brief 设置进度标题（跨线程安全）
 *
 * @param title 标题字符串（std::string，来自后端代码）
 */
void ProgressReporter::set_title(const std::string& title) {
    // 检测是否在正确的线程中调用
    if (QThread::currentThread() != this->thread()) {
        // 不在主线程 —— 通过事件循环重新分发到主线程
        QMetaObject::invokeMethod(this, [this, title]() {
            set_title(title);  // 在主线程中递归调用自身
        }, Qt::QueuedConnection);
        return;
    }
    emit titleChanged(QString::fromStdString(title));
}

/**
 * @brief 设置进度文本描述（跨线程安全）
 *
 * @param text 描述文本（std::string）
 */
void ProgressReporter::set_text(const std::string& text) {
    if (QThread::currentThread() != this->thread()) {
        QMetaObject::invokeMethod(this, [this, text]() {
            set_text(text);
        }, Qt::QueuedConnection);
        return;
    }
    emit textChanged(QString::fromStdString(text));
}

/**
 * @brief 设置进度最大值（跨线程安全）
 *
 * @param max 进度最大值
 */
void ProgressReporter::set_maximum(int max) {
    if (QThread::currentThread() != this->thread()) {
        QMetaObject::invokeMethod(this, [this, max]() {
            set_maximum(max);
        }, Qt::QueuedConnection);
        return;
    }
    m_maximum = max;
    emit progressChanged(m_current, m_maximum);
}

/**
 * @brief 设置当前进度值（跨线程安全）
 *
 * @param current 当前进度值
 */
void ProgressReporter::set_current(int current) {
    if (QThread::currentThread() != this->thread()) {
        QMetaObject::invokeMethod(this, [this, current]() {
            set_current(current);
        }, Qt::QueuedConnection);
        return;
    }
    m_current = current;
    emit progressChanged(m_current, m_maximum);
}

/**
 * @brief 进度值递增 1（跨线程安全）
 *
 * 适用于循环遍历等场景，每次处理完一个项目后调用。
 */
void ProgressReporter::increment() {
    if (QThread::currentThread() != this->thread()) {
        QMetaObject::invokeMethod(this, [this]() {
            increment();
        }, Qt::QueuedConnection);
        return;
    }
    m_current++;
    emit progressChanged(m_current, m_maximum);
}
