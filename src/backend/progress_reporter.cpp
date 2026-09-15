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
#include <QCoreApplication>
#include <QHash>
#include <QStringList>
#include <QDebug>

// ============================================================================
// 后端进度文案翻译表（spec B3 / C-3.1）
//
// 后端经 std::string 传来的英文文案在此单一边界点查表翻译：
//   - 固定文案：后端英文原串本身就是查表 key（后端零改动）；
//   - 含参文案：经 set_title_fmt/set_text_fmt 传 key + 参数（见下表模板）。
//
// QT_TRANSLATE_NOOP 登记 context 为 "ProgressReporter"，供 lupdate 抓取；
// 中文翻译在 translations/app_zh_CN.ts 中维护。
// 查表 miss 时回显原串并 qWarning 打点，便于发现漏登记的新文案（spec R-3）。
// ============================================================================
namespace {

const char kTitleOpening[] = QT_TRANSLATE_NOOP("ProgressReporter", "Opening %1");
const char kTextKeyframe[] = QT_TRANSLATE_NOOP("ProgressReporter", "keyframe %1/%2");

/// 固定文案 → 翻译查表（key 即后端英文原串）
const QHash<QString, const char*>& fixedTextTable() {
    static const QHash<QString, const char*> table = {
        {"loading graph",         QT_TRANSLATE_NOOP("ProgressReporter", "loading graph")},
        {"loading keyframes",     QT_TRANSLATE_NOOP("ProgressReporter", "loading keyframes")},
        {"saving graph",          QT_TRANSLATE_NOOP("ProgressReporter", "saving graph")},
        {"saving keyframes",      QT_TRANSLATE_NOOP("ProgressReporter", "saving keyframes")},
        {"accumulate points",     QT_TRANSLATE_NOOP("ProgressReporter", "accumulate points")},
        {"saving pcd",            QT_TRANSLATE_NOOP("ProgressReporter", "saving pcd")},
        {"saving LVBA format",    QT_TRANSLATE_NOOP("ProgressReporter", "saving LVBA format")},
        {"reading odometry",      QT_TRANSLATE_NOOP("ProgressReporter", "reading odometry")},
        {"reading point clouds",  QT_TRANSLATE_NOOP("ProgressReporter", "reading point clouds")},
        {"writing keyframes",     QT_TRANSLATE_NOOP("ProgressReporter", "writing keyframes")},
    };
    return table;
}

/// 翻译一条后端文案；未登记时回显原串并打点
QString translateProgressText(const QString& raw) {
    const auto& table = fixedTextTable();
    const auto it = table.constFind(raw);
    if (it == table.constEnd()) {
        qWarning() << "[ProgressReporter] untranslated progress text:" << raw;
        return raw;
    }
    return QCoreApplication::translate("ProgressReporter", it.value());
}

}  // namespace

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
 *
 * 后端固定文案（英文原串即 key）在此单一边界点查表翻译（spec C-3.1）；
 * 含参标题请走 set_title_fmt。
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
    const QString raw = QString::fromStdString(title);
    emit titleChanged(translateProgressText(raw));
}

/**
 * @brief 设置进度文本描述（跨线程安全）
 *
 * @param text 描述文本（std::string）
 *
 * 后端固定文案（英文原串即 key）在此单一边界点查表翻译（spec C-3.1）；
 * 含参文案请走 set_text_fmt。
 */
void ProgressReporter::set_text(const std::string& text) {
    if (QThread::currentThread() != this->thread()) {
        QMetaObject::invokeMethod(this, [this, text]() {
            set_text(text);
        }, Qt::QueuedConnection);
        return;
    }
    const QString raw = QString::fromStdString(text);
    emit textChanged(translateProgressText(raw));
}

/**
 * @brief 设置进度标题（带参模板，跨线程安全）
 *
 * key 查翻译模板（"Opening %1"）→ translate → .arg() 填参后发射（spec C-3.3）。
 */
void ProgressReporter::set_title_fmt(const std::string& key, const std::string& arg) {
    if (QThread::currentThread() != this->thread()) {
        QMetaObject::invokeMethod(this, [this, key, arg]() {
            set_title_fmt(key, arg);
        }, Qt::QueuedConnection);
        return;
    }
    if (key == "progress.opening") {
        emit titleChanged(QCoreApplication::translate("ProgressReporter", kTitleOpening)
                              .arg(QString::fromStdString(arg)));
        return;
    }
    // 未登记的含参 key：退回普通查表路径（miss 时回显原串并打点）
    set_title(key);
}

/**
 * @brief 设置进度文本描述（带参模板，两个整型参数，跨线程安全）
 *
 * key 查翻译模板（"keyframe %1/%2"）→ translate → .arg() 填参后发射（spec C-3.3）。
 */
void ProgressReporter::set_text_fmt(const std::string& key, int a, int b) {
    if (QThread::currentThread() != this->thread()) {
        QMetaObject::invokeMethod(this, [this, key, a, b]() {
            set_text_fmt(key, a, b);
        }, Qt::QueuedConnection);
        return;
    }
    if (key == "progress.keyframe") {
        emit textChanged(QCoreApplication::translate("ProgressReporter", kTextKeyframe)
                             .arg(a).arg(b));
        return;
    }
    // 未登记的含参 key：退回普通查表路径（miss 时回显原串并打点）
    set_text(key);
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
 * @brief 重置进度状态到构造初值（仅主线程调用，不发射信号）
 *
 * 由 GraphManager 的加载入口在 emit loadingStarted 之前同步调用，
 * 与工作线程首批 queued 进度调用满足 happens-before。
 * 语义为"清零 + 后续信号自然覆盖"：只清内部状态，立即由
 * set_maximum/increment/set_text 等真实信号接管遮罩显示。
 */
void ProgressReporter::reset() {
    m_maximum = 100;
    m_current = 0;
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
