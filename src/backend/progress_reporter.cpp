#include "backend/progress_reporter.hpp"
#include <QMetaObject>

ProgressReporter::ProgressReporter(QObject* parent)
    : QObject(parent), m_maximum(100), m_current(0) {}

void ProgressReporter::ensureMainThread() {
    if (QThread::currentThread() != this->thread()) {
        // Called from worker thread — re-dispatch to main thread via the event loop.
        // The method name passed to invokeMethod must match exactly.
        // We use a lambda wrapper because the ProgressInterface methods have
        // different signatures that can't all be dispatched with a single string.
        // Instead, each set_* method checks the thread and invokes itself.
    }
}

void ProgressReporter::set_title(const std::string& title) {
    if (QThread::currentThread() != this->thread()) {
        QMetaObject::invokeMethod(this, [this, title]() {
            set_title(title);
        }, Qt::QueuedConnection);
        return;
    }
    emit titleChanged(QString::fromStdString(title));
}

void ProgressReporter::set_text(const std::string& text) {
    if (QThread::currentThread() != this->thread()) {
        QMetaObject::invokeMethod(this, [this, text]() {
            set_text(text);
        }, Qt::QueuedConnection);
        return;
    }
    emit textChanged(QString::fromStdString(text));
}

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
