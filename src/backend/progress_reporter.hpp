#pragma once

#include <QObject>
#include <QString>
#include <QThread>
#include <string>
#include "guik/progress_interface.hpp"

/**
 * @brief Thread-safe progress reporter — bridges guik::ProgressInterface to QML.
 *
 * Worker threads call set_title/set_text/etc. The implementation detects
 * the calling thread and uses QMetaObject::invokeMethod with
 * Qt::QueuedConnection to forward the call to the main thread, where
 * Qt signals are safely emitted for QML consumption.
 */
class ProgressReporter : public QObject, public guik::ProgressInterface {
    Q_OBJECT
public:
    explicit ProgressReporter(QObject* parent = nullptr);

    // ProgressInterface implementation — safe to call from any thread
    void set_title(const std::string& title) override;
    void set_text(const std::string& text) override;
    void set_maximum(int max) override;
    void set_current(int current) override;
    void increment() override;

signals:
    void titleChanged(const QString& title);
    void textChanged(const QString& text);
    void progressChanged(int current, int maximum);

private:
    void ensureMainThread();

    int m_maximum = 100;
    int m_current = 0;
};
