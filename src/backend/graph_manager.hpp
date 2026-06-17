#pragma once

#include <QObject>
#include <QUrl>
#include <QFutureWatcher>
#include <memory>
#include <QAtomicInt>

#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "backend/progress_reporter.hpp"

/**
 * @brief QObject wrapper for InteractiveGraph — bridges data layer to QML.
 *
 * Thread safety:
 *   - openMapData() triggers QtConcurrent::run → worker thread loads data
 *   - onLoadFinished() runs in main thread, installs shared_ptr<InteractiveGraph>
 *   - Renderer reads via sharedGraph() in synchronize() (main thread blocked)
 *   - shared_ptr ensures the graph is never destroyed during render()
 */
class GraphManager : public QObject {
    Q_OBJECT

    Q_PROPERTY(bool isLoaded READ isLoaded NOTIFY isLoadedChanged)
    Q_PROPERTY(int vertexCount READ vertexCount NOTIFY statsChanged)
    Q_PROPERTY(int edgeCount READ edgeCount NOTIFY statsChanged)
    Q_PROPERTY(int keyframeCount READ keyframeCount NOTIFY statsChanged)
    Q_PROPERTY(QString lastMessage READ lastMessage NOTIFY lastMessageChanged)
    Q_PROPERTY(QString lastLogLevel READ lastLogLevel NOTIFY lastMessageChanged)
    Q_PROPERTY(bool isLoading READ isLoading NOTIFY isLoadingChanged)
    Q_PROPERTY(ProgressReporter* progress READ progress CONSTANT)

public:
    enum LogLevel { INFO, WARNING, ERROR };
    Q_ENUM(LogLevel)

    explicit GraphManager(QObject* parent = nullptr);

    bool isLoaded() const;
    bool isLoading() const;
    int vertexCount() const;
    int edgeCount() const;
    int keyframeCount() const;
    QString lastMessage() const;
    QString lastLogLevel() const;
    ProgressReporter* progress() const;

    Q_INVOKABLE void logInfo(const QString& msg);
    Q_INVOKABLE void logWarning(const QString& msg);
    Q_INVOKABLE void logError(const QString& msg);

    // Returns shared_ptr for renderer to obtain its own reference in synchronize().
    // Caller gets independent refcount → graph cannot be destroyed during render().
    std::shared_ptr<hdl_graph_slam::InteractiveGraph> sharedGraph() const;
    hdl_graph_slam::InteractiveGraph* graph() const;

public slots:
    void openMapData(const QUrl& folderUrl);
    void closeMap();

signals:
    void isLoadedChanged();
    void isLoadingChanged();
    void statsChanged();
    void lastMessageChanged(const QString& message);
    void loadingStarted();
    void loadingSucceeded();
    void loadingFailed(const QString& error);

private:
    void emitLog(LogLevel level, const QString& msg);

private:
    // Runs in worker thread. Returns shared_ptr (required by QFuture, which
    // does not support move-only types like unique_ptr in result()).
    static std::shared_ptr<hdl_graph_slam::InteractiveGraph> doLoad(
        const std::string& folderPath, ProgressReporter* progress);

    // Main-thread callback via QFutureWatcher
    void onLoadFinished();

    std::shared_ptr<hdl_graph_slam::InteractiveGraph> m_graph;
    ProgressReporter* m_progress;
    QFutureWatcher<std::shared_ptr<hdl_graph_slam::InteractiveGraph>>* m_loadWatcher;
    bool m_isLoaded = false;
    bool m_isLoading = false;
    QAtomicInt m_graphVersion{0};
    QString m_lastMessage;
    QString m_lastLogLevel = "INFO";
};
