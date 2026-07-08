#include "backend/graph_manager.hpp"
#include <QtConcurrent/QtConcurrent>
#include <QDateTime>

GraphManager::GraphManager(QObject* parent)
    : QObject(parent),
      m_progress(new ProgressReporter(this)),
      m_loadWatcher(new QFutureWatcher<std::shared_ptr<hdl_graph_slam::InteractiveGraph>>(this)) {
    connect(m_loadWatcher, &QFutureWatcher<std::shared_ptr<hdl_graph_slam::InteractiveGraph>>::finished,
            this, &GraphManager::onLoadFinished);
}

bool GraphManager::isLoaded() const { return m_isLoaded; }
bool GraphManager::isLoading() const { return m_isLoading; }
ProgressReporter* GraphManager::progress() const { return m_progress; }

int GraphManager::vertexCount() const {
    return m_graph ? m_graph->num_vertices() : 0;
}
int GraphManager::edgeCount() const {
    return m_graph ? m_graph->num_edges() : 0;
}
int GraphManager::keyframeCount() const {
    return m_graph ? static_cast<int>(m_graph->keyframes.size()) : 0;
}

QString GraphManager::lastMessage() const { return m_lastMessage; }
QString GraphManager::lastLogLevel() const { return m_lastLogLevel; }

void GraphManager::emitLog(LogLevel level, const QString& msg) {
    auto now = QDateTime::currentDateTime();
    QString ts = now.toString("HH:mm:ss");
    switch (level) {
    case WARNING: m_lastLogLevel = "WARNING"; break;
    case ERROR:   m_lastLogLevel = "ERROR";   break;
    default:      m_lastLogLevel = "INFO";    break;
    }
    m_lastMessage = QString("[%1] %2").arg(ts, msg);
    emit lastMessageChanged(m_lastMessage);
}

void GraphManager::logInfo(const QString& msg)    { emitLog(INFO, msg); }
void GraphManager::logWarning(const QString& msg) { emitLog(WARNING, msg); }
void GraphManager::logError(const QString& msg)   { emitLog(ERROR, msg); }

std::shared_ptr<hdl_graph_slam::InteractiveGraph> GraphManager::sharedGraph() const {
    return m_graph;
}

hdl_graph_slam::InteractiveGraph* GraphManager::graph() const {
    return m_graph.get();
}

void GraphManager::openMapData(const QUrl& folderUrl) {
    if (m_isLoading) {
        logWarning("Already loading, ignoring new request");
        return;
    }

    QString localPath = folderUrl.toLocalFile();
    if (localPath.isEmpty()) {
        logError("Invalid folder path");
        emit loadingFailed("Invalid folder path");
        return;
    }

    // Auto-close existing map before loading a new one
    if (m_isLoaded) {
        logInfo("Closing previous map before loading new one");
        m_graph.reset();
        m_isLoaded = false;
        m_graphVersion.ref();
        emit isLoadedChanged();
        emit statsChanged();
    }

    m_isLoading = true;
    emit isLoadingChanged();
    emit loadingStarted();
    logInfo(QString("Loading map: %1").arg(localPath));

    auto future = QtConcurrent::run(
        &GraphManager::doLoad,
        localPath.toStdString(),
        m_progress);

    m_loadWatcher->setFuture(future);
}

void GraphManager::closeMap() {
    if (!m_isLoaded) {
        logInfo("No map loaded");
        return;
    }
    logInfo("Closing map");
    m_graph.reset();
    m_isLoaded = false;
    m_graphVersion.ref();
    emit isLoadedChanged();
    emit statsChanged();
    logInfo("Map closed");
}

void GraphManager::onLoadFinished() {
    m_isLoading = false;
    emit isLoadingChanged();

    auto graph = m_loadWatcher->future().result();  // shared_ptr<InteractiveGraph>
    if (graph) {
        m_graph = graph;  // shared_ptr copy → refcount +1
        m_isLoaded = true;
        m_graphVersion.ref();
        emit isLoadedChanged();
        emit statsChanged();
        emit loadingSucceeded();
        logInfo(QString("Loaded: %1 vertices, %2 edges, %3 keyframes")
                    .arg(vertexCount()).arg(edgeCount()).arg(keyframeCount()));
    } else {
        logError("Failed to load map data");
        emit loadingFailed("Failed to load map data");
    }
}

std::shared_ptr<hdl_graph_slam::InteractiveGraph>
GraphManager::doLoad(const std::string& folderPath, ProgressReporter* progress) {
    auto graph = std::make_shared<hdl_graph_slam::InteractiveGraph>();
    bool ok = graph->load_map_data(folderPath, *progress);
    if (!ok) return nullptr;
    return graph;
}
