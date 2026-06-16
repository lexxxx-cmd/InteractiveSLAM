#include "backend/graph_manager.hpp"
#include <QtConcurrent/QtConcurrent>

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

QString GraphManager::lastMessage() const { return {}; }

std::shared_ptr<hdl_graph_slam::InteractiveGraph> GraphManager::sharedGraph() const {
    return m_graph;
}

hdl_graph_slam::InteractiveGraph* GraphManager::graph() const {
    return m_graph.get();
}

void GraphManager::openMapData(const QUrl& folderUrl) {
    if (m_isLoading) return;

    QString localPath = folderUrl.toLocalFile();
    if (localPath.isEmpty()) {
        emit loadingFailed("Invalid folder path");
        return;
    }

    m_isLoading = true;
    emit isLoadingChanged();
    emit loadingStarted();

    auto future = QtConcurrent::run(
        &GraphManager::doLoad,
        localPath.toStdString(),
        m_progress);

    m_loadWatcher->setFuture(future);
}

void GraphManager::closeMap() {
    if (!m_isLoaded) return;
    m_graph.reset();
    m_isLoaded = false;
    m_graphVersion.ref();
    emit isLoadedChanged();
    emit statsChanged();
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
    } else {
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
