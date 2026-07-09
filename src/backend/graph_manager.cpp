// ============================================================================
// graph_manager.cpp
// 图管理器实现文件
//
// 本文件实现了 GraphManager 类的所有功能，包括：
//   - 图数据的异步加载与关闭
//   - 进度报告与日志输出
//   - 图统计信息的查询接口
//   - 多线程安全的数据访问控制
// ============================================================================

#include "backend/graph_manager.hpp"
#include <QtConcurrent/QtConcurrent>
#include <QDateTime>

// ============================================================================
// 构造函数
// ============================================================================

/**
 * @brief 构造函数：初始化进度报告器、加载监视器，并连接加载完成信号
 * @param parent Qt 父对象
 */
GraphManager::GraphManager(QObject* parent)
    : QObject(parent),
      m_progress(new ProgressReporter(this)),           // 创建进度报告器实例
      m_loadWatcher(new QFutureWatcher<std::shared_ptr<hdl_graph_slam::InteractiveGraph>>(this)) {
    // 连接 QFutureWatcher 的 finished 信号到 onLoadFinished 槽函数
    // 当工作线程完成加载后，自动在主线程中处理加载结果
    connect(m_loadWatcher, &QFutureWatcher<std::shared_ptr<hdl_graph_slam::InteractiveGraph>>::finished,
            this, &GraphManager::onLoadFinished);
}

// ============================================================================
// 状态查询方法（内联实现）
// ============================================================================

/** @brief 返回图数据是否已加载 */
bool GraphManager::isLoaded() const { return m_isLoaded; }
/** @brief 返回是否正在加载中 */
bool GraphManager::isLoading() const { return m_isLoading; }
/** @brief 返回进度报告器指针 */
ProgressReporter* GraphManager::progress() const { return m_progress; }

/**
 * @brief 获取当前图中的顶点数量
 * @return 顶点数（图未加载时返回 0）
 */
int GraphManager::vertexCount() const {
    return m_graph ? m_graph->num_vertices() : 0;
}

/**
 * @brief 获取当前图中的边的数量
 * @return 边数（图未加载时返回 0）
 */
int GraphManager::edgeCount() const {
    return m_graph ? m_graph->num_edges() : 0;
}

/**
 * @brief 获取当前图中的关键帧数量
 * @return 关键帧数（图未加载时返回 0）
 */
int GraphManager::keyframeCount() const {
    return m_graph ? static_cast<int>(m_graph->keyframes.size()) : 0;
}

/** @brief 返回最后一条日志消息 */
QString GraphManager::lastMessage() const { return m_lastMessage; }
/** @brief 返回最后一条日志的级别字符串 */
QString GraphManager::lastLogLevel() const { return m_lastLogLevel; }

// ============================================================================
// 日志系统
// ============================================================================

/**
 * @brief 发射带时间戳的日志消息信号
 *
 * 为日志消息添加 [HH:mm:ss] 格式的时间戳前缀，
 * 并更新最后日志级别和消息内容，最后发射信号通知前端。
 *
 * @param level 日志级别（INFO/WARNING/ERROR）
 * @param msg   日志消息内容
 */
void GraphManager::emitLog(LogLevel level, const QString& msg) {
    auto now = QDateTime::currentDateTime();
    QString ts = now.toString("HH:mm:ss");
    // 根据日志级别设置对应的级别字符串
    switch (level) {
    case WARNING: m_lastLogLevel = "WARNING"; break;
    case ERROR:   m_lastLogLevel = "ERROR";   break;
    default:      m_lastLogLevel = "INFO";    break;
    }
    m_lastMessage = QString("[%1] %2").arg(ts, msg);
    emit lastMessageChanged(m_lastMessage);
}

/** @brief 信息级别日志 */
void GraphManager::logInfo(const QString& msg)    { emitLog(INFO, msg); }
/** @brief 警告级别日志 */
void GraphManager::logWarning(const QString& msg) { emitLog(WARNING, msg); }
/** @brief 错误级别日志 */
void GraphManager::logError(const QString& msg)   { emitLog(ERROR, msg); }

// ============================================================================
// 图数据访问
// ============================================================================

/**
 * @brief 获取共享图指针（增加引用计数）
 *
 * 返回 shared_ptr 的副本，使调用方拥有独立的引用计数。
 * 这是渲染器获取图数据的推荐方式，可确保渲染期间图对象不会被销毁。
 *
 * @return InteractiveGraph 的 shared_ptr 副本
 */
std::shared_ptr<hdl_graph_slam::InteractiveGraph> GraphManager::sharedGraph() const {
    return m_graph;
}

/**
 * @brief 获取原始图指针（不增加引用计数）
 * @return InteractiveGraph 的原始指针
 */
hdl_graph_slam::InteractiveGraph* GraphManager::graph() const {
    return m_graph.get();
}

// ============================================================================
// 公共槽函数：打开/关闭地图
// ============================================================================

/**
 * @brief 从文件夹 URL 异步加载地图数据
 *
 * 加载流程：
 *   1. 检查是否已在加载中（防止重复加载）
 *   2. 将 URL 转换为本地路径并验证有效性
 *   3. 如果已有打开的地图，自动关闭
 *   4. 通过 QtConcurrent::run 在工作线程中执行 doLoad()
 *   5. 将 future 设置到监视器中，等待完成回调
 *
 * @param folderUrl 地图数据文件夹的 URL
 */
void GraphManager::openMapData(const QUrl& folderUrl) {
    // 防止重复加载
    if (m_isLoading) {
        logWarning("Already loading, ignoring new request");
        return;
    }

    // 将 URL 转换为本地文件系统路径
    QString localPath = folderUrl.toLocalFile();
    if (localPath.isEmpty()) {
        logError("Invalid folder path");
        emit loadingFailed("Invalid folder path");
        return;
    }

    // 加载新地图前自动关闭已有地图
    if (m_isLoaded) {
        logInfo("Closing previous map before loading new one");
        m_graph.reset();          // 释放图数据（如果没有其他引用）
        m_isLoaded = false;
        m_graphVersion.ref();     // 递增版本号，通知渲染器图数据已变更
        emit isLoadedChanged();
        emit statsChanged();
    }

    // 更新加载状态
    m_isLoading = true;
    emit isLoadingChanged();
    emit loadingStarted();
    logInfo(QString("Loading map: %1").arg(localPath));

    // 在工作线程中执行加载，避免阻塞主线程（UI 界面）
    auto future = QtConcurrent::run(
        &GraphManager::doLoad,
        localPath.toStdString(),
        m_progress);

    // 将 future 设置到监视器中，当工作线程完成时会触发 onLoadFinished
    m_loadWatcher->setFuture(future);
}

/**
 * @brief 关闭当前已加载的地图
 *
 * 释放图数据、重置状态并发射相应信号。
 */
void GraphManager::closeMap() {
    if (!m_isLoaded) {
        logInfo("No map loaded");
        return;
    }
    logInfo("Closing map");
    m_graph.reset();          // 释放图数据
    m_isLoaded = false;
    m_graphVersion.ref();     // 递增版本号
    emit isLoadedChanged();
    emit statsChanged();
    logInfo("Map closed");
}

// ============================================================================
// 加载完成回调（主线程执行）
// ============================================================================

/**
 * @brief 加载完成后的主线程回调
 *
 * 由 QFutureWatcher::finished 信号触发，在工作线程完成加载后
 * 在主线程中执行。负责将加载结果安装到主线程的成员变量中，
 * 并发射成功/失败信号。
 */
void GraphManager::onLoadFinished() {
    m_isLoading = false;
    emit isLoadingChanged();

    // 获取工作线程返回的 shared_ptr<InteractiveGraph>
    auto graph = m_loadWatcher->future().result();
    if (graph) {
        // 加载成功：安装图数据并更新状态
        m_graph = graph;          // shared_ptr 拷贝 → 引用计数 +1
        m_isLoaded = true;
        m_graphVersion.ref();     // 递增版本号
        emit isLoadedChanged();
        emit statsChanged();
        emit loadingSucceeded();
        logInfo(QString("Loaded: %1 vertices, %2 edges, %3 keyframes")
                    .arg(vertexCount()).arg(edgeCount()).arg(keyframeCount()));
    } else {
        // 加载失败
        logError("Failed to load map data");
        emit loadingFailed("Failed to load map data");
    }
}

// ============================================================================
// 工作线程加载函数（静态）
// ============================================================================

/**
 * @brief 在工作线程中执行的实际加载函数
 *
 * 这是一个静态方法，通过 QtConcurrent::run 在工作线程中调用。
 * 创建 InteractiveGraph 实例并调用其 load_map_data() 方法。
 *
 * @param folderPath 地图数据文件夹路径
 * @param progress   进度报告器指针
 * @return 加载成功返回图数据共享指针，失败返回 nullptr
 */
std::shared_ptr<hdl_graph_slam::InteractiveGraph>
GraphManager::doLoad(const std::string& folderPath, ProgressReporter* progress) {
    // 创建图实例并加载地图数据
    auto graph = std::make_shared<hdl_graph_slam::InteractiveGraph>();
    bool ok = graph->load_map_data(folderPath, *progress);
    if (!ok) return nullptr;
    return graph;
}
