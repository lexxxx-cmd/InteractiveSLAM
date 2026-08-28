#pragma once

#include <QObject>
#include <QUrl>
#include <QFutureWatcher>
#include <memory>
#include <QAtomicInt>

#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "data/hdl_graph_slam/bag_importer.hpp"
#include "backend/progress_reporter.hpp"

/**
 * @brief 图数据管理器 —— 将 InteractiveGraph 数据层桥接到 QML 界面。
 *
 * 核心职责：
 *   1. 封装 InteractiveGraph 的加载、关闭生命周期
 *   2. 通过 Qt 属性系统暴露图统计信息（顶点数、边数等），供 QML 界面绑定
 *   3. 提供跨线程安全的图数据访问，确保渲染器在渲染期间图数据不会被销毁
 *
 * 线程安全设计：
 *   - openMapData() 通过 QtConcurrent::run 在工作线程中加载数据
 *   - onLoadFinished() 在主线程中执行，安装 shared_ptr<InteractiveGraph>
 *   - 渲染器通过 sharedGraph() 在 synchronize() 中获取独立引用（主线程被阻塞）
 *   - shared_ptr 确保渲染期间图对象不会被销毁
 *
 * 信号-槽机制：
 *   - 加载进度、状态变化通过信号通知前端更新
 *   - 日志消息通过 lastMessageChanged 信号传递给 UI
 */
class GraphManager : public QObject {
    Q_OBJECT

    // —— Qt 属性（供 QML 界面绑定）——
    Q_PROPERTY(bool isLoaded READ isLoaded NOTIFY isLoadedChanged)            ///< 是否已加载图数据
    Q_PROPERTY(int vertexCount READ vertexCount NOTIFY statsChanged)          ///< 当前图中的顶点数量
    Q_PROPERTY(int edgeCount READ edgeCount NOTIFY statsChanged)              ///< 当前图中的边的数量
    Q_PROPERTY(int keyframeCount READ keyframeCount NOTIFY statsChanged)      ///< 当前图中的关键帧数量
    Q_PROPERTY(QString lastMessage READ lastMessage NOTIFY lastMessageChanged)     ///< 最后一条日志消息内容
    Q_PROPERTY(QString lastLogLevel READ lastLogLevel NOTIFY lastMessageChanged)   ///< 最后一条日志消息级别
    Q_PROPERTY(bool isLoading READ isLoading NOTIFY isLoadingChanged)         ///< 是否正在加载数据
    Q_PROPERTY(ProgressReporter* progress READ progress CONSTANT)             ///< 进度报告器对象

public:
    /**
     * @brief 日志级别枚举
     */
    enum LogLevel { INFO, WARNING, ERROR };
    Q_ENUM(LogLevel)

    /**
     * @brief 构造函数
     * @param parent Qt 父对象
     */
    explicit GraphManager(QObject* parent = nullptr);

    // —— 状态查询 ——
    bool isLoaded() const;            ///< 返回图数据是否已加载
    bool isLoading() const;           ///< 返回是否正在加载中
    int vertexCount() const;          ///< 返回当前图中的顶点数
    int edgeCount() const;            ///< 返回当前图中的边数
    int keyframeCount() const;        ///< 返回当前图中的关键帧数
    QString lastMessage() const;      ///< 返回最后一条日志消息
    QString lastLogLevel() const;     ///< 返回最后一条日志的级别字符串
    ProgressReporter* progress() const; ///< 返回进度报告器指针

    // —— 日志接口 ——
    void logInfo(const QString& msg);     ///< 记录信息级别日志
    void logWarning(const QString& msg);  ///< 记录警告级别日志
    void logError(const QString& msg);    ///< 记录错误级别日志

    /**
     * @brief 获取共享图指针（供渲染器使用）
     *
     * 返回 shared_ptr 的副本，使调用方获得独立的引用计数。
     * 这保证在图正在渲染时，即使主线程触发了 closeMap()，图对象也不会被销毁。
     *
     * @return InteractiveGraph 的 shared_ptr 副本
     */
    std::shared_ptr<hdl_graph_slam::InteractiveGraph> sharedGraph() const;

    /**
     * @brief 获取原始图指针（不增加引用计数）
     * @return InteractiveGraph 的原始指针，调用方需确保图在使用期间存活
     */
    hdl_graph_slam::InteractiveGraph* graph() const;

public slots:
    /**
     * @brief 从文件夹 URL 打开地图数据（异步加载）
     * @param folderUrl 地图数据文件夹的 URL 路径
     *
     * 内部通过 QtConcurrent::run 启动工作线程，加载完成后自动回调到主线程。
     * 如果已有加载中的操作，忽略新请求；如果已有打开的地图，自动关闭。
     */
    void openMapData(const QUrl& folderUrl);

    /** @brief 当前地图的来源目录（openMapData 打开时的路径，保存时预填用） */
    QString mapSourceDir() const { return m_mapSourceDir; }

    /**
     * @brief 打开 ROS1 bag 并解析为标准地图（异步，配置由 YAML 控制）
     * @param bagUrl      bag 文件 URL
     * @param yamlPath    导入配置文件路径（config/bag_import.yaml；空则用默认参数）
     * @param odomTopic   位姿 topic（非空则覆盖 yaml 配置，来自用户选择）
     * @param cloudTopic  点云 topic（非空则覆盖 yaml 配置，来自用户选择）
     *
     * 后台线程运行 BagImporter::import（按 SCPGO 数据流：解析 topic、
     * 时间同步、关键帧抽稀、外参变换、导出 graph.g2o + keyframes），
     * 完成后自动调用 openMapData() 加载生成的地图目录。
     */
    void openBagFile(const QUrl& bagUrl, const QString& yamlPath,
                     const QString& odomTopic = QString(),
                     const QString& cloudTopic = QString());

    /**
     * @brief 打开 ROS1 bag 并解析为标准地图（异步，配置由调用方提供）
     * @param bagUrl bag 文件 URL
     * @param cfg    完整导入配置（由 UI 弹窗编辑后的 BagImportConfig）
     *
     * 与 openBagFile(bagUrl, yamlPath, ...) 等价，但直接使用调用方传入的
     * 完整配置（topic/外参/抽稀/输出），不再从 yaml 读取。输出目录为空时
     * 使用临时目录。
     */
    void openBagFile(const QUrl& bagUrl, const hdl_graph_slam::BagImportConfig& cfg);

    /**
     * @brief 关闭当前已加载的地图
     *
     * 重置图数据指针，更新状态并发出相应信号。
     */
    void closeMap();

signals:
    /** @brief 加载状态变化信号（已加载/未加载） */
    void isLoadedChanged();
    /** @brief 正在加载状态变化信号 */
    void isLoadingChanged();
    /** @brief 图统计信息（顶点/边/关键帧数）变化信号 */
    void statsChanged();
    /** @brief 最后一条日志消息变化信号 */
    void lastMessageChanged(const QString& message);
    /** @brief 开始加载信号 */
    void loadingStarted();
    /** @brief 加载成功信号 */
    void loadingSucceeded();
    /** @brief 加载失败信号，携带错误描述 */
    void loadingFailed(const QString& error);

private:
    /**
     * @brief 发射带时间戳的日志信号
     * @param level 日志级别
     * @param msg   日志消息内容
     */
    void emitLog(LogLevel level, const QString& msg);

private:
    /**
     * @brief 在工作线程中执行的实际加载函数（静态）
     *
     * 返回 shared_ptr 而非 unique_ptr，因为 QFuture::result()
     * 不支持 move-only 类型。
     *
     * @param folderPath 地图数据文件夹路径
     * @param progress   进度报告器指针（用于报告加载进度）
     * @return 加载成功返回文件共享指针，失败返回 nullptr
     */
    static std::shared_ptr<hdl_graph_slam::InteractiveGraph> doLoad(
        const std::string& folderPath, ProgressReporter* progress);

    /**
     * @brief 加载完成后的主线程回调
     *
     * 由 QFutureWatcher 的 finished 信号触发，负责将工作线程
     * 加载的结果安装到主线程，并发射相应信号。
     */
    void onLoadFinished();

    /**
     * @brief bag 导入完成后的主线程回调
     *
     * 由 QFutureWatcher 的 finished 信号触发；成功则自动调用
     * openMapData() 加载生成的地图目录。
     */
    void onBagImportFinished();

    // —— 成员变量 ——
    std::shared_ptr<hdl_graph_slam::InteractiveGraph> m_graph;  ///< 图数据共享指针（线程安全引用）
    ProgressReporter* m_progress;                                ///< 进度报告器（用于显示加载进度）
    QFutureWatcher<std::shared_ptr<hdl_graph_slam::InteractiveGraph>>* m_loadWatcher; ///< 异步加载的 future 监视器
    QFutureWatcher<hdl_graph_slam::BagImportResult>* m_bagWatcher = nullptr; ///< bag 导入 future 监视器
    bool m_isLoaded = false;         ///< 标记图数据是否已成功加载
    bool m_isLoading = false;        ///< 标记是否正在执行加载操作
    bool m_isImportingBag = false;   ///< 标记是否正在执行 bag 导入
    QString m_mapSourceDir;          ///< 当前地图来源目录（openMapData 时记录）
    QAtomicInt m_graphVersion{0};    ///< 图数据版本号（每次加载/关闭时递增，用于渲染器同步检测）
    QString m_lastMessage;           ///< 缓存最后一条日志消息内容
    QString m_lastLogLevel = "INFO"; ///< 缓存最后一条日志消息的级别
};
