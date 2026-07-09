/**
 * @file GraphStatsPanel.h
 * @brief 图统计信息面板头文件
 *
 * GraphStatsPanel 是一个浮动面板，实时显示 SLAM 图优化系统的统计信息：
 * - 顶点数（Vertices）
 * - 边数（Edges）
 * - 关键帧数（Keyframes）
 * - 渲染帧率（FPS）
 * - 加载状态提示
 *
 * 通过连接 GraphManager 的 statsChanged 信号和 ViewportWidget 的 fpsUpdated 信号
 * 实现数据的实时更新。
 */

#pragma once

#include <QWidget>
#include <QLabel>

class ViewportWidget;
class GraphManager;

/**
 * @brief 图统计信息浮动面板
 *
 * 显示图谱的统计信息和 3D 视口的帧率。
 * 在加载过程中显示状态提示。
 */
class GraphStatsPanel : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param manager  图数据管理器
     * @param viewport 3D 视口部件（用于获取 FPS）
     * @param parent   父级 Qt 组件
     */
    explicit GraphStatsPanel(GraphManager* manager, ViewportWidget* viewport,
                             QWidget* parent = nullptr);

public slots:
    void onStatsChanged();          ///< 统计数据变化时更新显示
    void onLoadingStateChanged();   ///< 加载状态变化时更新显示

private:
    void setupUi();  ///< 创建 UI 控件

    GraphManager* m_manager;     ///< 图数据管理器
    ViewportWidget* m_viewport;  ///< 3D 视口部件

    QLabel* m_vertexCountLabel;     ///< 顶点数标签
    QLabel* m_edgeCountLabel;       ///< 边数标签
    QLabel* m_keyframeCountLabel;   ///< 关键帧数标签
    QLabel* m_fpsLabel;             ///< FPS 标签
    QLabel* m_loadingLabel;         ///< 加载状态提示标签
};
