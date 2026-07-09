/**
 * @file EdgeListPanel.h
 * @brief 边列表面板头文件
 *
 * EdgeListPanel 是显示闭环边列表的浮动面板。
 * 它使用 QTreeWidget 列出图中所有非原始的闭环边，
 * 每行包含复选框（控制边在 3D 视口中的可见性）、
 * 边 ID、起点→终点、类型、距离和鲁棒核函数信息。
 *
 * 仅显示 EdgeSource 不是 Original 的边。
 * 选中 = 在 3D 视口中可见，未选中 = 隐藏。
 */

#pragma once

#include <QWidget>
#include <QTreeWidget>
#include <QPushButton>
#include <set>

#include "data/hdl_graph_slam/interactive_graph.hpp"

class GraphManager;

/**
 * @brief 闭环边列表面板
 *
 * 列出所有闭环边（手动/自动/锚点），每个条目带复选框控制可见性。
 * 提供批量删除、全部隐藏/显示等操作能力。
 *
 * 每当隐藏边集合发生变化时，发射 hiddenEdgesChanged 信号，
 * MainWindow 监听到后更新 ViewportWidget 的隐藏边集合。
 */
class EdgeListPanel : public QWidget {
    Q_OBJECT

public:
    explicit EdgeListPanel(GraphManager* manager, QWidget* parent = nullptr);

    /// 用户取消选中（隐藏）的边 ID 集合
    const std::set<long>& hiddenEdgeIds() const { return m_hiddenEdgeIds; }

signals:
    /// 隐藏边集合变化时发出 — MainWindow 刷新视口
    void hiddenEdgesChanged();

public slots:
    /// 从当前图谱数据重建列表
    void refreshList();
    /// 清空列表（关闭图谱时调用）
    void clearList();

private slots:
    void onItemChanged(QTreeWidgetItem* item, int column);  ///< 条目勾选状态变化
    void onDeleteSelected();  ///< 删除选中的边
    void onDeleteAll();       ///< 删除所有边
    void onHideAll();         ///< 隐藏所有边
    void onShowAll();         ///< 显示所有边

private:
    void setupUi();  ///< 创建 UI 控件

    /**
     * @brief 添加一行边信息
     * @param edgeId   边 ID
     * @param fromId   起点顶点 ID
     * @param toId     终点顶点 ID
     * @param source   边来源类型
     * @param distance 边长度
     * @param kernel   鲁棒核函数名称
     */
    void addEdgeRow(long edgeId, long fromId, long toId,
                    hdl_graph_slam::EdgeSource source, double distance,
                    const std::string& kernel);

    GraphManager* m_manager;  ///< 图数据管理器（非拥有指针）
    QTreeWidget* m_tree;      ///< 树形列表控件
    QPushButton* m_deleteBtn;     ///< "删除选中"按钮（实际删除）
    QPushButton* m_hideBtn;       ///< "全部删除"按钮（实际从图中删除所有）
    QPushButton* m_hideAllBtn;    ///< "全部隐藏"按钮（仅控制可见性）
    QPushButton* m_showAllBtn;    ///< "全部显示"按钮

    std::set<long> m_hiddenEdgeIds;  ///< 从渲染中隐藏的边 ID 集合

    /// 防止在程序化修改复选框状态时产生信号循环的守卫标志
    bool m_updatingCheckState = false;
};
