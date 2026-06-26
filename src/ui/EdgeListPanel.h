#pragma once

#include <QWidget>
#include <QTreeWidget>
#include <QPushButton>
#include <set>

#include "data/hdl_graph_slam/interactive_graph.hpp"

class GraphManager;

/// @brief Right-side dock panel listing all loop-closure edges with checkboxes
///        for per-edge visibility toggle and batch delete/hide operations.
///
///        Only edges whose EdgeSource is NOT Original are shown.
///        Checked = visible in 3D view; unchecked = hidden.
class EdgeListPanel : public QWidget {
    Q_OBJECT

public:
    explicit EdgeListPanel(GraphManager* manager, QWidget* parent = nullptr);

    /// IDs of edges the user has unchecked (hidden from renderer).
    const std::set<long>& hiddenEdgeIds() const { return m_hiddenEdgeIds; }

signals:
    /// Emitted whenever hidden-edge set changes — MainWindow refreshes viewport.
    void hiddenEdgesChanged();

public slots:
    /// Rebuild the list from current graph data.
    void refreshList();
    /// Clear the list (called on graph close).
    void clearList();

private slots:
    void onItemChanged(QTreeWidgetItem* item, int column);
    void onDeleteSelected();
    void onDeleteAll();
    void onHideAll();
    void onShowAll();

private:
    void setupUi();
    void addEdgeRow(long edgeId, long fromId, long toId,
                    hdl_graph_slam::EdgeSource source, double distance,
                    const std::string& kernel);

    GraphManager* m_manager;
    QTreeWidget* m_tree;
    QPushButton* m_deleteBtn;
    QPushButton* m_hideBtn;
    QPushButton* m_hideAllBtn;
    QPushButton* m_showAllBtn;

    std::set<long> m_hiddenEdgeIds;   // edge IDs hidden from rendering

    /// Guard to prevent signal loops during programmatic checkbox changes.
    bool m_updatingCheckState = false;
};
