#include "ui/EdgeListPanel.h"
#include "backend/graph_manager.hpp"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QPushButton>

#include <g2o/core/sparse_optimizer.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include "data/g2o/robust_kernel_io.hpp"

using hdl_graph_slam::EdgeSource;

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

EdgeListPanel::EdgeListPanel(GraphManager* manager, QWidget* parent)
    : QWidget(parent), m_manager(manager) {
    setupUi();
}

// ---------------------------------------------------------------------------
// UI Setup
// ---------------------------------------------------------------------------

void EdgeListPanel::setupUi() {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);

    // ── Tree widget ───────────────────────────────────────────────
    m_tree = new QTreeWidget(this);
    m_tree->setHeaderLabels({
        QString(),                    // 0: checkbox
        tr("Edge ID"),                // 1
        tr("From → To"),              // 2
        tr("Type"),                   // 3
        tr("Distance (m)"),           // 4
        tr("Kernel")                  // 5
    });
    m_tree->setRootIsDecorated(false);
    m_tree->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_tree->setAlternatingRowColors(true);

    // Column widths
    m_tree->header()->setStretchLastSection(true);
    m_tree->header()->resizeSection(0, 30);   // checkbox
    m_tree->header()->resizeSection(1, 60);   // Edge ID
    m_tree->header()->resizeSection(2, 120);  // From → To
    m_tree->header()->resizeSection(3, 60);   // Type
    m_tree->header()->resizeSection(4, 80);   // Distance

    connect(m_tree, &QTreeWidget::itemChanged,
            this, &EdgeListPanel::onItemChanged);

    layout->addWidget(m_tree);

    // ── Buttons ───────────────────────────────────────────────────
    auto* btnLayout = new QHBoxLayout;

    m_deleteBtn = new QPushButton(tr("Delete Selected"), this);
    connect(m_deleteBtn, &QPushButton::clicked,
            this, &EdgeListPanel::onDeleteSelected);
    btnLayout->addWidget(m_deleteBtn);

    m_hideBtn = new QPushButton(tr("Delete All"), this);
    connect(m_hideBtn, &QPushButton::clicked,
            this, &EdgeListPanel::onDeleteAll);
    btnLayout->addWidget(m_hideBtn);

    m_hideAllBtn = new QPushButton(tr("Hide All"), this);
    connect(m_hideAllBtn, &QPushButton::clicked,
            this, &EdgeListPanel::onHideAll);
    btnLayout->addWidget(m_hideAllBtn);

    m_showAllBtn = new QPushButton(tr("Show All"), this);
    connect(m_showAllBtn, &QPushButton::clicked,
            this, &EdgeListPanel::onShowAll);
    btnLayout->addWidget(m_showAllBtn);

    layout->addLayout(btnLayout);
}

// ---------------------------------------------------------------------------
// Public Slots
// ---------------------------------------------------------------------------

void EdgeListPanel::refreshList() {
    auto* graph = m_manager->graph();
    if (!graph) { clearList(); return; }

    m_tree->blockSignals(true);   // prevent itemChanged during rebuild
    m_tree->clear();

    auto* g2oGraph = dynamic_cast<g2o::SparseOptimizer*>(graph->graph.get());
    if (!g2oGraph) { m_tree->blockSignals(false); return; }

    for (auto* edge : g2oGraph->edges()) {
        auto* se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
        if (!se3) continue;

        EdgeSource src = graph->edge_source(se3->id());

        // Only show non-original edges in this panel
        if (src == EdgeSource::Original) continue;

        auto* v1 = dynamic_cast<g2o::VertexSE3*>(se3->vertices()[0]);
        auto* v2 = dynamic_cast<g2o::VertexSE3*>(se3->vertices()[1]);
        if (!v1 || !v2) continue;

        Eigen::Vector3d p1 = v1->estimate().translation();
        Eigen::Vector3d p2 = v2->estimate().translation();
        double dist = (p1 - p2).norm();

        std::string kernelName = g2o::kernel_type(se3->robustKernel());
        if (kernelName.empty()) kernelName = "NONE";

        addEdgeRow(se3->id(), v1->id(), v2->id(), src, dist, kernelName);
    }

    m_tree->blockSignals(false);

    // Emit so MainWindow can update hidden-edges pointer
    emit hiddenEdgesChanged();
}

void EdgeListPanel::clearList() {
    m_tree->blockSignals(true);
    m_tree->clear();
    m_tree->blockSignals(false);
    m_hiddenEdgeIds.clear();
}

// ---------------------------------------------------------------------------
// Private Helpers
// ---------------------------------------------------------------------------

void EdgeListPanel::addEdgeRow(long edgeId, long fromId, long toId,
                                EdgeSource source, double distance,
                                const std::string& kernel) {
    auto* item = new QTreeWidgetItem(m_tree);

    // Store edge ID in UserRole for lookup in slot handlers
    item->setData(0, Qt::UserRole, QVariant::fromValue(edgeId));

    // Column 0: checkbox (checked = visible)
    bool isHidden = (m_hiddenEdgeIds.count(edgeId) > 0);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(0, isHidden ? Qt::Unchecked : Qt::Checked);

    // Column 1: Edge ID
    item->setText(1, QString::number(edgeId));

    // Column 2: From → To
    item->setText(2, QString("%1 → %2").arg(fromId).arg(toId));

    // Column 3: Type
    const char* typeStr = "Unknown";
    switch (source) {
        case EdgeSource::ManualLoop: typeStr = "Manual"; break;
        case EdgeSource::AutoLoop:   typeStr = "Auto";   break;
        case EdgeSource::Anchor:     typeStr = "Anchor"; break;
        default: break;
    }
    item->setText(3, QString::fromUtf8(typeStr));

    // Column 4: Distance
    item->setText(4, QString::number(distance, 'f', 2));

    // Column 5: Kernel
    item->setText(5, QString::fromStdString(kernel));
}

// ---------------------------------------------------------------------------
// Private Slots
// ---------------------------------------------------------------------------

void EdgeListPanel::onItemChanged(QTreeWidgetItem* item, int column) {
    if (column != 0) return;
    if (m_updatingCheckState) return;

    long edgeId = item->data(0, Qt::UserRole).value<long>();
    if (item->checkState(0) == Qt::Unchecked) {
        m_hiddenEdgeIds.insert(edgeId);
    } else {
        m_hiddenEdgeIds.erase(edgeId);
    }
    emit hiddenEdgesChanged();
}

void EdgeListPanel::onDeleteSelected() {
    auto* graph = m_manager->graph();
    if (!graph) return;

    auto selected = m_tree->selectedItems();
    if (selected.isEmpty()) return;

    // Collect IDs first — deleting items during iteration is unsafe
    QVector<long> idsToDelete;
    for (auto* item : selected) {
        idsToDelete.append(item->data(0, Qt::UserRole).value<long>());
    }

    for (long id : idsToDelete) {
        graph->removeEdge(id);
        m_hiddenEdgeIds.erase(id);
    }

    refreshList();   // also emits hiddenEdgesChanged
}

void EdgeListPanel::onDeleteAll() {
    auto* graph = m_manager->graph();
    if (!graph) return;

    // Collect all edge IDs in the panel
    QVector<long> idsToDelete;
    for (int i = 0; i < m_tree->topLevelItemCount(); ++i) {
        idsToDelete.append(m_tree->topLevelItem(i)->data(0, Qt::UserRole).value<long>());
    }

    for (long id : idsToDelete) {
        graph->removeEdge(id);
        m_hiddenEdgeIds.erase(id);
    }

    refreshList();
}

void EdgeListPanel::onHideAll() {
    m_updatingCheckState = true;
    for (int i = 0; i < m_tree->topLevelItemCount(); ++i) {
        auto* item = m_tree->topLevelItem(i);
        item->setCheckState(0, Qt::Unchecked);
        long edgeId = item->data(0, Qt::UserRole).value<long>();
        m_hiddenEdgeIds.insert(edgeId);
    }
    m_updatingCheckState = false;
    emit hiddenEdgesChanged();
}

void EdgeListPanel::onShowAll() {
    m_updatingCheckState = true;
    for (int i = 0; i < m_tree->topLevelItemCount(); ++i) {
        m_tree->topLevelItem(i)->setCheckState(0, Qt::Checked);
    }
    m_updatingCheckState = false;

    m_hiddenEdgeIds.clear();
    emit hiddenEdgesChanged();
}
