/**
 * @file EdgeListPanel.cpp
 * @brief 边列表面板实现
 *
 * 实现闭环边的列表显示和管理功能：
 * - 从图谱中遍历 g2o 边，过滤出非原始的闭环边
 * - 每行显示边 ID、顶点关系、类型、距离、核函数
 * - 复选框控制单边可见性
 * - 批量操作：删除选中、全部删除、全部隐藏、全部显示
 */

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
// 构造
// ---------------------------------------------------------------------------

EdgeListPanel::EdgeListPanel(GraphManager* manager, QWidget* parent)
    : QWidget(parent), m_manager(manager) {
    setupUi();
}

// ---------------------------------------------------------------------------
// UI 设置
// ---------------------------------------------------------------------------

/**
 * @brief 创建 UI 布局
 *
 * 包含一个 QTreeWidget（6 列）和四个按钮。
 * 树形表的列：复选框、边 ID、From→To、类型、距离、核函数。
 */
void EdgeListPanel::setupUi() {
    auto* layout = new QVBoxLayout(this);
    layout->setContentsMargins(4, 4, 4, 4);

    // ---- 树形列表控件 ----
    m_tree = new QTreeWidget(this);
    m_tree->setHeaderLabels({
        QString(),                    // 第 0 列：复选框
        tr("Edge ID"),                // 第 1 列：边 ID
        tr("From → To"),              // 第 2 列：起点→终点
        tr("Type"),                   // 第 3 列：类型（Manual/Auto/Anchor）
        tr("Distance (m)"),           // 第 4 列：距离（米）
        tr("Kernel")                  // 第 5 列：鲁棒核函数
    });
    m_tree->setRootIsDecorated(false);
    m_tree->setSelectionMode(QAbstractItemView::ExtendedSelection);
    m_tree->setAlternatingRowColors(true);

    // 列宽设置
    m_tree->header()->setStretchLastSection(true);
    m_tree->header()->resizeSection(0, 30);   // 复选框
    m_tree->header()->resizeSection(1, 60);   // 边 ID
    m_tree->header()->resizeSection(2, 120);  // From → To
    m_tree->header()->resizeSection(3, 60);   // 类型
    m_tree->header()->resizeSection(4, 80);   // 距离

    // 勾选状态变化信号
    connect(m_tree, &QTreeWidget::itemChanged,
            this, &EdgeListPanel::onItemChanged);

    layout->addWidget(m_tree);

    // ---- 按钮区域 ----
    auto* btnLayout = new QHBoxLayout;

    m_deleteBtn = new QPushButton(tr("Delete Selected"), this);
    connect(m_deleteBtn, &QPushButton::clicked,
            this, &EdgeListPanel::onDeleteSelected);
    btnLayout->addWidget(m_deleteBtn);

    m_hideBtn = new QPushButton(tr("Delete All"), this);
    m_hideBtn->setObjectName("dangerButton");
    connect(m_hideBtn, &QPushButton::clicked,
            this, &EdgeListPanel::onDeleteAll);
    btnLayout->addWidget(m_hideBtn);

    m_hideAllBtn = new QPushButton(tr("Hide All"), this);
    m_hideAllBtn->setObjectName("tertiaryButton");
    connect(m_hideAllBtn, &QPushButton::clicked,
            this, &EdgeListPanel::onHideAll);
    btnLayout->addWidget(m_hideAllBtn);

    m_showAllBtn = new QPushButton(tr("Show All"), this);
    m_showAllBtn->setObjectName("tertiaryButton");
    connect(m_showAllBtn, &QPushButton::clicked,
            this, &EdgeListPanel::onShowAll);
    btnLayout->addWidget(m_showAllBtn);

    layout->addLayout(btnLayout);
}

// ---------------------------------------------------------------------------
// 公开槽函数
// ---------------------------------------------------------------------------

/**
 * @brief 刷新列表
 *
 * 遍历图谱中所有 g2o 边，过滤出非 Original 的闭环边，添加到树形列表。
 * 刷新过程中阻止信号发射以防止递归触发。
 */
void EdgeListPanel::refreshList() {
    auto* graph = m_manager->graph();
    if (!graph) { clearList(); return; }

    m_tree->blockSignals(true);   // 重建过程中阻止 itemChanged 信号
    m_tree->clear();

    // 获取底层的 g2o 优化器
    auto* g2oGraph = dynamic_cast<g2o::SparseOptimizer*>(graph->graph.get());
    if (!g2oGraph) { m_tree->blockSignals(false); return; }

    // 遍历所有边
    for (auto* edge : g2oGraph->edges()) {
        auto* se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
        if (!se3) continue;

        EdgeSource src = graph->edge_source(se3->id());

        // 仅显示非原始的边（原始边是 g2o 文件中自带的）
        if (src == EdgeSource::Original) continue;

        // 获取两个顶点
        auto* v1 = dynamic_cast<g2o::VertexSE3*>(se3->vertices()[0]);
        auto* v2 = dynamic_cast<g2o::VertexSE3*>(se3->vertices()[1]);
        if (!v1 || !v2) continue;

        // 计算距离
        Eigen::Vector3d p1 = v1->estimate().translation();
        Eigen::Vector3d p2 = v2->estimate().translation();
        double dist = (p1 - p2).norm();

        // 获取核函数名称
        std::string kernelName = g2o::kernel_type(se3->robustKernel());
        if (kernelName.empty()) kernelName = "NONE";

        addEdgeRow(se3->id(), v1->id(), v2->id(), src, dist, kernelName);
    }

    m_tree->blockSignals(false);

    // 通知 MainWindow 更新隐藏边指针
    emit hiddenEdgesChanged();
}

/**
 * @brief 清空列表
 */
void EdgeListPanel::clearList() {
    m_tree->blockSignals(true);
    m_tree->clear();
    m_tree->blockSignals(false);
    m_hiddenEdgeIds.clear();
}

// ---------------------------------------------------------------------------
// 私有辅助函数
// ---------------------------------------------------------------------------

/**
 * @brief 添加一行边信息
 *
 * 创建 QTreeWidgetItem，在 UserRole 中存储边 ID，
 * 设置各列文本：复选框状态、ID、起止顶点、类型、距离、核函数。
 */
void EdgeListPanel::addEdgeRow(long edgeId, long fromId, long toId,
                                EdgeSource source, double distance,
                                const std::string& kernel) {
    auto* item = new QTreeWidgetItem(m_tree);

    // 在 UserRole 中存储边 ID 供槽函数查找
    item->setData(0, Qt::UserRole, QVariant::fromValue(edgeId));

    // 第 0 列：复选框（选中=可见）
    bool isHidden = (m_hiddenEdgeIds.count(edgeId) > 0);
    item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
    item->setCheckState(0, isHidden ? Qt::Unchecked : Qt::Checked);

    // 第 1 列：边 ID
    item->setText(1, QString::number(edgeId));

    // 第 2 列：From → To
    item->setText(2, QString("%1 → %2").arg(fromId).arg(toId));

    // 第 3 列：类型（tr 化：面向用户的分类标签，spec C-3.4 拍板翻译）
    QString typeStr = tr("Unknown");
    switch (source) {
        case EdgeSource::ManualLoop: typeStr = tr("Manual"); break;
        case EdgeSource::AutoLoop:   typeStr = tr("Auto");   break;
        case EdgeSource::Anchor:     typeStr = tr("Anchor"); break;
        default: break;
    }
    item->setText(3, typeStr);

    // 第 4 列：距离
    item->setText(4, QString::number(distance, 'f', 2));

    // 第 5 列：核函数
    item->setText(5, QString::fromStdString(kernel));
}

// ---------------------------------------------------------------------------
// 私有槽函数
// ---------------------------------------------------------------------------

/**
 * @brief 复选框状态变化
 *
 * 当用户勾选/取消勾选某行复选框时，
 * 更新 m_hiddenEdgeIds 集合并发射 hiddenEdgesChanged 信号。
 */
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

/**
 * @brief 删除选中的边
 *
 * 先收集所有选中项的 ID，然后逐个从图中删除。
 * 删除后刷新列表。
 */
void EdgeListPanel::onDeleteSelected() {
    auto* graph = m_manager->graph();
    if (!graph) return;

    auto selected = m_tree->selectedItems();
    if (selected.isEmpty()) return;

    // 先收集 ID 再删除——在遍历过程中删除元素是不安全的
    QVector<long> idsToDelete;
    for (auto* item : selected) {
        idsToDelete.append(item->data(0, Qt::UserRole).value<long>());
    }

    for (long id : idsToDelete) {
        graph->removeEdge(id);
        m_hiddenEdgeIds.erase(id);
    }

    refreshList();   // 也发射 hiddenEdgesChanged
}

/**
 * @brief 删除所有边
 *
 * 收集列表中所有边的 ID，从图中全部删除。
 */
void EdgeListPanel::onDeleteAll() {
    auto* graph = m_manager->graph();
    if (!graph) return;

    // 收集面板中所有边 ID
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

/**
 * @brief 隐藏所有边
 *
 * 将所有边的复选框设为未选中（隐藏），不删除边。
 */
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

/**
 * @brief 显示所有边
 *
 * 将所有边的复选框设为选中（显示），清空隐藏集合。
 */
void EdgeListPanel::onShowAll() {
    m_updatingCheckState = true;
    for (int i = 0; i < m_tree->topLevelItemCount(); ++i) {
        m_tree->topLevelItem(i)->setCheckState(0, Qt::Checked);
    }
    m_updatingCheckState = false;

    m_hiddenEdgeIds.clear();
    emit hiddenEdgesChanged();
}
