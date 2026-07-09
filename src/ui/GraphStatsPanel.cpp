/**
 * @file GraphStatsPanel.cpp
 * @brief 图统计信息面板实现
 *
 * 实现统计面板的 UI 搭建和信号连接。
 * 通过 GraphManager 的信号获取顶点/边/关键帧计数，
 * 通过 ViewportWidget 的信号获取 FPS 数据。
 */

#include "ui/GraphStatsPanel.h"
#include "ui/ViewportWidget.h"
#include "backend/graph_manager.hpp"

#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>

/**
 * @brief 构造函数
 *
 * 连接 GraphManager 的 statsChanged 和 isLoadingChanged 信号，
 * 连接 ViewportWidget 的 fpsUpdated 信号，初始化显示。
 */
GraphStatsPanel::GraphStatsPanel(GraphManager* manager, ViewportWidget* viewport,
                                 QWidget* parent)
    : QWidget(parent), m_manager(manager), m_viewport(viewport) {
    setupUi();

    // 统计信息更新
    connect(m_manager, &GraphManager::statsChanged,
            this, &GraphStatsPanel::onStatsChanged);
    connect(m_manager, &GraphManager::isLoadingChanged,
            this, &GraphStatsPanel::onLoadingStateChanged);

    // 视口 FPS 更新
    connect(m_viewport, &ViewportWidget::fpsUpdated, this, [this](float fps) {
        m_fpsLabel->setText(QString::number(fps, 'f', 1));
    });

    // 初始显示
    onStatsChanged();
}

/**
 * @brief 创建 UI 布局
 *
 * 使用 QFormLayout 显示标签-值对，
 * 包含顶点数、边数、关键帧数和 FPS。
 * 底部有加载状态提示标签（默认隐藏）。
 */
void GraphStatsPanel::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(8, 8, 8, 8);

    auto* statsGroup = new QGroupBox(tr("Graph Statistics"));
    auto* statsForm = new QFormLayout(statsGroup);

    // 创建数值标签，初始为 0
    m_vertexCountLabel = new QLabel("0");
    m_edgeCountLabel = new QLabel("0");
    m_keyframeCountLabel = new QLabel("0");
    m_fpsLabel = new QLabel("0.0");

    statsForm->addRow(tr("Vertices:"), m_vertexCountLabel);
    statsForm->addRow(tr("Edges:"), m_edgeCountLabel);
    statsForm->addRow(tr("Keyframes:"), m_keyframeCountLabel);
    statsForm->addRow(tr("FPS:"), m_fpsLabel);

    mainLayout->addWidget(statsGroup);

    // 加载状态指示器（橙色粗体，默认隐藏）
    m_loadingLabel = new QLabel;
    m_loadingLabel->setVisible(false);
    m_loadingLabel->setStyleSheet("color: #f59e0b; font-weight: bold;");
    mainLayout->addWidget(m_loadingLabel);

    mainLayout->addStretch();
}

/**
 * @brief 统计数据变化时更新显示
 */
void GraphStatsPanel::onStatsChanged() {
    m_vertexCountLabel->setText(QString::number(m_manager->vertexCount()));
    m_edgeCountLabel->setText(QString::number(m_manager->edgeCount()));
    m_keyframeCountLabel->setText(QString::number(m_manager->keyframeCount()));
}

/**
 * @brief 加载状态变化时显示/隐藏提示
 */
void GraphStatsPanel::onLoadingStateChanged() {
    bool loading = m_manager->isLoading();
    m_loadingLabel->setVisible(loading);
    if (loading) {
        m_loadingLabel->setText(tr("Loading..."));
    }
}
