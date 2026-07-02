#include "ui/GraphStatsPanel.h"
#include "ui/ViewportWidget.h"
#include "backend/graph_manager.hpp"

#include <QVBoxLayout>
#include <QGroupBox>
#include <QFormLayout>

GraphStatsPanel::GraphStatsPanel(GraphManager* manager, ViewportWidget* viewport,
                                 QWidget* parent)
    : QWidget(parent), m_manager(manager), m_viewport(viewport) {
    setupUi();

    // Statistics updates
    connect(m_manager, &GraphManager::statsChanged,
            this, &GraphStatsPanel::onStatsChanged);
    connect(m_manager, &GraphManager::isLoadingChanged,
            this, &GraphStatsPanel::onLoadingStateChanged);

    // FPS from viewport
    connect(m_viewport, &ViewportWidget::fpsUpdated, this, [this](float fps) {
        m_fpsLabel->setText(QString::number(fps, 'f', 1));
    });

    onStatsChanged();
}

void GraphStatsPanel::setupUi() {
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(8, 8, 8, 8);

    auto* statsGroup = new QGroupBox(tr("Graph Statistics"));
    auto* statsForm = new QFormLayout(statsGroup);

    m_vertexCountLabel = new QLabel("0");
    m_edgeCountLabel = new QLabel("0");
    m_keyframeCountLabel = new QLabel("0");
    m_fpsLabel = new QLabel("0.0");

    statsForm->addRow(tr("Vertices:"), m_vertexCountLabel);
    statsForm->addRow(tr("Edges:"), m_edgeCountLabel);
    statsForm->addRow(tr("Keyframes:"), m_keyframeCountLabel);
    statsForm->addRow(tr("FPS:"), m_fpsLabel);

    mainLayout->addWidget(statsGroup);

    // Loading indicator
    m_loadingLabel = new QLabel;
    m_loadingLabel->setVisible(false);
    m_loadingLabel->setStyleSheet("color: #ffaa00; font-weight: bold;");
    mainLayout->addWidget(m_loadingLabel);

    mainLayout->addStretch();
}

void GraphStatsPanel::onStatsChanged() {
    m_vertexCountLabel->setText(QString::number(m_manager->vertexCount()));
    m_edgeCountLabel->setText(QString::number(m_manager->edgeCount()));
    m_keyframeCountLabel->setText(QString::number(m_manager->keyframeCount()));
}

void GraphStatsPanel::onLoadingStateChanged() {
    bool loading = m_manager->isLoading();
    m_loadingLabel->setVisible(loading);
    if (loading) {
        m_loadingLabel->setText(tr("Loading..."));
    }
}
