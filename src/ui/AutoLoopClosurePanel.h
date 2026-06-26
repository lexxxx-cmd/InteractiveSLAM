#pragma once

#include <QWidget>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QCheckBox>
#include <QPushButton>
#include <QLabel>
#include <QTimer>
#include <memory>

class GraphManager;

namespace hdl_graph_slam {
class AutomaticLoopClosure;
}  // namespace hdl_graph_slam

/**
 * @brief Right-side dock panel for automatic loop closure detection.
 *
 * Owns an AutomaticLoopClosure instance (data layer) and provides Qt
 * controls for configuration and status monitoring.  Uses a 100 ms
 * QTimer to poll the thread-safe Status snapshot from the data layer.
 */
class AutoLoopClosurePanel : public QWidget {
    Q_OBJECT

public:
    explicit AutoLoopClosurePanel(GraphManager* manager, QWidget* parent = nullptr);
    ~AutoLoopClosurePanel() override;

    /// Stop the detection thread (called by MainWindow on map close).
    void stopDetection();

signals:
    /// Emitted when a new loop edge is inserted — MainWindow refreshes the viewport.
    void loopEdgeInserted();
    /// Emitted on every poll tick (~10 Hz) — source=blue, candidates=green in viewport.
    void loopDetectionStatus(long sourceId, QVector<long> candidateIds);

private slots:
    void onStartStop();
    void onPollStatus();

private:
    void setupUi();
    void syncParamsToLoop();   // push UI widget values → data layer
    void syncParamsFromLoop(); // pull data layer defaults → UI widgets (on create)

    GraphManager* m_manager;

    // ── Data layer (lazy-created on first Start) ─────────────────
    std::unique_ptr<hdl_graph_slam::AutomaticLoopClosure> m_autoLoop;

    // ── Search parameters ────────────────────────────────────────
    QComboBox*      m_searchMethodCombo;
    QDoubleSpinBox* m_distanceThreshSpin;
    QDoubleSpinBox* m_accumDistThreshSpin;

    // ── Registration parameters ──────────────────────────────────
    QComboBox*      m_methodCombo;
    QSpinBox*       m_maxIterSpin;
    QDoubleSpinBox* m_epsSpin;
    QDoubleSpinBox* m_resolutionSpin;

    // ── Robust kernel ────────────────────────────────────────────
    QComboBox*      m_kernelCombo;
    QDoubleSpinBox* m_kernelDeltaSpin;

    // ── Fitness thresholds ───────────────────────────────────────
    QDoubleSpinBox* m_fitnessThreshSpin;
    QDoubleSpinBox* m_fitnessMaxRangeSpin;

    // ── Options ──────────────────────────────────────────────────
    QCheckBox* m_optimizeCb;

    // ── Action ───────────────────────────────────────────────────
    QPushButton* m_startStopBtn;

    // ── Status display ───────────────────────────────────────────
    QLabel* m_statusLabel;
    QLabel* m_sourceLabel;
    QLabel* m_candidatesLabel;
    QLabel* m_edgesInsertedLabel;
    QLabel* m_lastMatchLabel;

    // ── Polling ──────────────────────────────────────────────────
    QTimer* m_pollTimer;
    int m_lastKnownEdgesInserted = 0;
};
