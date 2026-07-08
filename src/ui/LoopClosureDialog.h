#pragma once

#include <QDialog>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QLabel>
#include <QPushButton>
#include <QProgressBar>
#include <QFutureWatcher>
#include <atomic>
#include <memory>
#include <Eigen/Geometry>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

class MiniViewportWidget;
class GraphManager;

namespace hdl_graph_slam {
class RegistrationMethods;
class InteractiveGraph;
}  // namespace hdl_graph_slam

/// Merge point clouds of keyframes adjacent to @p centerId into
/// @p centerId's local coordinate frame.
/// @param windowHalfSize  Number of keyframes to merge on each side of centerId.
///                        N merges centerId-N through centerId+N (up to 2N+1 frames).
///                        Default 1 (backward compatible: up to 3 keyframes).
/// Clouds from keyframes that do not exist (or have empty clouds) are
/// silently skipped.
pcl::PointCloud<pcl::PointXYZI>::Ptr mergeAdjacentClouds(
    const hdl_graph_slam::InteractiveGraph* graph, long centerId,
    int windowHalfSize = 1);

/// @brief Modal dialog for manually closing a loop between two keyframes.
///
/// Flow:
///   1. User right-clicks vertex A → "Loop Begin" (MainWindow stores vertex ID)
///   2. MainWindow merges adjacent-clouds for both A and B
///   3. User right-clicks vertex B → "Loop End" (MainWindow opens this dialog)
///   4. Dialog shows relative-pose preview, fitness score, adjustment controls
///   5. User can auto-align (FPFH), scan-match (ICP/GICP/NDT), or manually adjust
///   6. "Add Edge" commits the relative pose to the graph; "Cancel" discards
///
/// @param beginCloud  Pre-merged cloud for begin vertex (center-frame local)
/// @param endCloud    Pre-merged cloud for end vertex (center-frame local)
class LoopClosureDialog : public QDialog {
    Q_OBJECT
public:
    using PointT = pcl::PointXYZI;
    using CloudPtr = pcl::PointCloud<PointT>::ConstPtr;

    /// @param beginVertexId   ID of the "Loop Begin" vertex
    /// @param endVertexId     ID of the "Loop End" vertex
    /// @param manager         GraphManager for accessing the graph
    /// @param beginCloud      Pre-merged cloud for begin vertex (center-frame local coords)
    /// @param endCloud        Pre-merged cloud for end vertex (center-frame local coords)
    /// @param parent          Parent widget
    LoopClosureDialog(long beginVertexId, long endVertexId,
                      GraphManager* manager,
                      CloudPtr beginCloud, CloudPtr endCloud,
                      QWidget* parent = nullptr);
    ~LoopClosureDialog() override;

private slots:
    // --- Slider delta application ---
    void onSliderPXChanged(double value);
    void onSliderPYChanged(double value);
    void onSliderPZChanged(double value);
    void onSliderRXChanged(double value);
    void onSliderRYChanged(double value);
    void onSliderRZChanged(double value);

    // --- Buttons ---
    void onAutoAlign();
    void onScanMatching();
    void onReset();
    void onAddEdge();

    // --- Registration thread completion ---
    void onFpfhAlignFinished();
    void onScanMatchFinished();

private:
    void updateFitnessScore();
    void updatePreview();
    void setupUi();

    // Slider helper: apply delta and reset spinbox to 0
    void applySliderDelta(int axis, double delta, bool isRotation);

    // FPFH auto-align (runs on background thread)
    void runFpfhAlign(double normalRadius, double searchRadius,
                      int maxIter, int numSamples, int corrRandomness,
                      double similarityThresh, double maxCorrDist, double inlierFrac);

    // Scan matching (runs on background thread)
    void runScanMatching(int methodIndex, int maxIterations,
                         float transEpsilon, float resolution);

    // === Data ===
    GraphManager* m_manager;
    hdl_graph_slam::InteractiveGraph* m_graph;

    long m_beginVertexId;
    long m_endVertexId;

    CloudPtr m_beginCloud;
    CloudPtr m_endCloud;

    Eigen::Isometry3d m_beginPose;      // frozen at dialog open
    Eigen::Isometry3d m_endPoseInit;    // frozen at dialog open
    Eigen::Isometry3d m_endPose;        // mutable, adjusted by sliders/registration

    std::unique_ptr<hdl_graph_slam::RegistrationMethods> m_regMethods;

    // === UI Widgets ===
    MiniViewportWidget* m_miniViewport;
    QLabel* m_fitnessLabel;
    QComboBox* m_stepCombo;             // step-size gear selector
    QDoubleSpinBox* m_sliders[6];       // PX, PY, PZ, RX, RY, RZ
    double m_sliderPrevValues[6];        // track previous values for delta computation
    QPushButton* m_autoAlignBtn;
    QPushButton* m_scanMatchBtn;
    QPushButton* m_resetBtn;
    QPushButton* m_addEdgeBtn;
    QPushButton* m_cancelBtn;
    QProgressBar* m_progressBar;
    QLabel* m_statusLabel;

    // === Threading ===
    QFutureWatcher<Eigen::Isometry3d>* m_fpfhWatcher = nullptr;
    std::atomic_int m_fpfhProgress{0};  // 0-5 for progress bar stages
    bool m_fpfhRunning = false;

    QFutureWatcher<Eigen::Isometry3d>* m_scanMatchWatcher = nullptr;
    bool m_scanMatchRunning = false;
};
