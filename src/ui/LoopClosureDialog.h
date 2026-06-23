#pragma once

#include <QDialog>
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

/// @brief Modal dialog for manually closing a loop between two keyframes.
///
/// Flow:
///   1. User right-clicks vertex A → "Loop Begin" (MainWindow stores vertex ID)
///   2. User right-clicks vertex B → "Loop End" (MainWindow opens this dialog)
///   3. Dialog shows relative-pose preview, fitness score, adjustment controls
///   4. User can auto-align (FPFH), scan-match (ICP/GICP/NDT), or manually adjust
///   5. "Add Edge" commits the relative pose to the graph; "Cancel" discards
class LoopClosureDialog : public QDialog {
    Q_OBJECT
public:
    using PointT = pcl::PointXYZI;
    using CloudPtr = pcl::PointCloud<PointT>::ConstPtr;

    /// @param beginVertexId   ID of the "Loop Begin" vertex
    /// @param endVertexId     ID of the "Loop End" vertex
    /// @param manager         GraphManager for accessing the graph
    /// @param parent          Parent widget
    LoopClosureDialog(long beginVertexId, long endVertexId,
                      GraphManager* manager, QWidget* parent = nullptr);
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
