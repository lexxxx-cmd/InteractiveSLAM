/**
 * @file AutoLoopClosureDialog.h
 * @brief 「高级设置 → 优化相关 → 自动回环检测」对话框
 *
 * 内嵌 AutoLoopClosurePanel（开始/停止 + 实时状态），下方为自动回环
 * 参数（搜索/配准/鲁棒核/评分/选项），全部集中于此对话框。
 * 参数读写内嵌面板（LoopClosureParams），控件变化即时生效。
 */
#pragma once

#include <QDialog>
#include <QVector>

class GraphManager;
class AutoLoopClosurePanel;
class QComboBox;
class QSpinBox;
class QDoubleSpinBox;
class QCheckBox;

class AutoLoopClosureDialog : public QDialog {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param manager 图数据管理器（自动回环数据层依赖）
     * @param parent 父组件
     */
    explicit AutoLoopClosureDialog(GraphManager* manager, QWidget* parent = nullptr);

    /** @brief 停止自动回环检测（MainWindow 关闭地图时调用） */
    void stopAutoLoop();

    /**
     * @brief 查询"插入回环边后是否执行优化"
     *
     * MainWindow 据此决定自动回环插边后是否需要重建点云：
     * 优化会改变位姿（点云世界坐标随之变化，需重建）；
     * 未优化时点云不变，仅刷新边线即可（避免无谓的全量重建卡顿）。
     */
    bool optimizeAfterInsert() const;

signals:
    // —— 转发自内嵌 AutoLoopClosurePanel ——
    /** @brief 自动回环插入新边（MainWindow 刷新视口） */
    void loopEdgeInserted();
    /** @brief 自动回环搜索状态（source=蓝, candidates=绿，MainWindow 高亮） */
    void loopDetectionStatus(long sourceId, QVector<long> candidateIds);

private:
    void setupUi();
    void loadParams();
    void applyParams();

    GraphManager* m_manager;
    AutoLoopClosurePanel* m_autoLoopPanel;  ///< 内嵌自动回环面板（开始/停止+状态）

    // 自动回环参数
    QComboBox*      m_searchMethodCombo = nullptr;
    QDoubleSpinBox* m_distanceThreshSpin = nullptr;
    QDoubleSpinBox* m_accumDistThreshSpin = nullptr;
    QComboBox*      m_methodCombo = nullptr;
    QSpinBox*       m_maxIterSpin = nullptr;
    QDoubleSpinBox* m_epsSpin = nullptr;
    QDoubleSpinBox* m_resolutionSpin = nullptr;
    QComboBox*      m_kernelCombo = nullptr;
    QDoubleSpinBox* m_kernelDeltaSpin = nullptr;
    QDoubleSpinBox* m_fitnessThreshSpin = nullptr;
    QDoubleSpinBox* m_fitnessMaxRangeSpin = nullptr;
    QCheckBox*      m_optimizeCb = nullptr;
};
