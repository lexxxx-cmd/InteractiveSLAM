/**
 * @file RenderingPanel.h
 * @brief 渲染设置面板头文件
 *
 * RenderingPanel 是一个浮动面板，提供 3D 场景渲染的控制选项：
 * - 显示/隐藏位姿视锥体、边、点云
 * - 调节视锥体大小、边线宽度、点云大小/透明度和视锥体透明度
 *
 * 所有控件通过信号槽直接连接到 ViewportWidget 的对应 setter 方法。
 */

#pragma once

#include <QWidget>
#include <QLabel>
#include <QCheckBox>
#include <QSlider>
#include <QSpinBox>
#include <QComboBox>

class ViewportWidget;

/**
 * @brief 渲染控制浮动面板
 *
 * 包含三个复选框（位姿视锥体/边/点云可见性）
 * 和五个滑块（视锥体大小、边线宽度、点大小、点透明度、视锥体透明度）。
 * 控件变化实时作用于 ViewportWidget 的渲染状态。
 */
class RenderingPanel : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param viewport 关联的 3D 视口部件
     * @param parent   父级部件
     */
    explicit RenderingPanel(ViewportWidget* viewport, QWidget* parent = nullptr);

    /**
     * @brief 重置采样步长显示为 1（图谱关闭后调用）
     *
     * 场景侧在关图时已把采样步长重置回全量渲染（新图边 ID 从 0
     * 重编号，旧的采样隐藏快照会误隐藏新加回环边），本方法把
     * SpinBox 显示同步回 1。用 blockSignals 避免触发 valueChanged
     * 反向再设一次场景（无害但多余）。
     */
    void resetSampleStride();

private:
    void setupUi();  ///< 初始化 UI 控件和布局

    ViewportWidget* m_viewport;  ///< 关联的 3D 视口（非拥有指针）

    QCheckBox* m_drawVerticesCb;    ///< "位姿视锥体"复选框
    QCheckBox* m_drawEdgesCb;       ///< "显示边"复选框
    QCheckBox* m_drawCloudsCb;      ///< "点云"复选框
    QCheckBox* m_odomLayerCb;       ///< "原始层"复选框（里程计位姿参照底图）

    QSlider* m_sphereRadiusSlider;  ///< 视锥体大小滑块
    QLabel*  m_sphereRadiusLabel;   ///< 视锥体大小数值标签

    QSlider* m_edgeWidthSlider;     ///< 边线宽度滑块
    QLabel*  m_edgeWidthLabel;      ///< 边线宽度数值标签

    QSlider* m_pointSizeSlider;     ///< 点大小滑块
    QLabel*  m_pointSizeLabel;      ///< 点大小数值标签

    QSlider* m_pointOpacitySlider;  ///< 点不透明度滑块
    QLabel*  m_pointOpacityLabel;   ///< 点不透明度数值标签

    QSlider* m_vertexOpacitySlider; ///< 视锥体不透明度滑块
    QLabel*  m_vertexOpacityLabel;  ///< 视锥体不透明度数值标签

    QSpinBox* m_sampleStrideSpin;   ///< 采样步长输入框（名称待定）
    QLabel*   m_sampleStrideLabel;  ///< 采样步长标签
};
