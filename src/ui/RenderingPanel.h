/**
 * @file RenderingPanel.h
 * @brief 渲染设置面板头文件
 *
 * RenderingPanel 是一个浮动面板，提供 3D 场景渲染的控制选项：
 * - 显示/隐藏顶点、边、关键帧点云、SE3 约束边
 * - 调节球体半径、边线宽度、点云大小和点云透明度
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
 * 包含四个复选框（顶点可见性、边可见性、点云可见性、SE3边可见性）
 * 和四个滑块（球体半径、边线宽度、点大小、点不透明度）。
 * 控件变化实时作用于 ViewportWidget 的渲染状态。
 */
class RenderingPanel : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param viewport 关联的 3D 视口部件
     * @param parent   父级 Qt 组件
     */
    explicit RenderingPanel(ViewportWidget* viewport, QWidget* parent = nullptr);

private:
    void setupUi();  ///< 初始化 UI 控件和布局

    ViewportWidget* m_viewport;  ///< 关联的 3D 视口（非拥有指针）

    QCheckBox* m_drawVerticesCb;    ///< "显示顶点"复选框
    QCheckBox* m_drawEdgesCb;       ///< "显示边"复选框
    QCheckBox* m_drawCloudsCb;      ///< "显示关键帧点云"复选框
    QCheckBox* m_drawSE3EdgesCb;    ///< "显示 SE3 约束边"复选框

    QSlider* m_sphereRadiusSlider;  ///< 球体半径滑块
    QLabel*  m_sphereRadiusLabel;   ///< 球体半径数值标签

    QSlider* m_edgeWidthSlider;     ///< 边线宽度滑块
    QLabel*  m_edgeWidthLabel;      ///< 边线宽度数值标签

    QSlider* m_pointSizeSlider;     ///< 点大小滑块
    QLabel*  m_pointSizeLabel;      ///< 点大小数值标签

    QSlider* m_pointOpacitySlider;  ///< 点不透明度滑块
    QLabel*  m_pointOpacityLabel;   ///< 点不透明度数值标签

    QSpinBox* m_sampleStrideSpin;   ///< 采样步长输入框
    QLabel*   m_sampleStrideLabel;  ///< 采样步长标签
};
