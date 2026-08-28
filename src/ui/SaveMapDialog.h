/**
 * @file SaveMapDialog.h
 * @brief "保存地图"配置对话框 —— 统一保存入口
 *
 * 将原「保存位姿图」与「保存地图」两个菜单合并为单个保存对话框，
 * 用户勾选要保存的内容并选择目标目录：
 *   - 保存位姿图（graph.g2o）
 *   - 保存每帧点云及 data 文件（标准地图目录：NNNNNN/{data, raw.pcd, cloud.pcd} + LVBA）
 *   - 保存全局点云地图（accumulated_cloud.pcd）
 */
#pragma once

#include <QDialog>

class QCheckBox;
class QLineEdit;

class SaveMapDialog : public QDialog {
    Q_OBJECT

public:
    explicit SaveMapDialog(QWidget* parent = nullptr);

    /** @brief 是否勾选"保存位姿图" */
    bool savePoseGraph() const;
    /** @brief 是否勾选"保存每帧点云及 data 文件" */
    bool saveKeyframes() const;
    /** @brief 是否勾选"保存全局点云地图" */
    bool saveGlobalCloud() const;

    /** @brief 用户选择的目标目录（空 = 未选择） */
    QString outputDirectory() const;

private:
    QCheckBox* m_poseGraphCb = nullptr;   ///< 保存位姿图
    QCheckBox* m_keyframesCb = nullptr;   ///< 保存每帧点云及 data 文件
    QCheckBox* m_globalCloudCb = nullptr; ///< 保存全局点云地图
    QLineEdit* m_dirEdit = nullptr;       ///< 输出目录
};
