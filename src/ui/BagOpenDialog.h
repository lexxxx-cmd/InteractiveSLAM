/**
 * @file bag_open_dialog.h
 * @brief "打开 Bag 文件"对话框 —— 选择位姿 topic 与点云 topic
 *
 * 读取 bag 索引后列出全部 topic，供用户指定：
 *   - 位姿 topic（nav_msgs/Odometry 兼容，如 /odom、/odometry）
 *   - 点云 topic（sensor_msgs/PointCloud2，如 /velodyne_points、/cloud_registered）
 * 下拉框以 YAML 配置（config/bag_import.yaml）中的 topic 为默认值预填；
 * 无默认值或默认值不在 bag 中时按关键字智能匹配（位姿：odom/pose/mapped/gt；
 * 点云：cloud/lidar/velodyne/points）。
 */
#pragma once

#include <QDialog>
#include <QStringList>

class QComboBox;

class BagOpenDialog : public QDialog {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param topics            bag 内可用 topic 列表
     * @param defaultOdomTopic  yaml 配置的位姿 topic（可能不在 bag 中）
     * @param defaultCloudTopic yaml 配置的点云 topic（可能不在 bag 中）
     * @param parent            父组件
     */
    explicit BagOpenDialog(const QStringList& topics,
                           const QString& defaultOdomTopic = QString(),
                           const QString& defaultCloudTopic = QString(),
                           QWidget* parent = nullptr);

    /** @brief 用户选择的位姿 topic */
    QString odomTopic() const;
    /** @brief 用户选择的点云 topic */
    QString cloudTopic() const;

private:
    void setupUi(const QStringList& topics,
                 const QString& defaultOdomTopic,
                 const QString& defaultCloudTopic);
    static void selectDefault(QComboBox* combo, const QStringList& topics,
                              const QString& preferred, const QStringList& keywords);

    QComboBox* m_odomCombo = nullptr;   ///< 位姿 topic 下拉
    QComboBox* m_cloudCombo = nullptr;  ///< 点云 topic 下拉
};
