/**
 * @file BagOpenDialog.h
 * @brief "打开 Bag 文件"导入配置对话框
 *
 * 读取 bag 索引后列出全部 topic，供用户指定：
 *   - 位姿 topic（nav_msgs/Odometry 兼容，如 /odom、/odometry）
 *   - 点云 topic（sensor_msgs/PointCloud2，如 /velodyne_points、/cloud_registered）
 * 下拉框以 YAML 配置（config/bag_import.yaml）中的 topic 为默认值预填；
 * 无默认值或默认值不在 bag 中时按关键字智能匹配（位姿：odom/pose/mapped/gt；
 * 点云：cloud/lidar/velodyne/points）。
 *
 * 除 topic 外，弹窗还提供 yaml 中其余导入参数的 UI 配置（预填 yaml 值）：
 *   - Lidar↔IMU 外参（calib_r 3×3 行主序 + calib_t 3 个）
 *   - 关键帧抽稀（passthrough_mode / meter_gap / deg_gap）
 *   - 输出（save_full_cloud；输出目录由程序内部决定，不再提供设定）
 * 用户确认后经 config() 返回完整 BagImportConfig，供后台导入使用。
 */
#pragma once

#include <QDialog>
#include <QStringList>

#include "data/hdl_graph_slam/bag_importer.hpp"

class QComboBox;
class QDoubleSpinBox;
class QCheckBox;

class BagOpenDialog : public QDialog {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param topics    bag 内可用 topic 列表
     * @param defaults  yaml 配置的默认导入参数（topic/外参/抽稀/保存选项）
     * @param parent    父组件
     */
    explicit BagOpenDialog(const QStringList& topics,
                           const hdl_graph_slam::BagImportConfig& defaults,
                           QWidget* parent = nullptr);

    /** @brief 用户选择的位姿 topic */
    QString odomTopic() const;
    /** @brief 用户选择的点云 topic */
    QString cloudTopic() const;

    /**
     * @brief 返回用户编辑后的完整导入配置
     *
     * topic 取下拉框当前值，外参/抽稀/保存选项取各控件当前值
     * （以构造时传入的 yaml 默认值初始化，未改动部分保持一致）。
     */
    hdl_graph_slam::BagImportConfig config() const;

private:
    void setupUi(const QStringList& topics,
                 const hdl_graph_slam::BagImportConfig& defaults);
    static void selectDefault(QComboBox* combo, const QStringList& topics,
                              const QString& preferred, const QStringList& keywords);

    // Topics
    QComboBox* m_odomCombo = nullptr;   ///< 位姿 topic 下拉
    QComboBox* m_cloudCombo = nullptr;  ///< 点云 topic 下拉

    // Lidar↔IMU 外参（行主序 3×3 + 平移 3）
    QDoubleSpinBox* m_calibR[9]  = {nullptr};
    QDoubleSpinBox* m_calibT[3]  = {nullptr};

    // 关键帧抽稀
    QCheckBox*      m_passthroughCb = nullptr;
    QDoubleSpinBox* m_meterGapSpin = nullptr;
    QDoubleSpinBox* m_degGapSpin   = nullptr;

    // 输出（输出目录由程序内部决定，不再提供设定）
    QCheckBox*      m_saveFullCloudCb = nullptr;

    hdl_graph_slam::BagImportConfig m_defaults;  ///< 构造时的 yaml 默认值
};
