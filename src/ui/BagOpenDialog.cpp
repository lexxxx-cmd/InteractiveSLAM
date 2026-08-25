/**
 * @file bag_open_dialog.cpp
 * @brief "打开 Bag 文件"对话框实现
 */
#include "ui/BagOpenDialog.h"

#include <QComboBox>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QLabel>
#include <QVBoxLayout>

BagOpenDialog::BagOpenDialog(const QStringList& topics,
                             const QString& defaultOdomTopic,
                             const QString& defaultCloudTopic,
                             QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(tr("Import ROS Bag"));
    setupUi(topics, defaultOdomTopic, defaultCloudTopic);
}

QString BagOpenDialog::odomTopic() const {
    return m_odomCombo ? m_odomCombo->currentText() : QString();
}

QString BagOpenDialog::cloudTopic() const {
    return m_cloudCombo ? m_cloudCombo->currentText() : QString();
}

/**
 * @brief 选中下拉项：优先 yaml 默认值（若在列表中），否则按关键字匹配
 */
void BagOpenDialog::selectDefault(QComboBox* combo, const QStringList& topics,
                                  const QString& preferred,
                                  const QStringList& keywords) {
    if (!combo) return;

    // 1) yaml 配置的默认值（若存在于 bag topic 中）
    if (!preferred.isEmpty()) {
        int idx = topics.indexOf(preferred);
        if (idx >= 0) {
            combo->setCurrentIndex(idx);
            return;
        }
    }
    // 2) 关键字智能匹配
    for (int i = 0; i < combo->count(); ++i) {
        const QString text = combo->itemText(i).toLower();
        for (const auto& kw : keywords) {
            if (text.contains(kw)) {
                combo->setCurrentIndex(i);
                return;
            }
        }
    }
}

void BagOpenDialog::setupUi(const QStringList& topics,
                            const QString& defaultOdomTopic,
                            const QString& defaultCloudTopic) {
    auto* layout = new QVBoxLayout(this);

    layout->addWidget(new QLabel(tr("Select topics from the bag:")));

    auto* form = new QFormLayout;

    m_odomCombo = new QComboBox;
    m_odomCombo->addItems(topics);
    selectDefault(m_odomCombo, topics, defaultOdomTopic,
                  {"odom", "pose", "mapped", "gt", "odometry"});
    form->addRow(tr("Pose topic:"), m_odomCombo);

    m_cloudCombo = new QComboBox;
    m_cloudCombo->addItems(topics);
    selectDefault(m_cloudCombo, topics, defaultCloudTopic,
                  {"cloud", "lidar", "velodyne", "points"});
    form->addRow(tr("Point cloud topic:"), m_cloudCombo);

    layout->addLayout(form);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}
