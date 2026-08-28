/**
 * @file SaveMapDialog.cpp
 * @brief "保存地图"配置对话框实现
 */
#include "ui/SaveMapDialog.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSettings>
#include <QVBoxLayout>

SaveMapDialog::SaveMapDialog(QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(tr("Save Map"));
    resize(440, 260);

    auto* layout = new QVBoxLayout(this);

    auto* group = new QGroupBox(tr("Select what to save"));
    auto* form = new QFormLayout(group);

    // 保存内容复选框（默认全选；如上次保存过则恢复上次勾选）
    m_poseGraphCb = new QCheckBox(tr("Pose graph (graph.g2o)"));
    m_poseGraphCb->setChecked(true);
    form->addRow(m_poseGraphCb);

    m_keyframesCb = new QCheckBox(tr("Keyframes (per-frame clouds & data files)"));
    m_keyframesCb->setChecked(true);
    form->addRow(m_keyframesCb);

    m_globalCloudCb = new QCheckBox(tr("Global point cloud map (accumulated_cloud.pcd)"));
    m_globalCloudCb->setChecked(true);
    form->addRow(m_globalCloudCb);

    // 输出目录
    auto* dirRow = new QHBoxLayout;
    m_dirEdit = new QLineEdit;
    m_dirEdit->setPlaceholderText(tr("Choose output directory..."));
    dirRow->addWidget(m_dirEdit, 1);
    auto* browseBtn = new QPushButton(tr("Browse..."));
    connect(browseBtn, &QPushButton::clicked, this, [this]() {
        QString dir = QFileDialog::getExistingDirectory(
            this, tr("Save Map Directory"), m_dirEdit->text(),
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
        if (!dir.isEmpty()) m_dirEdit->setText(dir);
    });
    dirRow->addWidget(browseBtn);
    form->addRow(tr("Output directory:"), dirRow);

    // 地图目录布局说明：graph.g2o 与单帧数据固定保存在同一目录
    auto* hint = new QLabel(
        tr("All contents are saved into the selected directory: graph.g2o "
           "at the root, per-frame data in NNNNNN/ subdirectories."));
    hint->setWordWrap(true);
    hint->setStyleSheet("color: #888888; font-size: 11px;");
    layout->addWidget(group);
    layout->addWidget(hint);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Save | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    layout->addWidget(buttons);
}

void SaveMapDialog::setOutputDirectory(const QString& dir) {
    m_dirEdit->setText(dir);
    // 恢复上次保存的内容勾选（快速保存共用同一份配置）
    QSettings settings("DAFTECH", "InteractiveSLAM");
    m_poseGraphCb->setChecked(settings.value("save_map/pose_graph", true).toBool());
    m_keyframesCb->setChecked(settings.value("save_map/keyframes", true).toBool());
    m_globalCloudCb->setChecked(settings.value("save_map/global_cloud", true).toBool());
}

bool SaveMapDialog::savePoseGraph() const {
    return m_poseGraphCb ? m_poseGraphCb->isChecked() : false;
}

bool SaveMapDialog::saveKeyframes() const {
    return m_keyframesCb ? m_keyframesCb->isChecked() : false;
}

bool SaveMapDialog::saveGlobalCloud() const {
    return m_globalCloudCb ? m_globalCloudCb->isChecked() : false;
}

QString SaveMapDialog::outputDirectory() const {
    return m_dirEdit ? m_dirEdit->text().trimmed() : QString();
}
