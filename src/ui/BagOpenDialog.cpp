/**
 * @file BagOpenDialog.cpp
 * @brief "打开 Bag 文件"导入配置对话框实现
 */
#include "ui/BagOpenDialog.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QScrollArea>
#include <QToolButton>
#include <QVBoxLayout>

#include <Eigen/Geometry>

BagOpenDialog::BagOpenDialog(const QStringList& topics,
                             const hdl_graph_slam::BagImportConfig& defaults,
                             QWidget* parent)
    : QDialog(parent), m_defaults(defaults) {
    setWindowTitle(tr("Import ROS Bag"));
    setupUi(topics, defaults);
}

QString BagOpenDialog::odomTopic() const {
    return m_odomCombo ? m_odomCombo->currentText() : QString();
}

QString BagOpenDialog::cloudTopic() const {
    return m_cloudCombo ? m_cloudCombo->currentText() : QString();
}

hdl_graph_slam::BagImportConfig BagOpenDialog::config() const {
    hdl_graph_slam::BagImportConfig cfg = m_defaults;
    if (m_odomCombo)  cfg.odomTopic  = odomTopic().toStdString();
    if (m_cloudCombo) cfg.cloudTopic = cloudTopic().toStdString();

    // 外参：行主序 3×3 旋转 + 3 平移
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            if (m_calibR[r * 3 + c])
                cfg.calibR(r, c) = m_calibR[r * 3 + c]->value();
        }
    }
    for (int i = 0; i < 3; ++i) {
        if (m_calibT[i]) cfg.calibT[i] = m_calibT[i]->value();
    }

    // 关键帧抽稀
    if (m_passthroughCb)  cfg.passthroughMode = m_passthroughCb->isChecked();
    if (m_meterGapSpin)   cfg.meterGap = m_meterGapSpin->value();
    if (m_degGapSpin)     cfg.degGap   = m_degGapSpin->value();

    // 输出
    if (m_outputDirEdit)  cfg.outputDir = m_outputDirEdit->text().trimmed().toStdString();
    if (m_saveFullCloudCb) cfg.saveFullCloud = m_saveFullCloudCb->isChecked();

    return cfg;
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
                            const hdl_graph_slam::BagImportConfig& defaults) {
    auto* outer = new QVBoxLayout(this);

    // 内容放入滚动区域，避免小屏幕上被裁切
    auto* scroll = new QScrollArea;
    scroll->setWidgetResizable(true);
    scroll->setFrameShape(QFrame::NoFrame);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);

    auto* content = new QWidget;
    auto* layout = new QVBoxLayout(content);
    layout->setContentsMargins(8, 8, 8, 8);

    // ============ Topics ============
    auto* topicGroup = new QGroupBox(tr("Topics"));
    auto* topicForm = new QFormLayout(topicGroup);

    m_odomCombo = new QComboBox;
    m_odomCombo->addItems(topics);
    selectDefault(m_odomCombo, topics,
                  QString::fromStdString(defaults.odomTopic),
                  {"odom", "pose", "mapped", "gt", "odometry"});
    topicForm->addRow(tr("Pose topic:"), m_odomCombo);

    m_cloudCombo = new QComboBox;
    m_cloudCombo->addItems(topics);
    selectDefault(m_cloudCombo, topics,
                  QString::fromStdString(defaults.cloudTopic),
                  {"cloud", "lidar", "velodyne", "points"});
    topicForm->addRow(tr("Point cloud topic:"), m_cloudCombo);

    layout->addWidget(topicGroup);

    // ============ Lidar↔IMU 外参 ============
    auto* calibGroup = new QGroupBox(tr("Lidar↔IMU Extrinsic (calib T_IL)"));
    auto* calibLayout = new QVBoxLayout(calibGroup);

    auto* calibForm = new QFormLayout;
    // 3×3 旋转矩阵（行主序），3 行
    for (int r = 0; r < 3; ++r) {
        auto* row = new QHBoxLayout;
        for (int c = 0; c < 3; ++c) {
            auto* spin = new QDoubleSpinBox;
            spin->setRange(-100.0, 100.0);
            spin->setDecimals(6);
            spin->setSingleStep(0.01);
            spin->setValue(defaults.calibR(r, c));
            m_calibR[r * 3 + c] = spin;
            row->addWidget(spin);
        }
        calibForm->addRow(tr("R row %1:").arg(r + 1), row);
    }
    // 平移 3 个
    auto* tRow = new QHBoxLayout;
    for (int i = 0; i < 3; ++i) {
        auto* spin = new QDoubleSpinBox;
        spin->setRange(-100.0, 100.0);
        spin->setDecimals(6);
        spin->setSingleStep(0.01);
        spin->setValue(defaults.calibT[i]);
        m_calibT[i] = spin;
        tRow->addWidget(spin);
    }
    calibForm->addRow(tr("Translation t:"), tRow);

    calibLayout->addLayout(calibForm);

    // 单位阵快捷按钮
    auto* identityBtn = new QToolButton;
    identityBtn->setText(tr("Reset to Identity"));
    connect(identityBtn, &QToolButton::clicked, this, [this]() {
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                m_calibR[r * 3 + c]->setValue(r == c ? 1.0 : 0.0);
        for (int i = 0; i < 3; ++i) m_calibT[i]->setValue(0.0);
    });
    calibLayout->addWidget(identityBtn, 0, Qt::AlignLeft);

    layout->addWidget(calibGroup);

    // ============ 关键帧抽稀 ============
    auto* keyGroup = new QGroupBox(tr("Keyframe Thinning"));
    auto* keyForm = new QFormLayout(keyGroup);

    m_passthroughCb = new QCheckBox(tr("Passthrough (all frames as keyframes)"));
    m_passthroughCb->setChecked(defaults.passthroughMode);
    keyForm->addRow(m_passthroughCb);

    m_meterGapSpin = new QDoubleSpinBox;
    m_meterGapSpin->setRange(0.0, 1000.0);
    m_meterGapSpin->setDecimals(2);
    m_meterGapSpin->setSingleStep(0.5);
    m_meterGapSpin->setValue(defaults.meterGap);
    keyForm->addRow(tr("Meter gap (m):"), m_meterGapSpin);

    m_degGapSpin = new QDoubleSpinBox;
    m_degGapSpin->setRange(0.0, 360.0);
    m_degGapSpin->setDecimals(1);
    m_degGapSpin->setSingleStep(1.0);
    m_degGapSpin->setValue(defaults.degGap);
    keyForm->addRow(tr("Degree gap (deg):"), m_degGapSpin);

    // passthrough 开启时禁用抽稀阈值
    auto onPassthrough = [this](bool checked) {
        m_meterGapSpin->setEnabled(!checked);
        m_degGapSpin->setEnabled(!checked);
    };
    connect(m_passthroughCb, &QCheckBox::toggled, this, onPassthrough);
    onPassthrough(m_passthroughCb->isChecked());

    layout->addWidget(keyGroup);

    // ============ 输出 ============
    auto* outGroup = new QGroupBox(tr("Output"));
    auto* outForm = new QFormLayout(outGroup);

    auto* dirRow = new QHBoxLayout;
    m_outputDirEdit = new QLineEdit;
    m_outputDirEdit->setText(QString::fromStdString(defaults.outputDir));
    m_outputDirEdit->setPlaceholderText(tr("(empty = temporary directory)"));
    dirRow->addWidget(m_outputDirEdit, 1);
    auto* browseBtn = new QPushButton(tr("Browse..."));
    connect(browseBtn, &QPushButton::clicked, this, [this]() {
        QString dir = QFileDialog::getExistingDirectory(
            this, tr("Choose Output Directory"), m_outputDirEdit->text());
        if (!dir.isEmpty()) m_outputDirEdit->setText(dir);
    });
    dirRow->addWidget(browseBtn);
    outForm->addRow(tr("Output directory:"), dirRow);

    m_saveFullCloudCb = new QCheckBox(tr("Save full-resolution raw.pcd"));
    m_saveFullCloudCb->setChecked(defaults.saveFullCloud);
    outForm->addRow(m_saveFullCloudCb);

    layout->addWidget(outGroup);

    scroll->setWidget(content);
    outer->addWidget(scroll, 1);

    // ============ 按钮 ============
    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    connect(buttons, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
    outer->addWidget(buttons);

    resize(520, 560);
}
