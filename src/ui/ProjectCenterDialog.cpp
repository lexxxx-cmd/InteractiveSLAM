// ============================================================================
// ProjectCenterDialog.cpp
// 项目中心对话框实现
// ============================================================================

#include "ui/ProjectCenterDialog.h"

#include <QDateTime>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFormLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QSettings>
#include <QPushButton>
#include <QVBoxLayout>

#include "data/hdl_graph_slam/bag_importer.hpp"
#include "ui/BagOpenDialog.h"

namespace {

constexpr char kSettingsOrg[] = "DAFTECH";
constexpr char kSettingsApp[] = "InteractiveSLAM";
constexpr char kRecentKey[]   = "recent_projects";
constexpr int  kMaxRecent     = 10;

QStringList recentDirs() {
    QSettings settings(kSettingsOrg, kSettingsApp);
    return settings.value(kRecentKey).toStringList();
}

void pushRecentDir(const QString& dir) {
    QSettings settings(kSettingsOrg, kSettingsApp);
    QStringList list = settings.value(kRecentKey).toStringList();
    list.removeAll(dir);
    list.prepend(dir);
    while (list.size() > kMaxRecent) list.removeLast();
    settings.setValue(kRecentKey, list);
}

}  // namespace

ProjectCenterDialog::ProjectCenterDialog(QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(tr("项目中心"));
    setModal(true);
    resize(760, 480);

    auto* rootLayout = new QHBoxLayout(this);

    // ---- 左侧：最近项目 ----
    auto* leftLayout = new QVBoxLayout;
    auto* recentTitle = new QLabel(tr("最近项目"));
    recentTitle->setStyleSheet("font-weight: bold; font-size: 14px;");
    leftLayout->addWidget(recentTitle);

    m_recentList = new QListWidget;
    m_recentList->setSelectionMode(QAbstractItemView::SingleSelection);
    connect(m_recentList, &QListWidget::itemDoubleClicked,
            this, &ProjectCenterDialog::onOpenRecentItem);
    leftLayout->addWidget(m_recentList, 1);
    rootLayout->addLayout(leftLayout, 3);

    // ---- 右侧：开始使用 ----
    auto* rightLayout = new QVBoxLayout;
    auto* startTitle = new QLabel(tr("开始使用"));
    startTitle->setStyleSheet("font-weight: bold; font-size: 18px;");
    rightLayout->addWidget(startTitle);

    auto* newBtn = new QPushButton(tr("新建项目"));
    newBtn->setMinimumHeight(56);
    newBtn->setObjectName("primaryButton");
    newBtn->setToolTip(tr("设置项目名称和保存位置，可关联 Bag 原始数据"));
    auto* openBtn = new QPushButton(tr("打开项目"));
    openBtn->setMinimumHeight(56);
    openBtn->setToolTip(tr("从最近列表或本地目录打开已有项目"));
    rightLayout->addWidget(newBtn);
    rightLayout->addWidget(openBtn);
    rightLayout->addStretch();
    rootLayout->addLayout(rightLayout, 2);

    connect(newBtn, &QPushButton::clicked, this, &ProjectCenterDialog::onNewProject);
    connect(openBtn, &QPushButton::clicked, this, &ProjectCenterDialog::onOpenProject);

    refreshRecentList();
}

// ---------------------------------------------------------------------------
// 最近项目列表
// ---------------------------------------------------------------------------

void ProjectCenterDialog::refreshRecentList() {
    m_recentList->clear();
    for (const auto& info : ProjectManager::loadRecent(recentDirs())) {
        auto* item = new QListWidgetItem(m_recentList);

        // 列表项：占位缩略图 + 名称/时间/路径
        auto* row = new QWidget;
        auto* layout = new QHBoxLayout(row);
        layout->setContentsMargins(4, 4, 4, 4);

        auto* thumb = new QLabel;
        thumb->setFixedSize(56, 56);
        thumb->setAlignment(Qt::AlignCenter);
        // 占位缩略图：深色底 + 项目名首字符（后续版本替换为主视口截图）
        thumb->setStyleSheet(
            "background-color: #1e2430; color: #8899aa; font-size: 22px;"
            "border: 1px solid #33405a; border-radius: 4px;");
        thumb->setText(info.name.isEmpty() ? "?" : info.name.left(1).toUpper());
        layout->addWidget(thumb);

        auto* textLayout = new QVBoxLayout;
        auto* name = new QLabel(info.name.isEmpty()
                                    ? QDir(info.projectDir).dirName() : info.name);
        name->setStyleSheet("font-weight: bold;");
        auto statusText = info.valid ? info.status : QString("invalid");
        auto* meta = new QLabel(tr("最近打开: %1    状态: %2")
                                    .arg(info.lastOpenedTime.isValid()
                                             ? info.lastOpenedTime.toString("yyyy/MM/dd HH:mm")
                                             : tr("未知"),
                                         statusText));
        auto* path = new QLabel(info.projectDir);
        path->setStyleSheet("color: #8899aa;");
        textLayout->addWidget(name);
        textLayout->addWidget(meta);
        textLayout->addWidget(path);
        layout->addLayout(textLayout, 1);

        item->setData(Qt::UserRole, info.projectDir);
        item->setSizeHint(row->sizeHint());
        m_recentList->setItemWidget(item, row);
    }
}

void ProjectCenterDialog::accept() {
    if (!m_task.projectDir.isEmpty()) {
        pushRecentDir(m_task.projectDir);
    }
    QDialog::accept();
}

// ---------------------------------------------------------------------------
// 打开项目 —— 判定链
// ---------------------------------------------------------------------------

void ProjectCenterDialog::onOpenProject() {
    // 选中最近列表项则直接打开；否则浏览目录
    auto* sel = m_recentList->currentItem();
    if (sel) {
        openProjectDir(sel->data(Qt::UserRole).toString());
        return;
    }
    QString dir = QFileDialog::getExistingDirectory(
        this, tr("选择项目目录"), QString(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (!dir.isEmpty()) openProjectDir(dir);
}

void ProjectCenterDialog::onOpenRecentItem(int) {
    onOpenProject();
}

void ProjectCenterDialog::openProjectDir(const QString& dir) {
    if (!ProjectManager::isProjectDir(dir)) {
        // 不是项目目录：兼容旧行为——当作纯地图目录直接加载（不回写项目状态）
        QMessageBox::StandardButton btn = QMessageBox::question(
            this, tr("不是项目目录"),
            tr("所选目录不是项目（缺少 project.json）。\n\n"
               "是否直接将其作为地图目录打开？"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
        if (btn != QMessageBox::Yes) return;
        m_task.action = ProjectTask::Action::LoadDirectory;
        m_task.dataDir = QDir::cleanPath(dir);
        accept();
        return;
    }

    auto info = ProjectManager::read(dir);
    if (!info.valid) {
        QMessageBox::warning(this, tr("打开项目失败"), info.errorString);
        return;
    }
    handleOpenInfo(info);
}

void ProjectCenterDialog::handleOpenInfo(const ProjectInfo& info) {
    // ---- 异常状态（processing / failed）：上次可能异常退出 ----
    if (info.status == QStringLiteral("processing") ||
        info.status == QStringLiteral("failed")) {
        QMessageBox box(this);
        box.setIcon(QMessageBox::Warning);
        box.setWindowTitle(tr("项目未正常完成导入"));
        box.setText(tr("项目「%1」上次导入未完成（状态: %2）。\n"
                       "数据目录可能不完整，如何处理？")
                        .arg(info.name, info.status));
        QPushButton* reimportBtn =
            box.addButton(tr("重新导入"), QMessageBox::DestructiveRole);
        QPushButton* loadBtn =
            box.addButton(tr("直接加载已有文件"), QMessageBox::AcceptRole);
        box.addButton(QMessageBox::Cancel);
        box.exec();
        auto clicked = box.clickedButton();

        if (clicked == reimportBtn) {
            if (info.bagPath.isEmpty() || !QFile::exists(info.bagPath)) {
                QMessageBox::warning(this, tr("无法重新导入"),
                                     tr("未找到关联的 Bag 文件：%1").arg(info.bagPath));
                return;
            }
            startImport(info);
        } else if (clicked == loadBtn) {
            // 等价于旧的"打开地图"：不管状态直接加载已有文件
            if (!finalizeLoad(info)) return;
        }
        return;  // 取消 → 留在项目中心
    }

    // ---- completed / pending：数据目录就绪则直接加载 ----
    if (info.dataDirExists()) {
        finalizeLoad(info);
        return;
    }

    // ---- 数据目录丢失 ----
    if (!info.bagPath.isEmpty() && QFile::exists(info.bagPath)) {
        QMessageBox::StandardButton btn = QMessageBox::question(
            this, tr("数据目录丢失"),
            tr("项目「%1」的数据目录无效或缺少 graph.g2o。\n\n"
               "是否从关联的 Bag 重新导入？").arg(info.name),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
        if (btn == QMessageBox::Yes) startImport(info);
        return;
    }

    QMessageBox box(this);
    box.setIcon(QMessageBox::Warning);
    box.setWindowTitle(tr("数据目录丢失"));
    box.setText(tr("项目「%1」的数据目录无效或缺少 graph.g2o。")
                    .arg(info.name));
    QPushButton* relocateBtn = box.addButton(tr("重新定位"), QMessageBox::ActionRole);
    QPushButton* blankBtn =
        box.addButton(tr("以空白项目打开"), QMessageBox::ActionRole);
    box.addButton(QMessageBox::Cancel);
    box.exec();
    if (box.clickedButton() == relocateBtn) {
        QString dir = QFileDialog::getExistingDirectory(
            this, tr("定位数据目录"), info.projectDir,
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
        if (dir.isEmpty()) return;
        ProjectManager::relocateDataDir(info.projectDir, dir);
        auto updated = ProjectManager::read(info.projectDir);
        if (updated.dataDirExists()) {
            finalizeLoad(updated);
        } else {
            QMessageBox::warning(this, tr("数据目录丢失"),
                                 tr("所选目录仍不是有效地图（缺少 graph.g2o）。"));
        }
    } else if (box.clickedButton() == blankBtn) {
        finalizeLoad(info);  // 数据目录无效 → None 任务（空白项目进入主界面）
    }
}

// ---------------------------------------------------------------------------
// 加载 / 导入任务组装
// ---------------------------------------------------------------------------

bool ProjectCenterDialog::finalizeLoad(const ProjectInfo& info) {
    if (info.dataDirExists()) {
        m_task.action = ProjectTask::Action::LoadDirectory;
        m_task.dataDir = info.resolvedDataDir();
    } else {
        m_task.action = ProjectTask::Action::None;  // 空白项目
    }
    m_task.projectDir = info.projectDir;
    m_task.projectName = info.name;
    ProjectManager::markOpened(info.projectDir);
    accept();
    return true;
}

bool ProjectCenterDialog::confirmClearAndImport(const ProjectInfo& info) {
    const QString absData = info.resolvedDataDir();
    if (hdl_graph_slam::BagImporter::isOutputDirNonEmpty(absData.toStdString())) {
        QMessageBox box(this);
        box.setIcon(QMessageBox::Warning);
        box.setWindowTitle(tr("输出目录非空"));
        box.setText(tr("数据目录非空：\n%1\n\n"
                       "导入将永久删除该目录中的所有现有内容（无法撤销）。")
                        .arg(absData));
        auto* clearBtn = box.addButton(tr("清空并导入"), QMessageBox::DestructiveRole);
        box.addButton(QMessageBox::Cancel);
        box.exec();
        if (box.clickedButton() != clearBtn) return false;
        if (!hdl_graph_slam::BagImporter::clearDirectory(absData.toStdString())) {
            QMessageBox::warning(this, tr("清空失败"),
                                 tr("清空数据目录失败：\n%1").arg(absData));
            return false;
        }
    }
    return true;
}

bool ProjectCenterDialog::startImport(const ProjectInfo& info) {
    if (!confirmClearAndImport(info)) return false;

    // 读取 Bag 话题并弹出导入配置对话框（与"打开 Bag"一致的配置流程）
    auto topicsStd = hdl_graph_slam::BagImporter::listTopics(info.bagPath.toStdString());
    if (topicsStd.empty()) {
        QMessageBox::warning(this, tr("读取 Bag 失败"),
                             tr("读取 Bag 失败或未找到话题：\n%1").arg(info.bagPath));
        return false;
    }
    QStringList topics;
    for (auto& t : topicsStd) topics << QString::fromStdString(t);

    auto cfg = hdl_graph_slam::BagImporter::loadConfig(std::string());
    BagOpenDialog dlg(topics, cfg, this);
    if (dlg.exec() != QDialog::Accepted) return false;
    cfg = dlg.config();
    cfg.outputDir = info.resolvedDataDir().toStdString();

    // 先落盘 processing 状态再交给后台导入（MainWindow 完成后写 completed/failed）
    ProjectManager::setStatus(info.projectDir, "processing");

    m_task.action = ProjectTask::Action::ImportBag;
    m_task.projectDir = info.projectDir;
    m_task.projectName = info.name;
    m_task.dataDir = info.resolvedDataDir();
    m_task.bagPath = info.bagPath;
    m_task.importCfg = cfg;
    accept();
    return true;
}

// ---------------------------------------------------------------------------
// 新建项目
// ---------------------------------------------------------------------------

void ProjectCenterDialog::onNewProject() {
    QDialog dlg(this);
    dlg.setWindowTitle(tr("新建项目"));
    auto* form = new QFormLayout(&dlg);

    auto* nameEdit = new QLineEdit;
    nameEdit->setPlaceholderText(tr("项目名称"));
    form->addRow(tr("项目名称:"), nameEdit);

    auto* locEdit = new QLineEdit(QDir::homePath());
    auto* locBrowse = new QPushButton(tr("浏览..."));
    auto* locRow = new QHBoxLayout;
    locRow->addWidget(locEdit, 1);
    locRow->addWidget(locBrowse);
    form->addRow(tr("保存位置:"), locRow);
    connect(locBrowse, &QPushButton::clicked, this, [this, locEdit]() {
        QString d = QFileDialog::getExistingDirectory(
            this, tr("选择保存位置"), locEdit->text(),
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
        if (!d.isEmpty()) locEdit->setText(d);
    });

    auto* bagEdit = new QLineEdit;
    bagEdit->setPlaceholderText(tr("（可选）原始 .bag 文件路径"));
    auto* bagBrowse = new QPushButton(tr("浏览..."));
    auto* bagRow = new QHBoxLayout;
    bagRow->addWidget(bagEdit, 1);
    bagRow->addWidget(bagBrowse);
    form->addRow(tr("Bag 文件:"), bagRow);
    connect(bagBrowse, &QPushButton::clicked, this, [this, bagEdit]() {
        QString f = QFileDialog::getOpenFileName(
            this, tr("选择 Bag 文件"), QString(),
            tr("ROS Bag (*.bag);;所有文件 (*)"));
        if (!f.isEmpty()) bagEdit->setText(f);
    });

    auto* dataEdit = new QLineEdit(ProjectManager::kDefaultDataDir);
    form->addRow(tr("数据目录名:"), dataEdit);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    buttons->button(QDialogButtonBox::Ok)->setText(tr("创建"));
    form->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() != QDialog::Accepted) return;

    const QString name = nameEdit->text().trimmed();
    if (name.isEmpty()) {
        QMessageBox::warning(this, tr("新建项目"), tr("项目名称不能为空"));
        return;
    }
    auto info = ProjectManager::create(name, locEdit->text().trimmed(),
                                       bagEdit->text().trimmed(),
                                       dataEdit->text().trimmed());
    if (!info.valid) {
        QMessageBox::warning(this, tr("新建项目失败"), info.errorString);
        return;
    }

    pushRecentDir(info.projectDir);

    // 边界判定：有 Bag → 确认清空后导入；无 Bag → 校验数据目录或空白打开
    if (!info.bagPath.isEmpty()) {
        if (!QFile::exists(info.bagPath)) {
            QMessageBox::warning(this, tr("Bag 文件不存在"),
                                 tr("未找到 Bag 文件：%1\n"
                                    "已创建空白项目，可稍后在项目中重新导入。")
                                     .arg(info.bagPath));
        } else if (startImport(info)) {
            return;
        }
        // 导入流程被取消 → 留在项目中心（项目已创建，状态 pending）
        refreshRecentList();
        return;
    }

    if (info.dataDirExists()) {
        finalizeLoad(info);
        return;
    }
    QMessageBox::information(
        this, tr("空白项目"),
        tr("项目「%1」已创建（未关联 Bag，数据目录为空）。\n"
           "进入主界面后可稍后导入数据。").arg(name));
    finalizeLoad(info);
}
