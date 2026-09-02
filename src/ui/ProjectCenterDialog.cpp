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
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QMenu>
#include <QMessageBox>
#include <QCoreApplication>
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

QString findBagImportConfigYaml() {
    const QStringList candidates = {
        QCoreApplication::applicationDirPath() + "/config/bag_import.yaml",
        QDir::currentPath() + "/config/bag_import.yaml",
    };
    for (const auto& p : candidates) {
        if (QFileInfo::exists(p)) return p;
    }
    return QString();
}

ProjectCenterDialog::ProjectCenterDialog(QWidget* parent)
    : QDialog(parent) {
    setWindowTitle(tr("Project Center"));
    setModal(true);
    resize(760, 480);

    auto* rootLayout = new QHBoxLayout(this);

    // ---- 左侧：最近项目 ----
    auto* leftLayout = new QVBoxLayout;
    auto* recentTitle = new QLabel(tr("Recent Projects"));
    recentTitle->setStyleSheet("font-weight: bold; font-size: 14px;");
    leftLayout->addWidget(recentTitle);

    m_recentList = new QListWidget;
    m_recentList->setSelectionMode(QAbstractItemView::SingleSelection);
    // 悬停/选中高亮（与 ⋯ 按钮 hover 同色系），便于确认将要打开的项目。
    // 行 widget 与内部 QLabel 均为透明背景，item 高亮可透出显示
    m_recentList->setObjectName("recentProjectList");
    m_recentList->setStyleSheet(
        "QListWidget#recentProjectList { background: transparent; }"
        /* 基础态：透明边框占位，保证各状态下盒子几何一致不跳动 */
        "QListWidget#recentProjectList::item {"
        "    border-radius: 6px;"
        "    margin: 2px 4px;"
        "    padding: 4px;"
        "    border: 1px solid transparent;"
        "}"
        /* 悬停：轻量高亮 */
        "QListWidget#recentProjectList::item:hover {"
        "    background: #3a4a6a;"
        "}"
        /* 选中：更重的颜色 + 四边统一边框，左侧 3px 强调条 */
        "QListWidget#recentProjectList::item:selected {"
        "    background: #4a5a8a;"
        "    border: 1px solid #66aaff;"
        "}"
        /* 选中+悬停：比单纯选中再亮一点 */
        "QListWidget#recentProjectList::item:hover:selected {"
        "    background: #5a6a9a;"
        "    border: 1px solid #88ccff;"
        "}"
    );
    connect(m_recentList, &QListWidget::itemDoubleClicked,
            this, &ProjectCenterDialog::onOpenRecentItem);
    leftLayout->addWidget(m_recentList, 1);
    rootLayout->addLayout(leftLayout, 3);

    // ---- 右侧：开始使用 ----
    auto* rightLayout = new QVBoxLayout;
    auto* startTitle = new QLabel(tr("Get Started"));
    startTitle->setStyleSheet("font-weight: bold; font-size: 18px;");
    rightLayout->addWidget(startTitle);

    auto* newBtn = new QPushButton(tr("New Project"));
    newBtn->setMinimumHeight(56);
    newBtn->setObjectName("primaryButton");
    newBtn->setToolTip(tr("Set project name and location; optionally link a raw Bag file"));
    auto* openBtn = new QPushButton(tr("Open Project"));
    openBtn->setMinimumHeight(56);
    openBtn->setToolTip(tr("Open an existing project from the recent list or a local directory"));
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
        // 48×48：小于右侧三行文字的高度（条目高度的真正决定者），
        // 任何 ::item 盒模型变化下都有富余，不会被下缘截断
        thumb->setFixedSize(48, 48);
        thumb->setAlignment(Qt::AlignCenter);
        // 占位缩略图：深色底 + 项目名首字符（后续版本替换为主视口截图）
        thumb->setStyleSheet(
            "background-color: #1e2430; color: #8899aa; font-size: 20px;"
            "border: 1px solid #33405a; border-radius: 4px;");
        thumb->setText(info.name.isEmpty() ? "?" : info.name.left(1).toUpper());
        layout->addWidget(thumb, 0, Qt::AlignVCenter);

        auto* textLayout = new QVBoxLayout;
        auto* name = new QLabel(info.name.isEmpty()
                                    ? QDir(info.projectDir).dirName() : info.name);
        name->setStyleSheet("font-weight: bold;");
        auto statusText = info.valid ? info.status : QString("invalid");
        auto* meta = new QLabel(tr("Last opened: %1    Status: %2")
                                    .arg(info.lastOpenedTime.isValid()
                                             ? info.lastOpenedTime.toString("yyyy/MM/dd HH:mm")
                                             : tr("Unknown"),
                                         statusText));
        // 允许被布局压缩：QLabel 默认最小宽度 = 整行文字宽度，
        // 路径过长时会把最右侧的 ⋯ 按钮挤出裁剪区（按钮"消失"的根源）
        meta->setMinimumSize(0, 0);
        auto* path = new QLabel(info.projectDir);
        path->setStyleSheet("color: #8899aa;");
        path->setMinimumSize(0, 0);
        path->setToolTip(info.projectDir);  // 被截断时悬停可看完整路径
        textLayout->addWidget(name);
        textLayout->addWidget(meta);
        textLayout->addWidget(path);
        layout->addLayout(textLayout, 1);

        // 右对齐 ⋯ 按钮：弹出该条目的操作菜单（重命名/删除）。
        // 按值捕获目录——info 是循环引用，按钮点击发生在循环结束后。
        // 图标用 src/icon/dots.svg（Tabler 三点图标，qrc 内嵌）
        auto* moreBtn = new QPushButton;
        moreBtn->setFixedSize(24, 24);
        moreBtn->setFlat(true);
        moreBtn->setIcon(QIcon(":/ui/dots.svg"));
        moreBtn->setIconSize(QSize(16, 16));
        moreBtn->setStyleSheet(
            "QPushButton { border: none; border-radius: 4px; }"
            "QPushButton:hover { background: #2a3550; }");
        connect(moreBtn, &QPushButton::clicked, this,
                [this, dir = info.projectDir]() { showItemMenu(dir); });
        layout->addWidget(moreBtn);

        item->setData(Qt::UserRole, info.projectDir);
        // 宽度传 0：QListView 用视口宽度铺满条目（通栏）。
        // 若用 row->sizeHint().width()（含标签完整文字宽度，通常比视口宽），
        // 会出现横向溢出，选中边框右侧被裁掉。
        // 高度补偿 ::item 的上下 margin（2px×2）+ 2px 余量：
        // 行 widget 铺满条目矩形，而边框盒被 margin 内缩，
        // 不补偿则内容底部顶出圆角边框（下缘被截）
        item->setSizeHint(QSize(0, row->sizeHint().height() + 6));
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
// 最近列表条目操作（⋯ 按钮）
// ---------------------------------------------------------------------------

void ProjectCenterDialog::showItemMenu(const QString& dir) {
    QMenu menu(this);
    QAction* renameAct = menu.addAction(tr("Rename"));
    menu.addSeparator();
    QAction* removeAct = menu.addAction(tr("Remove"));

    QAction* chosen = menu.exec(QCursor::pos());
    if (chosen == renameAct) {
        renameProject(dir);
    } else if (chosen == removeAct) {
        removeProject(dir);
    }
}

void ProjectCenterDialog::renameProject(const QString& dir) {
    auto info = ProjectManager::read(dir);
    if (!info.valid) {
        QMessageBox::warning(this, tr("Open Project Failed"), info.errorString);
        return;
    }
    bool ok = false;
    QString name = QInputDialog::getText(
        this, tr("Rename Project"), tr("Project name:"),
        QLineEdit::Normal, info.name, &ok);
    if (!ok) return;
    name = name.trimmed();
    if (name.isEmpty() || name == info.name) return;

    info.name = name;
    if (!ProjectManager::write(info)) {
        QMessageBox::warning(this, tr("Rename Project"),
                             tr("Cannot write project.json"));
        return;
    }
    refreshRecentList();
}

void ProjectCenterDialog::removeProject(const QString& dir) {
    auto info = ProjectManager::read(dir);

    QMessageBox box(this);
    box.setIcon(QMessageBox::Warning);
    box.setWindowTitle(tr("Remove Project"));
    box.setText(tr("Remove project \"%1\"?\n\n%2")
                    .arg(info.valid ? info.name : QDir(dir).dirName(), dir));
    QPushButton* listBtn =
        box.addButton(tr("Remove from List"), QMessageBox::ActionRole);
    QPushButton* folderBtn =
        box.addButton(tr("Delete Project Folder"), QMessageBox::DestructiveRole);
    box.addButton(QMessageBox::Cancel);
    box.exec();
    auto clicked = box.clickedButton();

    if (clicked == listBtn) {
        // 仅移出最近列表，不动磁盘文件
        QSettings settings("DAFTECH", "InteractiveSLAM");
        QStringList list = settings.value("recent_projects").toStringList();
        list.removeAll(dir);
        settings.setValue("recent_projects", list);
        refreshRecentList();
    } else if (clicked == folderBtn) {
        // 删除磁盘上的项目文件夹（含地图数据，破坏性操作，路径已在上方展示）
        QDir d(dir);
        if (d.removeRecursively()) {
            QSettings settings("DAFTECH", "InteractiveSLAM");
            QStringList list = settings.value("recent_projects").toStringList();
            list.removeAll(dir);
            settings.setValue("recent_projects", list);
            refreshRecentList();
        } else {
            QMessageBox::warning(this, tr("Remove Project"),
                                 tr("Failed to delete the project folder:\n%1")
                                     .arg(dir));
        }
    }
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
        this, tr("Choose Project Directory"), QString(),
        QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
    if (!dir.isEmpty()) openProjectDir(dir);
}

void ProjectCenterDialog::onOpenRecentItem(QListWidgetItem*) {
    onOpenProject();
}

void ProjectCenterDialog::openProjectDir(const QString& dir) {
    if (!ProjectManager::isProjectDir(dir)) {
        // 不是项目目录：兼容旧行为——当作纯地图目录直接加载（不回写项目状态）
        QMessageBox::StandardButton btn = QMessageBox::question(
            this, tr("Not a Project Directory"),
            tr("The selected directory is not a project (missing project.json).\n\n"
               "Open it directly as a map directory?"),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
        if (btn != QMessageBox::Yes) return;
        m_task.action = ProjectTask::Action::LoadDirectory;
        m_task.dataDir = QDir::cleanPath(dir);
        accept();
        return;
    }

    auto info = ProjectManager::read(dir);
    if (!info.valid) {
        QMessageBox::warning(this, tr("Open Project Failed"), info.errorString);
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
        box.setWindowTitle(tr("Previous Import Not Completed"));
        box.setText(tr("Project \"%1\" did not finish its last import (status: %2).\n"
                       "The data directory may be incomplete. How to proceed?")
                        .arg(info.name, info.status));
        QPushButton* reimportBtn =
            box.addButton(tr("Re-import"), QMessageBox::DestructiveRole);
        QPushButton* loadBtn =
            box.addButton(tr("Load Existing Files"), QMessageBox::AcceptRole);
        box.addButton(QMessageBox::Cancel);
        box.exec();
        auto clicked = box.clickedButton();

        if (clicked == reimportBtn) {
            if (info.bagPath.isEmpty() || !QFile::exists(info.bagPath)) {
                QMessageBox::warning(this, tr("Cannot Re-import"),
                                     tr("Linked Bag file not found: %1").arg(info.bagPath));
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
            this, tr("Data Directory Missing"),
            tr("The data directory of project \"%1\" is invalid or missing graph.g2o.\n\n"
               "Re-import from the linked Bag?").arg(info.name),
            QMessageBox::Yes | QMessageBox::No, QMessageBox::Yes);
        if (btn == QMessageBox::Yes) startImport(info);
        return;
    }

    QMessageBox box(this);
    box.setIcon(QMessageBox::Warning);
    box.setWindowTitle(tr("Data Directory Missing"));
    box.setText(tr("The data directory of project \"%1\" is invalid or missing graph.g2o.")
                    .arg(info.name));
    QPushButton* relocateBtn = box.addButton(tr("Relocate"), QMessageBox::ActionRole);
    QPushButton* blankBtn =
        box.addButton(tr("Open as Blank Project"), QMessageBox::ActionRole);
    box.addButton(QMessageBox::Cancel);
    box.exec();
    if (box.clickedButton() == relocateBtn) {
        QString dir = QFileDialog::getExistingDirectory(
            this, tr("Locate Data Directory"), info.projectDir,
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
        if (dir.isEmpty()) return;
        ProjectManager::relocateDataDir(info.projectDir, dir);
        auto updated = ProjectManager::read(info.projectDir);
        if (updated.dataDirExists()) {
            finalizeLoad(updated);
        } else {
            QMessageBox::warning(this, tr("Data Directory Missing"),
                                 tr("The selected directory is still not a valid map (missing graph.g2o)."));
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
        box.setWindowTitle(tr("Output Directory Not Empty"));
        box.setText(tr("The output directory is not empty:\n%1\n\n"
                       "Importing will permanently delete all existing content "
                       "in this directory (cannot be undone).").arg(absData));
        auto* clearBtn = box.addButton(tr("Clear & Import"), QMessageBox::DestructiveRole);
        box.addButton(QMessageBox::Cancel);
        box.exec();
        if (box.clickedButton() != clearBtn) return false;
        if (!hdl_graph_slam::BagImporter::clearDirectory(absData.toStdString())) {
            QMessageBox::warning(this, tr("Clear Failed"),
                                 tr("Failed to clear the data directory:\n%1").arg(absData));
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
        QMessageBox::warning(this, tr("Read Bag Failed"),
                             tr("Failed to read bag or no topics found:\n%1").arg(info.bagPath));
        return false;
    }
    QStringList topics;
    for (auto& t : topicsStd) topics << QString::fromStdString(t);

    // 优先用找到的 bag_import.yaml 预填默认值；未找到时直接默认构造
    // （不要把空路径传给 loadConfig——会打 "YAML parse error" 日志再回退）
    const QString yamlPath = findBagImportConfigYaml();
    auto cfg = yamlPath.isEmpty()
                   ? hdl_graph_slam::BagImportConfig{}
                   : hdl_graph_slam::BagImporter::loadConfig(yamlPath.toStdString());
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
    dlg.setWindowTitle(tr("New Project"));
    auto* form = new QFormLayout(&dlg);

    auto* nameEdit = new QLineEdit;
    nameEdit->setPlaceholderText(tr("Project name"));
    form->addRow(tr("Project name:"), nameEdit);

    auto* locEdit = new QLineEdit(QDir::homePath());
    auto* locBrowse = new QPushButton(tr("Browse..."));
    auto* locRow = new QHBoxLayout;
    locRow->addWidget(locEdit, 1);
    locRow->addWidget(locBrowse);
    form->addRow(tr("Save location:"), locRow);
    connect(locBrowse, &QPushButton::clicked, this, [this, locEdit]() {
        QString d = QFileDialog::getExistingDirectory(
            this, tr("Choose Save Location"), locEdit->text(),
            QFileDialog::ShowDirsOnly | QFileDialog::DontResolveSymlinks);
        if (!d.isEmpty()) locEdit->setText(d);
    });

    auto* bagEdit = new QLineEdit;
    bagEdit->setPlaceholderText(tr("(Optional) Path to raw .bag file"));
    auto* bagBrowse = new QPushButton(tr("Browse..."));
    auto* bagRow = new QHBoxLayout;
    bagRow->addWidget(bagEdit, 1);
    bagRow->addWidget(bagBrowse);
    form->addRow(tr("Bag file:"), bagRow);
    connect(bagBrowse, &QPushButton::clicked, this, [this, bagEdit]() {
        QString f = QFileDialog::getOpenFileName(
            this, tr("Choose Bag File"), QString(),
            tr("ROS Bag (*.bag);;All Files (*)"));
        if (!f.isEmpty()) bagEdit->setText(f);
    });

    auto* dataEdit = new QLineEdit(ProjectManager::kDefaultDataDir);
    form->addRow(tr("Data directory name:"), dataEdit);

    auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel);
    buttons->button(QDialogButtonBox::Ok)->setText(tr("Create"));
    form->addRow(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() != QDialog::Accepted) return;

    const QString name = nameEdit->text().trimmed();
    if (name.isEmpty()) {
        QMessageBox::warning(this, tr("New Project"), tr("Project name cannot be empty"));
        return;
    }
    auto info = ProjectManager::create(name, locEdit->text().trimmed(),
                                       bagEdit->text().trimmed(),
                                       dataEdit->text().trimmed());
    if (!info.valid) {
        QMessageBox::warning(this, tr("Create Project Failed"), info.errorString);
        return;
    }

    pushRecentDir(info.projectDir);

    // 边界判定：有 Bag → 确认清空后导入；无 Bag → 校验数据目录或空白打开
    if (!info.bagPath.isEmpty()) {
        if (!QFile::exists(info.bagPath)) {
            QMessageBox::warning(this, tr("Bag File Not Found"),
                                 tr("Bag file not found: %1\n"
                                    "A blank project has been created; you can re-import later.")
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
        this, tr("Blank Project"),
        tr("Project \"%1\" created (no Bag linked, empty data directory).\n"
           "You can import data later in the main window.").arg(name));
    finalizeLoad(info);
}
