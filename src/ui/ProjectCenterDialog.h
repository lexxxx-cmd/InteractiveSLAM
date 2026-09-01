// ============================================================================
// ProjectCenterDialog.h
// 项目中心对话框
//
// 启动时最先弹出的模态对话框（参照 Bridge3D Studio 布局）：
//   - 左侧：最近项目列表（占位缩略图 + 名称 + 最近打开时间 + 路径）
//   - 右侧：新建项目 / 打开项目
//
// 职责：新建/打开的全部边界判定（清空确认、崩溃恢复、路径丢失重定位），
//       校验通过后产出 ProjectTask 交给 MainWindow 执行。
//       本对话框不做任何数据加载（Bag 导入耗时长，由 MainWindow 后台执行）。
// ============================================================================

#pragma once

#include <QDialog>
#include <QList>
#include <QListWidget>

#include "backend/project_manager.h"

class QLabel;
class QListWidgetItem;
class QLineEdit;

class ProjectCenterDialog : public QDialog {
    Q_OBJECT
public:
    explicit ProjectCenterDialog(QWidget* parent = nullptr);

    /** @brief 校验完成后的启动任务（exec() 返回 Accepted 后有效） */
    ProjectTask task() const { return m_task; }

protected:
    void accept() override;  ///< 记录最近项目 MRU 后关闭

private slots:
    void onNewProject();       ///< 新建项目（表单 + 边界判定）
    void onOpenProject();      ///< 打开选中的最近项目（无选中则浏览目录）
    void onOpenRecentItem(QListWidgetItem* item);  ///< 双击最近项目打开

private:
    // --- 打开判定链 ---
    void openProjectDir(const QString& dir);      ///< 入口：读取并判定
    void handleOpenInfo(const ProjectInfo& info); ///< 按 status 分派
    bool finalizeLoad(const ProjectInfo& info);   ///< 组装 LoadDirectory/None 任务并接受
    bool startImport(const ProjectInfo& info);    ///< 组装 ImportBag 任务（含导入配置弹窗）并接受
    bool confirmClearAndImport(const ProjectInfo& info); ///< 数据目录非空时的清空确认
    void refreshRecentList();

    // --- 最近列表条目操作（⋯ 按钮） ---
    void showItemMenu(const QString& dir);  ///< 弹出条目操作菜单（重命名/删除）
    void renameProject(const QString& dir); ///< 重命名项目（修改 project_name）
    void removeProject(const QString& dir); ///< 删除项目（移出列表/删除文件夹）

    ProjectTask m_task;            ///< 最终启动任务
    QListWidget* m_recentList;     ///< 最近项目列表
};
