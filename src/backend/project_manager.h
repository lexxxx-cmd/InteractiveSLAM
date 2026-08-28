// ============================================================================
// project_manager.h
// 项目管理器 —— "项目"概念的元数据读写与判定逻辑
//
// 功能：一个"项目"= 一个文件夹，内含 project.json（元数据）+ 已处理地图
//       数据目录（可被 GraphManager::openMapData 直接加载），可选关联一个
//       原始 .bag 文件。本类只负责 JSON 的读写、路径解析与状态判定，
//       不做任何 UI 决策（弹窗交由 ProjectCenterDialog）。
//
// import_status 状态机：
//   pending    —— 项目已创建但数据尚未就绪（空白项目或待导入）
//   processing —— Bag 正在后台导入（上次退出若停在此状态视为异常中断）
//   completed  —— 数据目录已就绪，可直接加载
//   failed     —— 上次导入失败
// ============================================================================

#pragma once

#include <QString>
#include <QStringList>
#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QJsonObject>
#include <QObject>
#include <QJsonDocument>

#include <algorithm>
#include <vector>

#include "data/hdl_graph_slam/bag_importer.hpp"

/**
 * @brief 项目元数据（project.json 的内存映像）
 */
struct ProjectInfo {
    QString projectDir;        ///< 项目根目录（绝对路径）
    QString name;              ///< 项目名称
    QString bagPath;           ///< 原始 Bag 路径（可为空）
    QString dataDir;           ///< 数据目录（相对项目根，或绝对路径）
    QString status = "pending";///< import_status（pending/processing/completed/failed）
    QDateTime createdTime;     ///< 创建时间
    QDateTime lastOpenedTime;  ///< 最近打开时间
    bool valid = false;        ///< JSON 是否成功读取
    QString errorString;       ///< 读取失败原因

    /**
     * @brief 解析数据目录为绝对路径
     *
     * data_dir 记录的是相对项目根的路径（推荐）或绝对路径。
     * 解析规则：绝对路径原样返回；否则拼接项目根。
     */
    QString resolvedDataDir() const {
        QDir d(dataDir);
        if (d.isAbsolute()) return QDir::cleanPath(dataDir);
        return QDir::cleanPath(projectDir + "/" + dataDir);
    }

    /** @brief 数据目录是否就绪（含 graph.g2o 即视为合法地图） */
    bool dataDirExists() const {
        return !dataDir.isEmpty() &&
               QFile::exists(resolvedDataDir() + "/graph.g2o");
    }

    /** @brief project.json 文件路径 */
    QString jsonPath() const { return projectDir + "/project.json"; }
};

/**
 * @brief 项目中心产出的启动任务（交给 MainWindow 执行）
 *
 * 项目中心对话框校验完边界情况后，把"接下来该做什么"封装为本结构，
 * MainWindow 据此调用 openMapData() 或 openBagFile()。
 */
struct ProjectTask {
    enum class Action {
        None,          ///< 不做任何事（不应出现）
        LoadDirectory, ///< 直接加载已处理地图目录（dataDir）
        ImportBag      ///< 导入 Bag 并加载（bagPath + importCfg，输出到 dataDir）
    };
    Action action = Action::None;
    QString projectDir;   ///< 项目根目录（空表示无项目，不回写状态）
    QString projectName;  ///< 项目名称（用于窗口标题）
    QString dataDir;      ///< 数据目录绝对路径（LoadDirectory 的加载目标 / ImportBag 的输出目录）
    QString bagPath;      ///< Bag 路径（仅 ImportBag）
    hdl_graph_slam::BagImportConfig importCfg; ///< 导入配置（仅 ImportBag）
};

/**
 * @brief 项目元数据读写工具（全静态方法，无状态）
 */
class ProjectManager {
public:
    static constexpr const char* kFileName = "project.json";
    static constexpr const char* kDefaultDataDir = "map";

    /** @brief 目录是否为一个项目（含 project.json） */
    static bool isProjectDir(const QString& dir) {
        return QFile::exists(dir + "/" + kFileName);
    }

    /** @brief 读取项目元数据；失败时 valid=false 并填充 errorString */
    static ProjectInfo read(const QString& projectDir) {
        ProjectInfo info;
        info.projectDir = QDir::cleanPath(projectDir);

        QFile file(info.jsonPath());
        if (!file.open(QIODevice::ReadOnly)) {
            info.errorString = QObject::tr("Cannot read project.json");
            return info;
        }
        QJsonParseError err;
        auto doc = QJsonDocument::fromJson(file.readAll(), &err);
        file.close();
        if (doc.isNull()) {
            info.errorString = QObject::tr("Failed to parse project.json: %1").arg(err.errorString());
            return info;
        }

        QJsonObject obj = doc.object();
        info.name           = obj.value("project_name").toString();
        info.bagPath        = obj.value("bag_path").toString();
        info.dataDir        = obj.value("data_dir").toString(kDefaultDataDir);
        info.status         = obj.value("import_status").toString("pending");
        info.createdTime    = QDateTime::fromString(obj.value("created_time").toString(), Qt::ISODate);
        info.lastOpenedTime = QDateTime::fromString(obj.value("last_opened_time").toString(), Qt::ISODate);
        info.valid = true;
        return info;
    }

    /** @brief 写入项目元数据（全量覆盖） */
    static bool write(const ProjectInfo& info) {
        QJsonObject obj;
        obj.insert("version", 1);
        obj.insert("project_name", info.name);
        obj.insert("created_time", info.createdTime.toString(Qt::ISODate));
        obj.insert("last_opened_time", info.lastOpenedTime.toString(Qt::ISODate));
        obj.insert("bag_path", info.bagPath);
        obj.insert("data_dir", info.dataDir);
        obj.insert("import_status", info.status);
        if (!obj.value("ui_settings").isObject()) {
            obj.insert("ui_settings", QJsonObject());  // 预留
        }

        QFile file(info.jsonPath());
        if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) return false;
        file.write(QJsonDocument(obj).toJson(QJsonDocument::Indented));
        file.close();
        return true;
    }

    /**
     * @brief 创建新项目
     *
     * 创建项目文件夹与初始 project.json（状态 pending，不在此处启动导入）。
     *
     * @param name       项目名称
     * @param parentDir  项目所在父目录
     * @param bagPath    原始 Bag 路径（可为空）
     * @param dataDirName 数据目录名（相对项目根，默认 "map"）
     * @return 创建好的元数据；失败时 valid=false
     */
    static ProjectInfo create(const QString& name, const QString& parentDir,
                              const QString& bagPath,
                              const QString& dataDirName = QStringLiteral("map")) {
        ProjectInfo info;
        info.projectDir = QDir::cleanPath(parentDir + "/" + name);
        info.name       = name;
        info.bagPath    = bagPath;
        info.dataDir    = dataDirName.isEmpty() ? QStringLiteral("map") : dataDirName;
        info.status     = "pending";
        info.createdTime = info.lastOpenedTime = QDateTime::currentDateTime();

        if (QFile::exists(info.jsonPath())) {
            info.errorString = QObject::tr("Directory already contains a project file");
            return info;
        }
        if (!QDir().mkpath(info.projectDir) || !QDir().mkpath(info.resolvedDataDir())) {
            info.errorString = QObject::tr("Cannot create project directories");
            return info;
        }
        if (!write(info)) {
            info.errorString = QObject::tr("Cannot write project.json");
            return info;
        }
        info.valid = true;
        return info;
    }

    /** @brief 更新导入状态（读改写，保留其他字段） */
    static bool setStatus(const QString& projectDir, const char* status) {
        auto info = read(projectDir);
        if (!info.valid) return false;
        info.status = status;
        return write(info);
    }

    /** @brief 更新最近打开时间 */
    static bool markOpened(const QString& projectDir) {
        auto info = read(projectDir);
        if (!info.valid) return false;
        info.lastOpenedTime = QDateTime::currentDateTime();
        return write(info);
    }

    /** @brief 重新定位数据目录（写回绝对路径） */
    static bool relocateDataDir(const QString& projectDir, const QString& absPath) {
        auto info = read(projectDir);
        if (!info.valid) return false;
        info.dataDir = QDir::cleanPath(absPath);
        return write(info);
    }

    /**
     * @brief 扫描最近项目列表（按 last_opened_time 降序）
     *
     * 逐项读取 project.json；读取失败的项仍然返回（valid=false），
     * 由 UI 决定展示方式（置灰/仅显示路径）。
     *
     * @param dirs 候选项目目录列表（来自 QSettings 的 MRU）
     */
    static std::vector<ProjectInfo> loadRecent(const QStringList& dirs) {
        std::vector<ProjectInfo> list;
        for (const QString& d : dirs) {
            auto info = read(d);
            info.lastOpenedTime = info.lastOpenedTime.isValid()
                                      ? info.lastOpenedTime
                                      : QDateTime::fromSecsSinceEpoch(0);  // 排序稳定
            list.push_back(info);
        }
        std::sort(list.begin(), list.end(),
                  [](const ProjectInfo& a, const ProjectInfo& b) {
                      return a.lastOpenedTime > b.lastOpenedTime;
                  });
        return list;
    }
};
