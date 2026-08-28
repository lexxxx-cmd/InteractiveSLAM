// ============================================================================
// main.cpp
// 应用程序入口点
//
// 功能：初始化 Qt 应用程序环境，加载中文本地化翻译，
//       创建图管理器后端和主窗口，启动事件循环。
// ============================================================================

#include <QApplication>
#include <QSurfaceFormat>
#include <QTranslator>
#include <QLibraryInfo>
#include <QFile>
#include <QTextStream>
#include <QIcon>
#include <iostream>

// 第三方依赖头文件（编译时检查可用性）
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <g2o/core/hyper_graph.h>

// 后端
#include "backend/graph_manager.hpp"
#include "ui/MainWindow.h"
#include "ui/ProjectCenterDialog.h"

/**
 * @brief 应用程序入口点
 *
 * 启动流程：
 *   1. 创建 QApplication 实例
 *   2. 加载中文本地化翻译文件（应用自定义翻译 + Qt 内置翻译）
 *   3. 创建 GraphManager 后端管理器（应用程序生命周期内的单例）
 *   4. 创建并显示主窗口
 *   5. 进入 Qt 事件循环
 *
 * @param argc  命令行参数数量
 * @param argv  命令行参数数组
 * @return 应用程序退出码
 */
int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // --- 加载中文翻译 ---

    // 加载应用程序自定义的中文翻译文件（位于 resources:/i18n/app_zh_CN）
    QTranslator appTranslator;
    if (appTranslator.load(":/i18n/app_zh_CN")) {
        app.installTranslator(&appTranslator);
    }

    // 加载 Qt 内置控件的中文翻译（按钮文字如"确定/取消"、"打开"等）
    QTranslator qtTranslator;
    if (qtTranslator.load(QLocale::Chinese, "qt", "_",
                          QLibraryInfo::path(QLibraryInfo::TranslationsPath))) {
        app.installTranslator(&qtTranslator);
    }

    // --- 加载全局暗色主题样式表 ---
    QFile styleFile(":/ui/dark_theme.qss");
    if (styleFile.open(QFile::ReadOnly | QFile::Text)) {
        QTextStream ts(&styleFile);
        app.setStyleSheet(ts.readAll());
        styleFile.close();
    }

    // --- 设置应用图标（来自 src/icon 文件夹，qrc 资源） ---
    app.setWindowIcon(QIcon(":/ui/app_icon.jpg"));

    // 创建图管理器后端实例（应用程序生命周期内作为全局单例使用）
    GraphManager graphManager;

    // --- 项目中心：新建/打开项目，产出启动任务 ---
    // 用户取消（关闭项目中心）则直接退出程序
    ProjectCenterDialog center;
    if (center.exec() != QDialog::Accepted) return 0;
    const ProjectTask task = center.task();

    // 创建并显示主窗口，按启动任务加载/导入数据
    MainWindow mainWindow(&graphManager);
    mainWindow.show();
    mainWindow.launchFromProject(task);

    // 进入 Qt 事件循环
    return app.exec();
}
