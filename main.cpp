#include <QApplication>
#include <QSurfaceFormat>
#include <QTranslator>
#include <QLibraryInfo>
#include <iostream>

// Third-party dependency headers (verify compile-time availability)
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <g2o/core/hyper_graph.h>

// Backend
#include "backend/graph_manager.hpp"
#include "ui/MainWindow.h"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // --- 加载中文翻译 ---
    QTranslator appTranslator;
    if (appTranslator.load(":/i18n/app_zh_CN")) {
        app.installTranslator(&appTranslator);
    }

    // 加载 Qt 内置控件的中文翻译（按钮文字如"确定/取消"等）
    QTranslator qtTranslator;
    if (qtTranslator.load(QLocale::Chinese, "qt", "_",
                          QLibraryInfo::path(QLibraryInfo::TranslationsPath))) {
        app.installTranslator(&qtTranslator);
    }

    // Create backend manager (singleton for the application lifetime)
    GraphManager graphManager;

    MainWindow mainWindow(&graphManager);
    mainWindow.show();

    return app.exec();
}
