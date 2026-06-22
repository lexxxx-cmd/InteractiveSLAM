#include <QApplication>
#include <QSurfaceFormat>
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

    // Create backend manager (singleton for the application lifetime)
    GraphManager graphManager;

    MainWindow mainWindow(&graphManager);
    mainWindow.show();

    return app.exec();
}
