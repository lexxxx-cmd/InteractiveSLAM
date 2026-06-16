#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include <QSGRendererInterface>
#include <QSurfaceFormat>
#include <iostream>

// Third-party dependency headers (verify compile-time availability)
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <g2o/core/hyper_graph.h>

// Backend
#include "backend/graph_manager.hpp"

int main(int argc, char *argv[]) {
    // ⚠ Must be set BEFORE QGuiApplication creation!
    // Qt6 defaults to D3D11 on Windows — QOpenGLContext would return nullptr.
    // Explicitly force OpenGL RHI backend.
    QQuickWindow::setGraphicsApi(QSGRendererInterface::OpenGL);

    // Specify OpenGL version and profile
    QSurfaceFormat format;
    format.setVersion(3, 3);
    format.setProfile(QSurfaceFormat::CoreProfile);
    format.setDepthBufferSize(24);
    format.setStencilBufferSize(8);
    QSurfaceFormat::setDefaultFormat(format);

    // Verify third-party dependencies
    std::cout << "InterSLAM: All dependencies loaded successfully" << std::endl;
    std::cout << "Eigen version: " << EIGEN_WORLD_VERSION << "."
              << EIGEN_MAJOR_VERSION << "." << EIGEN_MINOR_VERSION << std::endl;
    std::cout << "PCL version: " << PCL_VERSION_PRETTY << std::endl;
    std::cout << "g2o headers: available" << std::endl;
    std::cout << "RHI backend: OpenGL (forced)" << std::endl;

    QGuiApplication app(argc, argv);

    // Create backend manager and expose to QML
    GraphManager graphManager;
    graphManager.setObjectName("graphManager");

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("graphManager", &graphManager);

    QObject::connect(&engine, &QQmlApplicationEngine::objectCreationFailed,
        &app, []() { QCoreApplication::exit(-1); }, Qt::QueuedConnection);
    engine.loadFromModule("InteractiveSLAM", "Main");
    return app.exec();
}
