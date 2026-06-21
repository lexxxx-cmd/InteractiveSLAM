#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>
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
#include "viewport/graph_viewport.hpp"
#include "viewport/osg_viewport.hpp"

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

    // Register QML types
    qmlRegisterType<GraphViewport>("InteractiveSLAM", 1, 0, "GraphViewport");
    qmlRegisterType<OSGViewport>("InteractiveSLAM", 1, 0, "OSGViewport");

    // Create backend manager and expose to QML as singleton
    GraphManager graphManager;
    qmlRegisterSingletonInstance<GraphManager>("InteractiveSLAM", 1, 0, "GraphManager", &graphManager);

    QQmlApplicationEngine engine;

    QObject::connect(&engine, &QQmlApplicationEngine::objectCreationFailed,
        &app, []() { QCoreApplication::exit(-1); }, Qt::QueuedConnection);
    engine.loadFromModule("InteractiveSLAM", "Main");
    return app.exec();
}
