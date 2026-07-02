cmake_minimum_required(VERSION 3.16)

project(InteractiveSLAM VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# Third-party find modules
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io common search features registration)
find_package(g2o REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

# OpenMP (for ndt_omp multi-threaded GICP / NDT)
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(HAS_PCLOMP ON)
    message(STATUS "OpenMP found — enabling GICP_OMP / NDT_OMP")
else()
    message(WARNING "OpenMP not found — GICP_OMP / NDT_OMP will be unavailable")
endif()

# OpenSceneGraph
find_package(OpenSceneGraph REQUIRED
    COMPONENTS osg osgViewer osgGA osgUtil osgDB OpenThreads)

# Qt6 — Widgets-based UI (replaces QML)
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets Concurrent)

# --- Source files ---

# Data layer (unchanged except for progress_interface.hpp shim)
set(DATA_SOURCES
    src/data/hdl_graph_slam/keyframe.cpp
    src/data/hdl_graph_slam/graph_slam.cpp
    src/data/hdl_graph_slam/information_matrix_calculator.cpp
    src/data/hdl_graph_slam/interactive_keyframe.cpp
    src/data/hdl_graph_slam/interactive_graph.cpp
    src/data/hdl_graph_slam/registration_methods.cpp
    src/data/hdl_graph_slam/automatic_loop_closure.cpp
    src/data/g2o/robust_kernel_io.cpp
)

# Qt backend bridge (adapted for Widgets)
set(BACKEND_SOURCES
    src/backend/progress_reporter.cpp
    src/backend/graph_manager.cpp
)

# Qt Widgets UI (new — replaces Main.qml + QML viewport)
set(UI_SOURCES
    src/ui/MainWindow.cpp
    src/ui/ViewportWidget.cpp
    src/ui/GraphStatsPanel.cpp
    src/ui/RenderingPanel.cpp
    src/ui/ZClippingPanel.cpp
    src/ui/ColorRangePanel.cpp
    src/ui/MiniViewportWidget.cpp
    src/ui/LoopClosureDialog.cpp
    src/ui/AutoLoopClosurePanel.cpp
    src/ui/EdgeListPanel.cpp
    src/ui/OverlayPanelWidget.cpp
)

# ndt_omp_MSVC — multi-threaded GICP / NDT (forked with PCL 1.12+/MSVC fixes)
if(HAS_PCLOMP)
    add_subdirectory(${CMAKE_SOURCE_DIR}/thirdparty/ndt_omp_MSVC)
endif()

# osgQOpenGL adapter library (now full Widgets integration)
add_subdirectory(src/osgQOpenGL)
if(MSVC)
    add_compile_options(/utf-8)
endif()

# --- Main executable ---
qt_add_executable(appInteractiveSLAM
    main.cpp
    ${DATA_SOURCES}
    ${BACKEND_SOURCES}
    ${UI_SOURCES}
)

target_include_directories(appInteractiveSLAM PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(appInteractiveSLAM PRIVATE
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    Qt6::Concurrent
    osgQOpenGL
    ${PCL_LIBRARIES}
    ${G2O_LIBRARIES}
    Boost::filesystem
    ${OPENSCENEGRAPH_LIBRARIES}
    ${OpenMP_CXX_LIBRARIES}
)

# ndt_omp: link + include propagation
if(HAS_PCLOMP)
    target_link_libraries(appInteractiveSLAM PRIVATE ndt_omp)
endif()

# Enable HAS_PCLOMP macro so registration_methods.cpp can use GICP_OMP / NDT_OMP
if(HAS_PCLOMP AND OpenMP_CXX_FOUND)
    target_compile_definitions(appInteractiveSLAM PRIVATE HAS_PCLOMP)
    target_compile_options(appInteractiveSLAM PRIVATE ${OpenMP_CXX_FLAGS})
endif()

if(MSVC)
    target_compile_options(appInteractiveSLAM PRIVATE /W3 /wd4996)
endif()

# --- Standalone CLI test (unchanged) ---
add_executable(test_data_loading
    tests/test_data_loading.cpp
    ${DATA_SOURCES}
)
target_include_directories(test_data_loading PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(test_data_loading PRIVATE
    ${PCL_LIBRARIES}
    ${G2O_LIBRARIES}
    Boost::filesystem
)
if(MSVC)
    target_compile_options(test_data_loading PRIVATE /W3 /wd4996)
endif()

install(TARGETS appInteractiveSLAM
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
