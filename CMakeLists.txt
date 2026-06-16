cmake_minimum_required(VERSION 3.16)

project(InteractiveSLAM VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTORCC ON)  # Auto-process .qrc files in source lists

# Third-party find modules (before Qt, so vcpkg toolchain can inject paths)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io common search features)
find_package(g2o REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

# Qt6
find_package(Qt6 REQUIRED COMPONENTS Quick Concurrent)
qt_standard_project_setup(REQUIRES 6.8)

# Source files (Phase B: data loading layer)
set(DATA_SOURCES
    src/data/hdl_graph_slam/keyframe.cpp
    src/data/hdl_graph_slam/graph_slam.cpp
    src/data/hdl_graph_slam/information_matrix_calculator.cpp
    src/data/hdl_graph_slam/interactive_keyframe.cpp
    src/data/hdl_graph_slam/interactive_graph.cpp
    src/data/g2o/robust_kernel_io.cpp
)

# Phase C: Qt backend
set(BACKEND_SOURCES
    src/backend/progress_reporter.cpp
    src/backend/graph_manager.cpp
)

# Phase D: OpenGL rendering
set(GLK_SOURCES
    src/glk/glsl_shader.cpp
    src/glk/mesh.cpp
    src/glk/primitives.cpp
    src/glk/pointcloud_buffer.cpp
    src/glk/lines.cpp
)

set(VIEWPORT_SOURCES
    src/viewport/graph_viewport.cpp
    src/viewport/graph_viewport_renderer.cpp
)

qt_add_executable(appInteractiveSLAM
    main.cpp
    ${DATA_SOURCES}
    ${BACKEND_SOURCES}
    ${GLK_SOURCES}
    ${VIEWPORT_SOURCES}
    resources/shaders.qrc
)

# Shader resources — .qrc processed by CMAKE_AUTORCC

# Standalone CLI test for data loading (Phase B verification)
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

qt_add_qml_module(appInteractiveSLAM
    URI InteractiveSLAM
    VERSION 1.0
    QML_FILES
        Main.qml
)

target_include_directories(appInteractiveSLAM PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)

target_link_libraries(appInteractiveSLAM PRIVATE
    Qt6::Quick
    Qt6::Concurrent
    ${PCL_LIBRARIES}
    ${G2O_LIBRARIES}
    Boost::filesystem
)

# Ensure the target compiles despite PCL/g2o header warnings on MSVC
if(MSVC)
    target_compile_options(appInteractiveSLAM PRIVATE /W3 /wd4996)
endif()

install(TARGETS appInteractiveSLAM
    BUNDLE DESTINATION .
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)
