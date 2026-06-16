cmake_minimum_required(VERSION 3.16)

project(InteractiveSLAM VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Third-party find modules (before Qt, so vcpkg toolchain can inject paths)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS io common search features)
find_package(g2o REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

# Qt6
find_package(Qt6 REQUIRED COMPONENTS Quick)
qt_standard_project_setup(REQUIRES 6.8)

# Source files (expanded in later phases)
qt_add_executable(appInteractiveSLAM
    main.cpp
)

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
