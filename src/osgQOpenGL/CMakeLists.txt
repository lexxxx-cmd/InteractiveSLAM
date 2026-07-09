# ============================================================================
# @file CMakeLists.txt
# @brief osgQOpenGL 库的 CMake 构建配置文件
#
# 本文件定义 osgQOpenGL 静态库的构建规则。
# osgQOpenGL 是 InterSLAM 项目中的 OSG-Qt 集成层，
# 提供将 OpenSceneGraph (OSG) 渲染引擎与 Qt6 图形界面框架集成的核心功能。
#
# 构建的静态库包含以下组件：
#   CullVisitorEx    - 自定义裁剪访问器扩展
#   GraphicsWindowEx - 扩展的图形窗口实现
#   OSGRenderer      - OSG 渲染器（事件处理、帧更新管理）
#   RenderStageEx    - 自定义渲染阶段扩展
#   StateEx          - OpenGL 状态管理扩展
#   osgQOpenGLWidget - 基于 QOpenGLWidget 的 OSG 集成部件
#   osgQOpenGLWindow - 基于 QOpenGLWindow 的 OSG 集成窗口
#
# 依赖项：
#   - Qt6 (Widgets, OpenGL, OpenGLWidgets)
#   - OpenSceneGraph (OSG)
# ============================================================================

# ============================================================================
# 仅在找到 Qt6Widgets 时构建（避免在没有 Qt6 的环境中出错）
# ============================================================================
IF ( Qt6Widgets_FOUND )
    # 生成导出头文件（用于控制符号的导入/导出）
    include(GenerateExportHeader)

    # 库名称
    SET(LIB_NAME osgQOpenGL)

    # 包含当前源目录，使头文件可被找到
    set(CMAKE_INCLUDE_CURRENT_DIR yes)

    # ============================================================================
    # 源文件配置
    # 使用当前目录下的头文件（而不是不存在的 include 目录）
    # ============================================================================
    SET(SOURCE_H
        osgQOpenGLWidget.h
        osgQOpenGLWindow.h
        OSGRenderer.h
    )

    # 为头文件生成 Qt MOC 源文件（处理 Q_OBJECT 宏）
    qt6_wrap_cpp(SOURCES_H_MOC ${SOURCE_H})

    # 所有需要编译的源文件
    SET(TARGET_SRC
        CullVisitorEx.cpp
        GraphicsWindowEx.cpp
        osgQOpenGLWidget.cpp
        osgQOpenGLWindow.cpp
        OSGRenderer.cpp
        RenderStageEx.cpp
        StateEx.cpp
        ${SOURCES_H_MOC}
    )

    # ============================================================================
    # 创建静态库
    # ============================================================================
    add_library(${LIB_NAME} STATIC ${TARGET_SRC})

    # ============================================================================
    # 设置包含目录
    # 包括当前源码目录、构建目录（用于 MOC 生成的头文件）、OSG 头文件路径
    # ============================================================================
    target_include_directories(${LIB_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${OPENSCENEGRAPH_INCLUDE_DIRS}
    )

    # ============================================================================
    # 链接依赖库
    # ============================================================================
    target_link_libraries(${LIB_NAME} PUBLIC
        Qt6::Widgets          # Qt 核心部件库（QWidget、布局等）
        Qt6::OpenGL           # Qt OpenGL 支持
        Qt6::OpenGLWidgets    # Qt OpenGL 部件（QOpenGLWidget）
        ${OPENSCENEGRAPH_LIBRARIES}  # OSG 库（osgViewer、osgUtil、osg 等）
    )

    # ============================================================================
    # 生成导出头文件
    # ============================================================================
    generate_export_header(${LIB_NAME} EXPORT_FILE_NAME "Export")

    # ============================================================================
    # 注意：osgQOpenGL 库通过主 CMakeLists.txt 链接到主可执行文件 appInteractiveSLAM
    # 此处仅构建静态库，不创建可执行文件
    # ============================================================================

ENDIF()
