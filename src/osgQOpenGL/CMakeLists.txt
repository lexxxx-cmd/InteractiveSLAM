IF ( Qt6Widgets_FOUND )
    include(GenerateExportHeader)

    SET(LIB_NAME osgQOpenGL)

    set(CMAKE_INCLUDE_CURRENT_DIR yes)

    # 使用当前目录下的头文件，而不是不存在的include目录
    SET(SOURCE_H 
        osgQOpenGLWidget.h
        osgQOpenGLWindow.h
        OSGRenderer.h
    )

    qt6_wrap_cpp(SOURCES_H_MOC ${SOURCE_H})

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

    # 创建静态库
    add_library(${LIB_NAME} STATIC ${TARGET_SRC})

    # 设置包含目录
    target_include_directories(${LIB_NAME} PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}
        ${OPENSCENEGRAPH_INCLUDE_DIRS}
    )

    # 链接Qt6库
    target_link_libraries(${LIB_NAME} PUBLIC
        Qt6::Widgets
        Qt6::OpenGL
        Qt6::OpenGLWidgets
        ${OPENSCENEGRAPH_LIBRARIES}
    )

    # 生成Export头文件
    generate_export_header(${LIB_NAME} EXPORT_FILE_NAME "Export")

    # 将库链接到主可执行文件
    target_link_libraries(3DPCViewer PRIVATE ${LIB_NAME})

ENDIF()
