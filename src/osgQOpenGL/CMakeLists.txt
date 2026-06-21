# osgQOpenGL — minimal OSG-to-Qt adapter.
#
# Originally this folder shipped a QtWidgets QOpenGLWidget (osgQOpenGLWidget) and
# a QOpenGLWindow variant. The app embeds rendering into QML via
# QQuickFramebufferObject instead, so only the widget-agnostic OSGRenderer is
# compiled here. It is consumed by src/viewport/osg_viewport_renderer.cpp.

set(LIB_NAME osgQOpenGL)
set(CMAKE_INCLUDE_CURRENT_DIR yes)

# OSGRenderer.h uses Q_OBJECT (timerEvent is a slot-able override) → run moc.
set(SOURCE_H OSGRenderer.h)
qt6_wrap_cpp(SOURCES_H_MOC ${SOURCE_H})

set(TARGET_SRC
    OSGRenderer.cpp
    ${SOURCES_H_MOC}
)

add_library(${LIB_NAME} STATIC ${TARGET_SRC})

target_include_directories(${LIB_NAME} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
)

# OSGRenderer needs QObject (Core) + the GUI event classes (QKeyEvent etc. live
# in Qt6::Gui). No Widgets.
target_link_libraries(${LIB_NAME} PUBLIC
    Qt6::Core
    Qt6::Gui
    ${OPENSCENEGRAPH_LIBRARIES}
)
