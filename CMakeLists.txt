# ============================================================================
# CMakeLists.txt
# InteractiveSLAM 项目构建配置文件
#
# 项目概述：交互式 SLAM（同步定位与地图构建）系统。
#   使用 g2o 进行图优化，PCL 处理点云，OpenSceneGraph 进行 3D 可视化，
#   Qt6 提供图形用户界面。
#
# 依赖项：
#   - Eigen3: 线性代数库
#   - PCL:    点云库（组件：io, common, search, features, registration）
#   - g2o:    图优化库
#   - Boost:  文件系统库
#   - OpenMP: 并行计算（可选，用于多线程 GICP/NDT 配准）
#   - OpenSceneGraph: 3D 渲染引擎
#   - Qt6:    用户界面框架（组件：Widgets, OpenGL, OpenGLWidgets, Concurrent）
#
# 构建目标：
#   - appInteractiveSLAM: 交互式 SLAM 主应用程序
#   - test_data_loading: 数据加载测试程序
# ============================================================================

# ============================================================================
# 项目基础配置
# ============================================================================

cmake_minimum_required(VERSION 3.16)

project(InteractiveSLAM VERSION 0.1 LANGUAGES CXX)

# 使用 C++17 标准（必需）
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用 Qt 自动处理：MOC（元对象编译）、UIC（界面编译）、RCC（资源编译）
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

# ============================================================================
# 第三方依赖查找
# ============================================================================

# 自定义 find 模块路径
list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# —— 必需依赖 ——
find_package(Eigen3 REQUIRED)                    # 线性代数
find_package(PCL REQUIRED COMPONENTS io common search features registration)  # 点云库
find_package(g2o REQUIRED)                       # 图优化
find_package(Boost REQUIRED COMPONENTS filesystem)  # 文件系统
find_package(yaml-cpp REQUIRED)                  # YAML 配置解析（bag 导入配置）

# —— OpenMP（可选，用于 ndt_omp 多线程 GICP/NDT） ——
find_package(OpenMP)
if(OpenMP_CXX_FOUND)
    set(HAS_PCLOMP ON)
    message(STATUS "OpenMP found — enabling GICP_OMP / NDT_OMP")
else()
    message(WARNING "OpenMP not found — GICP_OMP / NDT_OMP will be unavailable")
endif()

# —— OpenSceneGraph（3D 渲染） ——
find_package(OpenSceneGraph REQUIRED
    COMPONENTS osg osgViewer osgGA osgUtil osgDB OpenThreads)

# —— Qt6（基于 Widgets 的 UI，替代旧版 QML） ——
find_package(Qt6 REQUIRED COMPONENTS Widgets OpenGL OpenGLWidgets Concurrent LinguistTools)

# ============================================================================
# 源文件分类
# ============================================================================

# --- 数据层（核心 SLAM 数据结构与算法） ---
set(DATA_SOURCES
    src/data/hdl_graph_slam/keyframe.cpp                  # 关键帧数据结构
    src/data/hdl_graph_slam/graph_slam.cpp                # 图 SLAM 核心算法
    src/data/hdl_graph_slam/information_matrix_calculator.cpp  # 信息矩阵计算
    src/data/hdl_graph_slam/interactive_keyframe.cpp      # 交互式关键帧
    src/data/hdl_graph_slam/interactive_graph.cpp         # 交互式图数据
    src/data/hdl_graph_slam/registration_methods.cpp      # 点云配准方法
    src/data/hdl_graph_slam/automatic_loop_closure.cpp    # 自动回环检测
    src/data/hdl_graph_slam/bag_importer.cpp              # ROS1 bag 导入器（打开 Bag 路径）
    src/data/rosbag/rosbag.cpp                            # ROS1 bag 文件读取（精简版）
    src/data/g2o/robust_kernel_io.cpp                     # 鲁棒核函数 IO
)

# --- Qt 后端桥接层 ---
set(BACKEND_SOURCES
    src/backend/progress_reporter.cpp                     # 线程安全进度报告
    src/backend/graph_manager.cpp                         # 图管理器
)

# --- Qt Widgets UI（新的基于 Widgets 的界面，替代 QML） ---
set(UI_SOURCES
    src/ui/MainWindow.cpp                                 # 主窗口
    src/ui/ProjectCenterDialog.cpp                        # 项目中心对话框（新建/打开项目）
    src/ui/BagOpenDialog.cpp                              # 打开 Bag 对话框（topic 选择）
    src/ui/AutoLoopClosureDialog.cpp                       # 高级设置：自动回环对话框
    src/ui/RenderingAdvancedDialogs.cpp                    # 高级设置：渲染相关对话框（LOD/Z裁剪/颜色范围）
    src/ui/SaveMapDialog.cpp                               # 保存地图配置对话框（位姿图/每帧点云/全局点云）
    src/ui/ViewportWidget.cpp                             # 3D 视口控件
    src/ui/GraphStatsPanel.cpp                            # 图统计面板
    src/ui/RenderingPanel.cpp                             # 渲染设置面板
    src/ui/MiniViewportWidget.cpp                         # 缩略图视口
    src/ui/LoopClosureDialog.cpp                          # 回环闭合对话框
    src/ui/AutoLoopClosurePanel.cpp                       # 自动回环面板（精简）
    src/ui/EdgeListPanel.cpp                              # 边列表面板
    src/ui/PlaybackPanel.cpp                              # 播放轴面板
    src/ui/OverlayPanelWidget.cpp                         # 叠加面板控件
    src/ui/LoadingOverlayWidget.cpp                       # 加载遮罩进度覆盖层
)

# --- 可视化层源文件 ---
# 说明：src/visualizers/ 下绝大多数可视化器是纯头文件（模板式内联实现），
#       不参与编译单元列表；此处只登记确实需要独立编译单元的少数文件。
set(VISUALIZER_SOURCES
    src/visualizers/CloudPerfLog.cpp                      # 性能日志 + NVML 显存查询（基线测量用）
)

# --- 点云帧存储（.isf）源文件 ---
# 说明：Phase 1 产物。目前只被 tools/isf_pack 使用；Phase 2 接入渲染路径时
#       会再加进主程序目标。刻意不提前塞进 appInteractiveSLAM，避免影响
#       用户正在跑的 Phase 0 基线构建。
set(ISF_SOURCES
    src/pointcloud/isf/IsfWriter.cpp                      # .isf/.isf.idx 打包器
    src/pointcloud/isf/IsfReader.cpp                      # .isf/.isf.idx 读取器（mmap + 顺序读兜底）
)

# --- 位姿表 + 渲染分块（Phase 2 数据侧） ---
# 无 OSG / 无 Qt 依赖，因此既能进主程序，也能被无头自检工具链接。
set(RENDER_DATA_SOURCES
    src/visualizers/CloudRenderData.cpp                   # CloudPoseTable + buildChunks + 世界 AABB
)

# --- 点云渲染几何体与位姿纹理（Phase 2b，OSG 侧原语） ---
# 只依赖 OSG 核心（不需要 GL 上下文即可构造），随主程序一起编译以便尽早暴露
# 编译错误；但**尚未接入渲染路径**（接线属 Phase 2b 后半，且需与 Phase 4 的
# CloudRayIntersector 同时落地，理由见 CloudGeometry.h 的接入前置条件）。
set(CLOUD_GL_SOURCES
    src/visualizers/CloudGeometry.cpp                     # osg::Geometry 子类：局部顶点 + 动态世界 AABB + 屏蔽原生相交器
    src/visualizers/CloudPoseBuffer.cpp                   # 位姿表 → TextureBuffer（双槽，只更新优化槽）
)

# ============================================================================
# 第三方子项目
# ============================================================================

# ndt_omp_MSVC —— 多线程 GICP/NDT（针对 PCL 1.12+/MSVC 修复的分支）
if(HAS_PCLOMP)
    add_subdirectory(${CMAKE_SOURCE_DIR}/thirdparty/ndt_omp_MSVC)
endif()

# osgQOpenGL —— OSG 与 Qt6 Widgets 的适配库
add_subdirectory(src/osgQOpenGL)
if(MSVC)
    add_compile_options(/utf-8)  # MSVC 下启用 UTF-8 源代码编码
endif()

# ============================================================================
# 主应用程序目标
# ============================================================================

# 创建可执行文件
qt_add_executable(appInteractiveSLAM
    main.cpp                  # 应用程序入口
    ${DATA_SOURCES}           # 数据层源文件
    ${BACKEND_SOURCES}        # 后端桥接层源文件
    ${UI_SOURCES}             # UI 层源文件
    ${VISUALIZER_SOURCES}     # 可视化层源文件（需要独立编译单元的）
    ${CLOUD_GL_SOURCES}       # Phase 2b：点云几何体与位姿纹理（OSG 侧原语）
    src/ui/ui_resources.qrc   # Qt 资源文件（暗色主题 QSS）
)

# Windows exe 图标（.rc 资源脚本引用 icon/app.ico）
if(WIN32)
    target_sources(appInteractiveSLAM PRIVATE app.rc)
endif()

# --- Qt 中文翻译 ---
qt_add_translations(appInteractiveSLAM
    TS_FILES
        translations/app_zh_CN.ts   # 中文翻译文件
    RESOURCE_PREFIX /i18n            # 资源路径前缀
)

# 包含目录
#
# 注意 OSG 的头必须靠 OPENSCENEGRAPH_INCLUDE_DIRS 显式带上：
# OPENSCENEGRAPH_LIBRARIES 是**库文件路径列表**（形如
#   optimized;E:/vcpkg/installed/x64-windows/lib/osg.lib;debug;.../osgd.lib
# ），不是 imported target，因此**不会传递任何头文件路径**。
# 此前之所以能编译 osg/*，只是因为 PCL_INCLUDE_DIRS 恰好包含 vcpkg 的 include
# 根目录（E:/vcpkg/installed/x64-windows/include）——那是巧合；任何不带 PCL 的
# 目标（例如 cloud_shader_selftest）都会立刻报 "无法打开包括文件 osg/Array"。
target_include_directories(appInteractiveSLAM PRIVATE
    ${CMAKE_SOURCE_DIR}/src          # 项目源代码根目录
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}   # OSG 头文件（理由见上）
)

# 链接库
target_link_libraries(appInteractiveSLAM PRIVATE
    Qt6::Widgets              # Qt 控件模块
    Qt6::OpenGL               # Qt OpenGL 模块
    Qt6::OpenGLWidgets        # Qt OpenGL 控件模块
    Qt6::Concurrent           # Qt 并行计算模块（用于异步加载）
    osgQOpenGL                # OSG-Qt 适配库
    ${PCL_LIBRARIES}          # 点云库
    ${G2O_LIBRARIES}          # 图优化库
    Boost::filesystem         # 文件系统库
    yaml-cpp                  # YAML 配置解析（bag 导入配置）
    ${OPENSCENEGRAPH_LIBRARIES}  # OSG 库
    ${OpenMP_CXX_LIBRARIES}   # OpenMP 库
)

# ndt_omp 链接（可选）
if(HAS_PCLOMP)
    target_link_libraries(appInteractiveSLAM PRIVATE ndt_omp)
endif()

# 启用 HAS_PCLOMP 宏定义，使 registration_methods.cpp 可以使用 GICP_OMP / NDT_OMP
if(HAS_PCLOMP AND OpenMP_CXX_FOUND)
    target_compile_definitions(appInteractiveSLAM PRIVATE HAS_PCLOMP)
    target_compile_options(appInteractiveSLAM PRIVATE ${OpenMP_CXX_FLAGS})
endif()

# MSVC 编译器选项：警告等级 3，禁用 4996（安全的 STL 警告）
if(MSVC)
    target_compile_options(appInteractiveSLAM PRIVATE /W3 /wd4996)
endif()

# ============================================================================
# 命令行测试程序
# ============================================================================

# 数据加载测试（独立 CLI 程序，无需 Qt 和 OSG 依赖）
add_executable(test_data_loading
    tests/test_data_loading.cpp   # 测试文件
    ${DATA_SOURCES}               # 数据层源文件
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
    yaml-cpp
)
if(MSVC)
    target_compile_options(test_data_loading PRIVATE /W3 /wd4996)
endif()

# ============================================================================
# .isf 打包/校验命令行工具（Phase 1 验收入口）
# ============================================================================

# 链接方式照抄 test_data_loading：不定义 HAS_PCLOMP，因此不必依赖 ndt_omp
add_executable(isf_pack
    tools/isf_pack.cpp            # 打包 + 结构校验 + 与源点云逐位往返比对
    ${DATA_SOURCES}               # 数据层源文件（含 InteractiveGraph 加载）
    ${ISF_SOURCES}                # .isf 读写器
)
target_include_directories(isf_pack PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(isf_pack PRIVATE
    ${PCL_LIBRARIES}
    ${G2O_LIBRARIES}
    Boost::filesystem
    yaml-cpp
)
if(MSVC)
    target_compile_options(isf_pack PRIVATE /W3 /wd4996)
endif()

# ============================================================================
# Phase 2 数据侧无头自检（不需要 GPU / Qt / GUI）
# ============================================================================

# 验证"位姿上移着色器"架构的可测量论断：位姿表正确性、优化更新只重写位姿槽
# （与点数无关）、分块帧号映射、LOD 索引合法性、位姿变化后世界 AABB 仍正确、
# 双层共享 = 1× 顶点显存。
add_executable(cloud_selftest
    tools/cloud_selftest.cpp      # 自检主体
    ${DATA_SOURCES}               # 地图加载
    ${RENDER_DATA_SOURCES}        # 位姿表 + 渲染分块
)
target_include_directories(cloud_selftest PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
)
target_link_libraries(cloud_selftest PRIVATE
    ${PCL_LIBRARIES}
    ${G2O_LIBRARIES}
    Boost::filesystem
    yaml-cpp
)
if(MSVC)
    target_compile_options(cloud_selftest PRIVATE /W3 /wd4996)
endif()

# ============================================================================
# Phase 4 拾取无头自检（不需要 GPU / Qt / 窗口）
# ============================================================================

# 验证 pose-aware 拾取：射线是世界系、顶点是帧局部系，这条变换链最容易
# "看起来对、实际错"，所以用合成数据做精确数值断言 + 真实地图往返验证。
# 需要 OSG（osgUtil 的 IntersectionVisitor / Intersector 体系），但不需要 GL 上下文。
add_executable(cloud_pick_selftest
    tools/cloud_pick_selftest.cpp # 自检主体（合成数据 + 真实地图往返）
    ${DATA_SOURCES}               # 地图加载
    ${RENDER_DATA_SOURCES}        # 位姿表 + 渲染分块
)
target_include_directories(cloud_pick_selftest PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}   # OSG 头（OPENSCENEGRAPH_LIBRARIES 不带头路径）
)
target_link_libraries(cloud_pick_selftest PRIVATE
    ${PCL_LIBRARIES}
    ${G2O_LIBRARIES}
    Boost::filesystem
    yaml-cpp
    ${OPENSCENEGRAPH_LIBRARIES}   # osgUtil（相交器框架）；同时带入 OSG 头文件路径
)
if(MSVC)
    target_compile_options(cloud_pick_selftest PRIVATE /W3 /wd4996)
endif()

# ============================================================================
# Phase 2b GL 侧无头自检（不需要 GPU / GL 上下文）
# ============================================================================

# GLSL 里 uniform 名字打错是**静默失败**：着色器照常编译，OSG 找不到 location
# 就什么都不做，最后只是画面悄悄不对。这里在 CPU 侧把"着色器声明了什么"与
# "C++ 创建了什么"做双向交叉比对，另外核对 Turbo 色表的 CPU/GPU 逐位一致
# （含取色索引的截断语义）与每 chunk uFrameIdBias 的独立性。
# 不依赖数据层与地图文件，只需 OSG 核心（构造 Program/StateSet/TextureBuffer
# 都不需要 GL 上下文）。
add_executable(cloud_shader_selftest
    tools/cloud_shader_selftest.cpp
)
target_include_directories(cloud_shader_selftest PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    # 本目标刻意不依赖 PCL/数据层，因此**必须**显式带上 OSG 头路径
    # （不能像其它目标那样指望 PCL_INCLUDE_DIRS 顺带把 vcpkg include 根带进来）
    ${OPENSCENEGRAPH_INCLUDE_DIRS}
)
target_link_libraries(cloud_shader_selftest PRIVATE
    ${OPENSCENEGRAPH_LIBRARIES}
)
if(MSVC)
    target_compile_options(cloud_shader_selftest PRIVATE /W3 /wd4996)
endif()

# ============================================================================
# Phase 2b/3 装配层无头自检（不需要 GPU / GL 上下文 / 地图文件）
# ============================================================================

# 用合成 chunk 验证装配层的结构性不变量，秒级且完全确定：
# 两层是否共享同一批 Geometry（"双层 = 1× 显存"的全部来源）、每 chunk 的
# uFrameIdBias 是否正确且不共享、层级的量是否挂在父 Group 靠继承传播、
# LOD 切换是否只换 primitive set、世界 AABB 是否逐 chunk 落地。
add_executable(cloud_scene_selftest
    tools/cloud_scene_selftest.cpp   # 自检主体（合成 chunk）
    ${CLOUD_GL_SOURCES}              # CloudGeometry + CloudPoseBuffer（需独立编译单元）
    ${RENDER_DATA_SOURCES}           # 分块数据结构
    ${DATA_SOURCES}                  # 地图加载（本自检不加载地图，但保证符号齐全）
)
target_include_directories(cloud_scene_selftest PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${EIGEN3_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OPENSCENEGRAPH_INCLUDE_DIRS}   # OSG 头（OPENSCENEGRAPH_LIBRARIES 不带头路径）
)
target_link_libraries(cloud_scene_selftest PRIVATE
    ${PCL_LIBRARIES}
    ${G2O_LIBRARIES}
    Boost::filesystem
    yaml-cpp
    ${OPENSCENEGRAPH_LIBRARIES}
)
if(MSVC)
    target_compile_options(cloud_scene_selftest PRIVATE /W3 /wd4996)
endif()

# ============================================================================
# 安装规则
# ============================================================================

install(TARGETS appInteractiveSLAM
    BUNDLE DESTINATION .                               # macOS bundle
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}        # 共享库
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}        # 可执行文件
)
