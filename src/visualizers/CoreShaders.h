// ============================================================================
// CoreShaders.h
// 核心着色器程序集合
//
// 本文件定义了一组 GLSL 330 Core Profile 着色器程序，用于替代 OSG 内建的
// 固定功能管线。核心原则：通过为每个可绘制对象设置显式的着色器程序，
// OSG 就不会尝试设置已废弃的内建 uniform 变量（如 Material、LightModel、
// AlphaFunc），从而消除 Core Profile 上下文中的 GL 错误。
//
// 包含的着色器：
//   1. createSimpleColorProgram() —— 纯色着色器（用于线段、坐标轴、球体）
//   2. createPointCloudProgram() —— 点云着色器（支持可调点大小和 Z 轴裁剪）
// ============================================================================

#pragma once

#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>
#include <osg/StateSet>

// ============================================================================
// 简易纯色着色器 —— 用于线段、坐标轴、球体等简单几何体
// ============================================================================

/**
 * @brief 创建简易纯色着色器程序（单例）
 *
 * 顶点着色器：将顶点位置通过模型-视图-投影矩阵变换，直接传递顶点颜色。
 * 片段着色器：直接输出接收到的颜色值。
 *
 * 此着色器没有光照计算，也没有材质属性，仅输出逐顶点颜色。
 * 适用于坐标轴、边线、球体等不需要复杂光照效果的几何体。
 *
 * @return 静态单例 osg::Program 指针
 */
inline osg::Program* createSimpleColorProgram() {
    static osg::ref_ptr<osg::Program> s_prog;
    if (s_prog.valid()) return s_prog.get();

    // 顶点着色器：标准 MVP 变换，传递颜色
    const char* vert = R"(
        #version 330 core
        in vec4 osg_Vertex;          // OSG 标准顶点属性
        in vec4 osg_Color;           // OSG 标准颜色属性
        uniform mat4 osg_ModelViewProjectionMatrix;  // MVP 矩阵
        out vec4 vColor;             // 传递到片段着色器的颜色
        void main() {
            gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;
            vColor = osg_Color;
        }
    )";
    // 片段着色器：直接输出颜色
    const char* frag = R"(
        #version 330 core
        in vec4 vColor;              // 从顶点着色器接收的颜色
        out vec4 fragColor;          // 最终像素颜色
        void main() {
            fragColor = vColor;
        }
    )";
    s_prog = new osg::Program;
    s_prog->addShader(new osg::Shader(osg::Shader::VERTEX, vert));
    s_prog->addShader(new osg::Shader(osg::Shader::FRAGMENT, frag));
    return s_prog.get();
}

// ============================================================================
// 点云着色器 —— 支持逐顶点着色、可调点大小、Z 轴范围裁剪
// ============================================================================

/**
 * @brief 创建点云着色器程序（单例）
 *
 * 顶点着色器：MVP 变换 + 通过 uniform 控制点大小 + 将世界坐标传递给片段着色器。
 * 片段着色器：支持 Z 轴范围裁剪，可通过 uniform 动态开关。
 *
 * 额外功能：
 *   - uPointSize: 控制点的大小（像素单位）
 *   - z_clipping: Z 轴裁剪开关（0 关闭，非 0 开启）
 *   - z_range: Z 轴裁剪范围 [min, max]
 *
 * @return 静态单例 osg::Program 指针
 */
inline osg::Program* createPointCloudProgram() {
    static osg::ref_ptr<osg::Program> s_prog;
    if (s_prog.valid()) return s_prog.get();

    // 顶点着色器：MVP 变换 + 点大小控制 + 传递世界坐标
    const char* vert = R"(
        #version 330 core
        in vec4 osg_Vertex;
        in vec4 osg_Color;
        uniform mat4 osg_ModelViewProjectionMatrix;
        uniform float uPointSize;     // 点的大小（像素）
        out vec4 vColor;
        out vec3 vWorldPos;           // 世界坐标，用于片段着色器的裁剪判断
        void main() {
            gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;
            gl_PointSize = uPointSize;
            vColor = osg_Color;
            vWorldPos = osg_Vertex.xyz;
        }
    )";
    // 片段着色器：颜色输出 + Z 轴裁剪
    const char* frag = R"(
        #version 330 core
        in vec4 vColor;
        in vec3 vWorldPos;            // 片元的世界坐标（用于 Z 轴裁剪）
        uniform int z_clipping;       // Z 轴裁剪开关
        uniform vec2 z_range;         // Z 轴裁剪范围 [最小值, 最大值]
        out vec4 fragColor;
        void main() {
            // 如果 Z 轴裁剪开启，且片元 Z 坐标超出范围，则丢弃该片元
            if (z_clipping != 0 && (vWorldPos.z < z_range[0] || vWorldPos.z > z_range[1]))
                discard;
            fragColor = vColor;
        }
    )";
    s_prog = new osg::Program;
    s_prog->addShader(new osg::Shader(osg::Shader::VERTEX, vert));
    s_prog->addShader(new osg::Shader(osg::Shader::FRAGMENT, frag));
    return s_prog.get();
}

/**
 * @brief 将简易纯色着色器应用到指定的 StateSet 上
 * @param ss 目标状态集
 */
inline void applySimpleColorShader(osg::StateSet* ss) {
    ss->setAttributeAndModes(createSimpleColorProgram(),
                             osg::StateAttribute::ON);
}

/**
 * @brief 将点云着色器应用到指定的 StateSet 上，并创建点大小 uniform
 *
 * 此函数会：
 *   1. 设置点云着色器程序
 *   2. 启用 GL_PROGRAM_POINT_SIZE 模式（使顶点着色器中的 gl_PointSize 生效）
 *   3. 创建并添加 uPointSize、z_clipping、z_range 三个 uniform
 *
 * @param ss        目标状态集
 * @param pointSize 初始点大小（像素单位，默认 3.0）
 * @return uPointSize uniform 指针，调用方可在运行时更新其值
 */
inline osg::Uniform* applyPointCloudShader(osg::StateSet* ss, float pointSize = 3.0f) {
    ss->setAttributeAndModes(createPointCloudProgram(),
                             osg::StateAttribute::ON);
    // OpenGL Core Profile 中必须启用此模式，顶点着色器的 gl_PointSize
    // 才能生效；否则着色器中设置的点大小会被静默忽略。
    ss->setMode(GL_PROGRAM_POINT_SIZE, osg::StateAttribute::ON);
    // 创建点大小 uniform，调用方可以保存此指针并在运行时更新点大小
    auto* u = new osg::Uniform("uPointSize", pointSize);
    ss->addUniform(u);
    // Z 轴裁剪 uniform（默认关闭，裁剪范围在数据加载时设置）
    ss->addUniform(new osg::Uniform("z_clipping", 0));
    ss->addUniform(new osg::Uniform("z_range", osg::Vec2(0.0f, 1.0f)));
    return u;
}
