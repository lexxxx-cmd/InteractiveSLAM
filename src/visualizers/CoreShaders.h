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

#include <osg/Array>
#include <osg/GL>
#include <osg/Program>
#include <osg/Shader>
#include <osg/StateSet>
#include <osg/Texture>
#include <osg/TextureBuffer>
#include <osg/Uniform>

#include "visualizers/TurboColormap.h"   // turboTable()：CPU 与 GPU 共用同一份色表

// ============================================================================
// 共用的 GL 常量
// ============================================================================

// —— 内部格式常量：为什么不用 GL_RGBA32F 这个名字 ——
// 本 OSG 构建里 `OSG_GL3_AVAILABLE` 是 undef（vcpkg 的 osg/GL:23），因此
// `osg/GL` 回落到 Windows `<GL/gl.h>`（只到 GL 1.1）。OSG 自带的 GLDefines
// 虽然把枚举补到了 GL 3.0（`GL_TEXTURE_BUFFER` 0x8C2A、`GL_PROGRAM_POINT_SIZE`
// 0x8642 都在其中，所以 TextureBuffer 与点云着色器能编译），但**没有
// GL_RGBA32F**；而 `<GL/glext.h>` / `<GL/glew.h>` 并未被包含。
//
// 为了一个枚举去 include `<GL/glext.h>`（数千行，且依赖 APIENTRY 等 Win32 宏）
// 不划算，所以按 OpenGL 3.0 规范给出常量。取值已在本机 vcpkg 的三个独立
// 头文件中核对一致：
//     GL/glcorearb.h:1000    GL/glew.h:2087    GL/glext.h:882   →  0x8814
//
// 位姿纹理必须是**浮点**内部格式：位姿矩阵含平移分量（地图尺度可达 1e3~1e4），
// 若退化成 8 位归一化格式，位姿会被彻底毁掉（症状：点云挤成一团）。
// Turbo LUT 也用它：色表是 [0,1] 浮点，8 位量化会让 CPU/GPU 配色出现可见差。
constexpr GLint kInternalFormatRGBA32F = 0x8814;  // GL_RGBA32F

/** @brief 位姿表纹理占用的纹理单元（samplerBuffer） */
constexpr int kPoseTexUnit = 0;
/** @brief Turbo 色表纹理占用的纹理单元（samplerBuffer） */
constexpr int kLutTexUnit  = 1;

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

// ============================================================================
// 位姿点云着色器 —— "位姿上移着色器"架构的 GL 侧（Phase 2b）
// ============================================================================
//
// 与上面的 createPointCloudProgram() 的根本区别：那个 shader 把 osg_Vertex 当作
// **世界坐标**（`vWorldPos = osg_Vertex.xyz`），因为旧路径的顶点确实是世界坐标；
// 新架构的顶点是**帧局部系**坐标，世界坐标必须由顶点着色器用位姿表算出来，
// 因此：
//   · 顶点着色器多一个整型属性 aFrameId（帧在 chunk 内的序号）与一个
//     uFrameIdBias（该 chunk 的起始全局帧序），据此从 uPoseTable 取 4 个 texel
//     组成 mat4，把局部点变换到世界；
//   · **vWorldPos 必须是着色器算出的世界坐标**——Z 轴裁剪与 Turbo 配色都读它，
//     若仍用 osg_Vertex 会按"帧局部 Z"上色，逐帧看都是错的（每帧都被归一化到
//     自己的局部高程上）。
//   · 颜色不再来自逐顶点属性，而是按世界 Z 查 uTurboLut（256×RGBA32F），
//     取色公式与 CPU turboColor() 逐位一致（见片段着色器注释）。
//
// uniform 契约（哪些是全局、哪些是每 chunk / 每层，决定了 StateSet 怎么组织）：
//   | 名称             | 类型          | 作用域   | 由谁设置                     |
//   |------------------|---------------|----------|------------------------------|
//   | uPoseTable       | samplerBuffer | **每层** | bindPoseTableTexture()        |
//   | uTurboLut        | samplerBuffer | 全局     | applyPosePointCloudShader()   |
//   | uFrameIdBias     | int           | **每chunk** | setPoseCloudFrameBias()    |
//   | uPointSize       | float         | 全局     | applyPosePointCloudShader()   |
//   | uZRange          | vec2          | 全局     | 同上（按构建时颜色范围更新）   |
//   | uOpacity         | float         | 全局     | 同上（改透明度只写这一个值）   |
//   | uColorMode       | int           | **每层** | 同上（0=Turbo 0..1；1=常量色） |
//   | uConstantColor   | vec4          | **每层** | 同上（原始层淡橙半透明）       |
//   | z_clipping       | int           | 全局     | 同上（沿用旧 shader 同名语义） |
//   | z_range          | vec2          | 全局     | 同上                          |
//
// 前置条件：点云 geode **不得**挂在非恒等变换下。着色器算出的 world 已经在地图
// 世界系里，而 osg_ModelViewProjectionMatrix 含模型矩阵；只有模型矩阵恒等时
// 两者才不冲突。（CloudGeometry::computeBoundingBox 返回世界 AABB、
// CloudRayIntersector 用世界射线，也基于同一前提。）

/**
 * @brief 创建 Turbo 色表纹理（256×RGBA32F，单例，全局共享）
 *
 * 数据直接取自 `turboTable()` —— 与 CPU 侧 `turboColor()` 是**同一份** 256 项色表，
 * 因此 GPU 采样结果与旧路径的 CPU 逐顶点着色逐位一致，切架构不会出现配色漂移。
 *
 * 用 NEAREST 过滤：色表是离散查表，线性插值会在色带边界产生 CPU 侧没有的颜色。
 */
inline osg::TextureBuffer* createTurboLutTexture() {
    static osg::ref_ptr<osg::TextureBuffer> s_tex;
    if (s_tex.valid()) return s_tex.get();

    const float (*tbl)[3] = turboTable();
    osg::ref_ptr<osg::Vec4Array> data = new osg::Vec4Array(256);
    for (size_t i = 0; i < 256; ++i) {
        // alpha 固定 1：透明度由 uOpacity 统一乘上去，色表本身不含透明度
        (*data)[i].set(tbl[i][0], tbl[i][1], tbl[i][2], 1.0f);
    }

    auto* tex = new osg::TextureBuffer;
    tex->setInternalFormat(kInternalFormatRGBA32F);
    tex->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
    tex->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
    tex->setTextureWidth(256);          // GL_TEXTURE_BUFFER 下即 texel 数
    tex->setBufferData(data.get());
    s_tex = tex;
    return s_tex.get();
}

/**
 * @brief 创建位姿点云着色器程序（单例）
 *
 * 顶点着色器：aFrameId + uFrameIdBias → texelFetch(uPoseTable) → mat4 → 世界坐标
 * 片段着色器：Z 轴裁剪 →（Turbo 查表 | 常量色）× uOpacity
 */
inline osg::Program* createPosePointCloudProgram() {
    static osg::ref_ptr<osg::Program> s_prog;
    if (s_prog.valid()) return s_prog.get();

    const char* vert = R"(
        #version 330 core

        // 帧号：由 CloudGeometry 以属性 location 13 提供（chunk 内的帧序号）。
        // 必须用 layout 显式钉住位置：aFrameId 不在 OSG 的默认属性别名表里，
        // 不声明则绑定不确定（症状：帧号读成 0，所有点堆到同一帧的位姿上）。
        layout(location = 13) in float aFrameId;
        in vec4 osg_Vertex;                    // **帧局部系**坐标（不是世界坐标）

        uniform mat4 osg_ModelViewProjectionMatrix;
        uniform samplerBuffer uPoseTable;      // 每帧 4 个 RGBA32F texel = 4×4 列主序
        uniform int   uFrameIdBias;            // 该 chunk 的起始全局帧序
        uniform float uPointSize;

        out vec3 vWorldPos;

        void main() {
            // 属性是 float，先加 0.5 再截断，避免将来帧号大到 float 无法精确
            // 表示整数时出现 off-by-one（当前帧数规模下本来就精确）
            int frame = uFrameIdBias + int(aFrameId + 0.5);
            int base  = frame * 4;
            // 4 个 texel 就是 4 列（CloudPoseTable 的数据是数学列主序，每 4 个
            // float 一列），所以 mat4(列0,列1,列2,列3) 直接就是该帧的位姿
            mat4 T = mat4(texelFetch(uPoseTable, base + 0),
                          texelFetch(uPoseTable, base + 1),
                          texelFetch(uPoseTable, base + 2),
                          texelFetch(uPoseTable, base + 3));
            vec4 world = T * vec4(osg_Vertex.xyz, 1.0);
            gl_Position = osg_ModelViewProjectionMatrix * world;
            gl_PointSize = uPointSize;
            vWorldPos = world.xyz;
        }
    )";

    const char* frag = R"(
        #version 330 core

        in vec3 vWorldPos;                     // 世界坐标（由顶点着色器算出）
        out vec4 fragColor;

        uniform samplerBuffer uTurboLut;       // 256 × RGBA32F Turbo 色表
        uniform vec2  uZRange;                 // 颜色映射范围 [zMin, zMax]（世界 Z）
        uniform float uOpacity;                // 全局透明度
        uniform int   uColorMode;              // 0 = 按世界 Z 查 Turbo；1 = 常量色
        uniform vec4  uConstantColor;          // uColorMode==1 时使用
        uniform int   z_clipping;              // Z 轴裁剪（沿用旧 shader 同名语义）
        uniform vec2  z_range;

        void main() {
            if (z_clipping != 0 && (vWorldPos.z < z_range[0] || vWorldPos.z > z_range[1]))
                discard;

            vec4 c;
            if (uColorMode == 1) {
                c = uConstantColor;
            } else {
                float range = uZRange.y - uZRange.x;
                if (range <= 0.0) {            // 与 CPU turboColor() 的分支一致
                    c = vec4(0.5, 0.5, 0.5, 1.0);
                } else {
                    // 必须与 CPU 逐位一致：CPU 是 idx = clamp(int(t*255), 0, 255)
                    // —— int() 是**截断**不是四舍五入，写成 int(t*255.0 + 0.5)
                    // 会让整条色带偏移一档（112 项里约一半会不同）
                    float t = (vWorldPos.z - uZRange.x) / range;
                    int idx = clamp(int(t * 255.0), 0, 255);
                    c = texelFetch(uTurboLut, idx);
                }
            }
            fragColor = vec4(c.rgb, c.a * uOpacity);
        }
    )";

    s_prog = new osg::Program;
    s_prog->addShader(new osg::Shader(osg::Shader::VERTEX, vert));
    s_prog->addShader(new osg::Shader(osg::Shader::FRAGMENT, frag));
    return s_prog.get();
}

/**
 * @brief 由 applyPosePointCloudShader() 创建、调用方需要运行时更新的 uniform 句柄
 *
 * 全部是**全局**量（不随 chunk/层变化），可以只创建一次然后共用。
 */
struct PoseCloudUniforms {
    osg::ref_ptr<osg::Uniform> pointSize;      ///< uPointSize（像素）
    osg::ref_ptr<osg::Uniform> zRange;         ///< uZRange（世界 Z 的颜色范围）
    osg::ref_ptr<osg::Uniform> opacity;        ///< uOpacity（改透明度只写它）
    osg::ref_ptr<osg::Uniform> colorMode;      ///< uColorMode（0=Turbo，1=常量色）
    osg::ref_ptr<osg::Uniform> constantColor;  ///< uConstantColor（原始层用）
};

/**
 * @brief 把位姿点云着色器应用到 StateSet，并创建全局 uniform + 绑定 Turbo LUT
 *
 * **刻意不创建 `uFrameIdBias`**：它是每 chunk 不同的量。若在这里建好，那么按
 * chunk 浅拷贝 StateSet 时这个 Uniform 会被共享，给某个 chunk 设值会污染所有
 * chunk（症状：除了第一个 chunk，其余点云都用了错误的帧位姿，整体错位）。
 * 每 chunk 的 StateSet 请调 setPoseCloudFrameBias() 单独创建。
 *
 * @param ss        目标状态集（每层一个；层内所有 chunk 共用）
 * @param pointSize 初始点大小（像素）
 * @return 需要在运行时更新的全局 uniform 句柄
 */
inline PoseCloudUniforms applyPosePointCloudShader(osg::StateSet* ss, float pointSize = 3.0f) {
    PoseCloudUniforms u;
    ss->setAttributeAndModes(createPosePointCloudProgram(), osg::StateAttribute::ON);
    // Core Profile 下必须开启，顶点着色器里的 gl_PointSize 才会生效
    ss->setMode(GL_PROGRAM_POINT_SIZE, osg::StateAttribute::ON);

    u.pointSize     = new osg::Uniform("uPointSize", pointSize);
    u.zRange        = new osg::Uniform("uZRange", osg::Vec2(0.0f, 1.0f));
    u.opacity       = new osg::Uniform("uOpacity", 1.0f);
    u.colorMode     = new osg::Uniform("uColorMode", 0);
    u.constantColor = new osg::Uniform("uConstantColor", osg::Vec4(1.0f, 1.0f, 1.0f, 1.0f));
    ss->addUniform(u.pointSize.get());
    ss->addUniform(u.zRange.get());
    ss->addUniform(u.opacity.get());
    ss->addUniform(u.colorMode.get());
    ss->addUniform(u.constantColor.get());

    // Z 轴裁剪：沿用旧 shader 的同名 uniform，面板侧无需改
    ss->addUniform(new osg::Uniform("z_clipping", 0));
    ss->addUniform(new osg::Uniform("z_range", osg::Vec2(0.0f, 1.0f)));

    // 采样器 uniform 的值就是**纹理单元号**，纹理本身绑到同一单元
    ss->setTextureAttributeAndModes(kLutTexUnit, createTurboLutTexture(),
                                    osg::StateAttribute::ON);
    ss->addUniform(new osg::Uniform("uTurboLut", kLutTexUnit));

    // 位姿表纹理每层不同（优化槽/原始槽），这里只声明单元号，纹理由调用方绑
    ss->addUniform(new osg::Uniform("uPoseTable", kPoseTexUnit));
    return u;
}

/**
 * @brief 把某槽的位姿纹理绑到 kPoseTexUnit（每层调用一次）
 *
 * 两个层各自把自己槽位的纹理绑到同一个采样器单元 —— 这就是"双层共享顶点、
 * 只换位姿纹理"的落地方式，着色器因此不需要 uPoseOffset 之类的分支。
 */
inline void bindPoseTableTexture(osg::StateSet* ss, osg::TextureBuffer* poseTable) {
    if (!ss || !poseTable) return;
    ss->setTextureAttributeAndModes(kPoseTexUnit, poseTable, osg::StateAttribute::ON);
}

/**
 * @brief 为**每 chunk 的** StateSet 设置帧号基准（创建独立的 uFrameIdBias）
 *
 * 前提：`ss` 必须是该 chunk 独有的 StateSet（按层 StateSet 浅拷贝而来即可，
 * 纹理与全局 uniform 都是共享的，只有这一个值是独立的）。**不要**对层级的
 * 共享 StateSet 调用它。
 *
 * @param ss              该 chunk 的 StateSet
 * @param firstFrameIndex 该 chunk 的起始全局帧序（= CloudChunk::firstFrameIndex）
 */
inline void setPoseCloudFrameBias(osg::StateSet* ss, int firstFrameIndex) {
    if (!ss) return;
    osg::Uniform* u = ss->getUniform("uFrameIdBias");
    if (u) {
        u->set(firstFrameIndex);
        return;
    }
    // 新建（而不是改共享的那个）：见 applyPosePointCloudShader 的说明
    ss->addUniform(new osg::Uniform("uFrameIdBias", firstFrameIndex));
}
