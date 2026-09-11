// ============================================================================
// cloud_shader_selftest.cpp
// Phase 2b GL 侧的无头自检（不需要 GPU / GL 上下文 / 窗口）
//
// 为什么值得单独做：GLSL 里 uniform 名字写错是**静默失败** —— 着色器照常编译、
// OSG 照常"设置"那个不存在的 uniform（找不到 location 就什么都不做），最后只是
// 画面悄悄不对（例如颜色全一样、点云挤成一团）。等到了 GPU 上才发现，归因成本很高。
// 所以这里在 CPU 侧把"sahder 声明了什么"与"C++ 创建了什么"做双向交叉比对。
//
// 另外两段同样只能靠静态判定、但错了就会静默出错的东西：
//   · Turbo 色表 CPU/GPU 一致性（含取色索引的**截断**语义）
//   · uFrameIdBias 的每 chunk 独立性（浅拷贝 StateSet 时共享 uniform 会污染所有 chunk）
//
// 用法：cloud_shader_selftest
// 退出码: 0 = 全部通过；1 = 有失败项
// ============================================================================

#include "visualizers/CoreShaders.h"
#include "visualizers/CloudGeometry.h"   // kFrameIdAttribLocation
#include "visualizers/TurboColormap.h"

#include <osg/CopyOp>
#include <osg/Program>
#include <osg/Shader>
#include <osg/StateSet>
#include <osg/TextureBuffer>
#include <osg/Uniform>
#include <osg/Vec4Array>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace {

struct Checker {
    int checks = 0, failures = 0;
    void check(bool cond, const char* what) {
        ++checks;
        if (!cond) {
            ++failures;
            std::printf("  [FAIL] %s\n", what);
            std::fflush(stdout);
        }
    }
    void section(const char* name) {
        std::printf("-- %s\n", name);
        std::fflush(stdout);
    }
    int summary() const {
        std::printf("\n自检项 %d，失败 %d\n", checks, failures);
        return failures == 0 ? 0 : 1;
    }
};

// ---------------------------------------------------------------------------
// 极简 GLSL 声明提取（只认 uniform / in，足够做契约比对）
// ---------------------------------------------------------------------------

std::string stripLineComments(const std::string& s) {
    std::string out;
    out.reserve(s.size());
    size_t i = 0;
    while (i < s.size()) {
        const size_t e = s.find('\n', i);
        const std::string line =
            s.substr(i, e == std::string::npos ? std::string::npos : e - i);
        const size_t c = line.find("//");
        out += (c == std::string::npos) ? line : line.substr(0, c);
        out += '\n';
        if (e == std::string::npos) break;
        i = e + 1;
    }
    return out;
}

std::vector<std::string> tokenize(const std::string& s) {
    std::vector<std::string> t;
    std::string cur;
    for (char ch : s) {
        const unsigned char u = static_cast<unsigned char>(ch);
        if (std::isalnum(u) || ch == '_') {
            cur += ch;
        } else {
            if (!cur.empty()) { t.push_back(cur); cur.clear(); }
            if (ch == '(' || ch == ')' || ch == ';' || ch == '=') t.push_back(std::string(1, ch));
        }
    }
    if (!cur.empty()) t.push_back(cur);
    return t;
}

struct GlslDecls {
    std::vector<std::pair<std::string, std::string>> uniforms;    // (type, name)
    std::vector<std::pair<std::string, std::string>> attributes;  // (type, name)
    int  frameIdLocation = -1;
    bool hasOsgVertex = false;
    bool hasOsgColor  = false;
    bool usesVWorldPos = false;
};

GlslDecls parseGlsl(const std::string& src) {
    GlslDecls d;
    const std::string clean = stripLineComments(src);
    d.usesVWorldPos = clean.find("vWorldPos") != std::string::npos;

    const std::vector<std::string> tk = tokenize(clean);
    int pendingLocation = -1;
    for (size_t i = 0; i < tk.size(); ++i) {
        const std::string& t = tk[i];
        if (t == "layout") {
            int loc = -1;
            size_t j = i + 1;
            for (; j < tk.size() && tk[j] != ")"; ++j) {
                if (tk[j] == "location" && j + 2 < tk.size() && tk[j + 1] == "=") {
                    loc = std::atoi(tk[j + 2].c_str());
                }
            }
            pendingLocation = loc;
            if (j < tk.size()) i = j;   // 跳到 ')'
            continue;
        }
        if (t == "uniform" && i + 2 < tk.size()) {
            d.uniforms.emplace_back(tk[i + 1], tk[i + 2]);
            i += 2;
            continue;
        }
        if (t == "in" && i + 2 < tk.size()) {
            d.attributes.emplace_back(tk[i + 1], tk[i + 2]);
            if (tk[i + 2] == "aFrameId")   d.frameIdLocation = pendingLocation;
            if (tk[i + 2] == "osg_Vertex")  d.hasOsgVertex = true;
            if (tk[i + 2] == "osg_Color")   d.hasOsgColor  = true;
            pendingLocation = -1;
            i += 2;
            continue;
        }
    }
    return d;
}

bool hasUniform(const GlslDecls& d, const std::string& name) {
    for (const auto& u : d.uniforms) if (u.second == name) return true;
    return false;
}

std::string uniformType(const GlslDecls& d, const std::string& name) {
    for (const auto& u : d.uniforms) if (u.second == name) return u.first;
    return std::string();
}

// 供自检使用的 GLSL 源码（按类型取）
const std::string* shaderSourceByType(osg::Program* prog, osg::Shader::Type type) {
    for (unsigned i = 0; i < prog->getNumShaders(); ++i) {
        osg::Shader* sh = prog->getShader(i);
        if (sh && sh->getType() == type) return &sh->getShaderSource();
    }
    return nullptr;
}

// ---------------------------------------------------------------------------
// 段 A：Turbo 色表 CPU/GPU 一致性
// ---------------------------------------------------------------------------

/** @brief 复刻片段着色器里的取色索引语义（截断，不是四舍五入） */
int gpuLutIndex(double z, double zMin, double zMax, bool& gray) {
    gray = false;
    const double range = zMax - zMin;
    if (range <= 0.0) { gray = true; return -1; }
    const double t = (z - zMin) / range;
    // GLSL int() 与 C++ static_cast<int>() 同为**向零截断**
    int idx = static_cast<int>(t * 255.0);
    if (idx < 0) idx = 0;
    if (idx > 255) idx = 255;
    return idx;
}

void sectionLut(Checker& ck) {
    ck.section("A. Turbo 色表：纹理内容与取色语义（CPU/GPU 必须逐位一致）");

    osg::TextureBuffer* tex = createTurboLutTexture();
    ck.check(tex != nullptr, "createTurboLutTexture() 返回非空");

    ck.check(tex->getTextureWidth() == 256, "色表 texel 数 == 256");
    ck.check(tex->getFilter(osg::Texture::MIN_FILTER) == osg::Texture::NEAREST &&
             tex->getFilter(osg::Texture::MAG_FILTER) == osg::Texture::NEAREST,
             "色表用 NEAREST 过滤（离散查表，线性插值会产生 CPU 侧没有的颜色）");

    const auto* data = dynamic_cast<const osg::Vec4Array*>(tex->getBufferData());
    ck.check(data != nullptr, "纹理绑定的数据是 osg::Vec4Array");
    if (!data) return;
    ck.check(data->size() == 256, "绑定数组长度 == 256");

    const float (*tbl)[3] = turboTable();
    int bad = 0, badAlpha = 0;
    for (size_t i = 0; i < 256 && i < data->size(); ++i) {
        const osg::Vec4& v = (*data)[i];
        if (v.x() != tbl[i][0] || v.y() != tbl[i][1] || v.z() != tbl[i][2]) ++bad;
        if (v.w() != 1.0f) ++badAlpha;
    }
    ck.check(bad == 0, "256 项 RGB 与 turboTable() 逐位相同");
    ck.check(badAlpha == 0, "色表 alpha 恒为 1（透明度由 uOpacity 统一乘）");

    // 取色语义：在 t<0 / t=0 / t=0.5 / t=1 / t>1 与 range<=0 上逐点对照 CPU turboColor()
    int mismatch = 0, grayMismatch = 0;
    const double zMin = -12.5, zMax = 37.25;
    for (int k = -20; k <= 120; ++k) {
        const double z = zMin + (zMax - zMin) * (k / 100.0);   // 覆盖 t<0..t>1.2
        bool gray = false;
        const int idx = gpuLutIndex(z, zMin, zMax, gray);
        const osg::Vec4 cpu = turboColor(static_cast<float>(z),
                                         static_cast<float>(zMin),
                                         static_cast<float>(zMax));
        if (gray) continue;
        const osg::Vec4& g = (*data)[static_cast<size_t>(idx)];
        if (g.x() != cpu.x() || g.y() != cpu.y() || g.z() != cpu.z()) ++mismatch;
    }
    ck.check(mismatch == 0, "z 扫描上 GPU 索引公式与 turboColor() 逐位一致（截断语义）");

    // range <= 0 的分支：着色器给灰 (0.5,0.5,0.5,1)，CPU turboColor() 同
    {
        bool gray = false;
        gpuLutIndex(1.0, 5.0, 5.0, gray);
        const osg::Vec4 cpu = turboColor(1.0f, 5.0f, 5.0f);
        if (!gray || cpu.x() != 0.5f || cpu.y() != 0.5f || cpu.z() != 0.5f) ++grayMismatch;
    }
    ck.check(grayMismatch == 0, "range <= 0 时 GPU 与 CPU 都退化为灰色 (0.5,0.5,0.5)");

    // 索引公式必须是截断：源码里不能出现 +0.5 的四舍五入写法
    osg::ref_ptr<osg::Program> prog = createPosePointCloudProgram();
    const std::string* frag = shaderSourceByType(prog.get(), osg::Shader::FRAGMENT);
    ck.check(frag != nullptr, "取到片段着色器源码");
    if (frag) {
        const std::string clean = stripLineComments(*frag);
        ck.check(clean.find("int(t * 255.0)") != std::string::npos,
                 "片段着色器用 int(t * 255.0) 截断取色");
        ck.check(clean.find("int(t * 255.0 + 0.5)") == std::string::npos,
                 "片段着色器没有写成四舍五入 int(t * 255.0 + 0.5)");
    }
}

// ---------------------------------------------------------------------------
// 段 B：着色器 ↔ C++ uniform 契约
// ---------------------------------------------------------------------------

void sectionContract(Checker& ck) {
    ck.section("B. 着色器 ↔ C++ uniform 契约（抓打错名字这类静默失败）");

    osg::ref_ptr<osg::Program> prog = createPosePointCloudProgram();
    ck.check(prog.valid(), "createPosePointCloudProgram() 返回非空");
    ck.check(prog->getNumShaders() == 2, "程序含顶点 + 片段两个着色器");
    if (!prog.valid()) return;

    const std::string* vsrc = shaderSourceByType(prog.get(), osg::Shader::VERTEX);
    const std::string* fsrc = shaderSourceByType(prog.get(), osg::Shader::FRAGMENT);
    ck.check(vsrc != nullptr && fsrc != nullptr, "取到顶点与片段着色器源码");
    if (!vsrc || !fsrc) return;

    const GlslDecls vd = parseGlsl(*vsrc);
    const GlslDecls fd = parseGlsl(*fsrc);

    // —— 属性 ——
    ck.check(vd.hasOsgVertex, "顶点着色器声明了 OSG 别名属性 osg_Vertex");
    ck.check(!vd.hasOsgColor,
             "顶点着色器没有声明 osg_Color（新路径颜色来自 LUT，不该再绑逐顶点色）");
    ck.check(vd.frameIdLocation == static_cast<int>(hdl_graph_slam::CloudGeometry::kFrameIdAttribLocation),
             "aFrameId 的 layout(location) == CloudGeometry::kFrameIdAttribLocation");
    ck.check(uniformType(vd, "aFrameId").empty(), "aFrameId 是属性而不是 uniform");

    // —— 片段着色器必须用世界坐标 ——
    ck.check(fd.usesVWorldPos,
             "片段着色器用 vWorldPos（世界坐标）而不是局部坐标做裁剪/配色");
    ck.check(stripLineComments(*fsrc).find("osg_Vertex") == std::string::npos,
             "片段着色器里不出现 osg_Vertex（帧局部坐标不能用于配色）");

    // —— StateSet 侧 ——
    osg::ref_ptr<osg::StateSet> base = new osg::StateSet;
    const PoseCloudUniforms u = applyPosePointCloudShader(base.get(), 2.5f);

    std::set<std::string> cppUniforms;
    for (const auto& kv : base->getUniformList()) cppUniforms.insert(kv.first);

    // OSG 自动提供的内建量，Shader 里必须声明、但不由我们创建
    const std::set<std::string> kOsgBuiltins = {"osg_ModelViewProjectionMatrix"};
    // 文档明确：uFrameIdBias 是**每 chunk** 的，刻意不在层级 StateSet 里创建
    const std::set<std::string> kPerChunk = {"uFrameIdBias"};

    // ① 着色器声明的每个 uniform，C++ 侧都得有（内建与每 chunk 的除外）
    std::vector<std::string> missing;
    for (const auto& d : vd.uniforms) {
        if (kOsgBuiltins.count(d.second) || kPerChunk.count(d.second)) continue;
        if (!cppUniforms.count(d.second)) missing.push_back(d.second);
    }
    for (const auto& d : fd.uniforms) {
        if (kOsgBuiltins.count(d.second) || kPerChunk.count(d.second)) continue;
        if (!cppUniforms.count(d.second)) missing.push_back(d.second);
    }
    if (!missing.empty()) {
        std::printf("  着色器声明但 C++ 未创建：");
        for (const auto& m : missing) std::printf(" %s", m.c_str());
        std::printf("\n");
    }
    ck.check(missing.empty(), "着色器声明的每个 uniform 都被 C++ 创建了");

    // ② C++ 创建的每个 uniform，着色器里都得声明（打错名字会在这里露出来）
    std::vector<std::string> undeclared;
    for (const auto& name : cppUniforms) {
        if (!hasUniform(vd, name) && !hasUniform(fd, name)) undeclared.push_back(name);
    }
    if (!undeclared.empty()) {
        std::printf("  C++ 创建但着色器未声明：");
        for (const auto& m : undeclared) std::printf(" %s", m.c_str());
        std::printf("\n");
    }
    ck.check(undeclared.empty(), "C++ 创建的每个 uniform 都在着色器里声明了");

    // ③ 采样器 uniform 的值必须是纹理单元号
    {
        int unit = -1;
        osg::Uniform* up = base->getUniform("uPoseTable");
        osg::Uniform* ul = base->getUniform("uTurboLut");
        ck.check(up && ul, "uPoseTable / uTurboLut 都存在");
        if (up && up->get(unit)) ck.check(unit == kPoseTexUnit, "uPoseTable == kPoseTexUnit");
        if (ul && ul->get(unit)) ck.check(unit == kLutTexUnit, "uTurboLut == kLutTexUnit");
    }

    // ④ 返回值句柄指向的确实是 StateSet 里那两个对象（不是新造的孤儿 uniform）
    ck.check(u.pointSize.valid() && base->getUniform("uPointSize") == u.pointSize.get(),
             "返回的 uPointSize 句柄就是 StateSet 里那一个");
    ck.check(u.zRange.valid() && base->getUniform("uZRange") == u.zRange.get(),
             "返回的 uZRange 句柄就是 StateSet 里那一个");
    ck.check(u.opacity.valid() && base->getUniform("uOpacity") == u.opacity.get(),
             "返回的 uOpacity 句柄就是 StateSet 里那一个");
    ck.check(u.colorMode.valid() && base->getUniform("uColorMode") == u.colorMode.get(),
             "返回的 uColorMode 句柄就是 StateSet 里那一个");

    // ⑤ 初值
    {
        float f = 0.0f;
        int   i = -1;
        osg::Vec2 v2;
        if (u.pointSize->get(f)) ck.check(std::fabs(f - 2.5f) < 1e-6f, "uPointSize 初值按入参设置");
        if (u.opacity->get(f))   ck.check(std::fabs(f - 1.0f) < 1e-6f, "uOpacity 初值 1.0");
        if (u.colorMode->get(i)) ck.check(i == 0, "uColorMode 初值 0（Turbo 模式）");
        osg::Uniform* zc = base->getUniform("z_clipping");
        if (zc && zc->get(i)) ck.check(i == 0, "z_clipping 初值 0（裁剪默认关闭）");
        osg::Uniform* zr = base->getUniform("z_range");
        if (zr && zr->get(v2)) ck.check(v2.x() == 0.0f && v2.y() == 1.0f, "z_range 初值 [0,1]");
        ck.check(u.zRange->get(v2) && v2.x() == 0.0f && v2.y() == 1.0f, "uZRange 初值 [0,1]");
    }

    // ⑥ 每 chunk 的 uFrameIdBias 必须独立（浅拷贝 StateSet 时最容易踩的坑）
    {
        ck.check(base->getUniform("uFrameIdBias") == nullptr,
                 "层级 StateSet 里**没有** uFrameIdBias（否则会被所有 chunk 共享）");

        osg::ref_ptr<osg::StateSet> clone =
            static_cast<osg::StateSet*>(base->clone(osg::CopyOp::SHALLOW_COPY));
        setPoseCloudFrameBias(clone.get(), 1234);
        const osg::Uniform* uc = clone->getUniform("uFrameIdBias");
        int bias = -1;
        ck.check(uc != nullptr, "setPoseCloudFrameBias() 在 chunk StateSet 上创建了 uniform");
        if (uc && uc->get(bias)) ck.check(bias == 1234, "uFrameIdBias 值正确");
        ck.check(base->getUniform("uFrameIdBias") == nullptr,
                 "给 chunk 设 bias 不会污染层级 StateSet（无共享 uniform）");

        setPoseCloudFrameBias(clone.get(), 7);
        if (uc && uc->get(bias)) ck.check(bias == 7, "重复调用是就地更新而不是叠加");

        ck.check(clone->getAttribute(osg::StateAttribute::PROGRAM) ==
                     base->getAttribute(osg::StateAttribute::PROGRAM),
                 "chunk StateSet 与层级 StateSet 共享同一个 Program（浅拷贝语义）");
    }

    // ⑦ 点大小模式必须开启，否则 gl_PointSize 被静默忽略
    ck.check(base->getMode(GL_PROGRAM_POINT_SIZE) & osg::StateAttribute::ON,
             "GL_PROGRAM_POINT_SIZE 已开启（否则 gl_PointSize 无效）");
}

}  // namespace

int main() {
    std::printf("=== cloud_shader_selftest ===\n\n");
    Checker ck;
    sectionLut(ck);
    sectionContract(ck);
    const int rc = ck.summary();
    std::printf(rc == 0 ? "\n结果：全部通过 ✓\n" : "\n结果：存在失败项 ✗\n");
    return rc;
}
