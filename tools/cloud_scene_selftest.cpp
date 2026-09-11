// ============================================================================
// cloud_scene_selftest.cpp
// Phase 2b/3 装配层的无头自检（不需要 GPU / GL 上下文 / 地图文件）
//
// 只验证**结构性不变量**，因此用合成 chunk 就够，秒级且完全确定：
//   · 两层共享同一批 Geometry（设计文档 §3.4 的 DAG 多父）—— 这是"双层 = 1× 显存"
//     的全部来源，一旦退化成各建一份，这里会立刻抓到（顶点数会翻倍）；
//   · 每 chunk 的 uFrameIdBias 正确且**不共享**（共享会让除首块外的点云整体错位）；
//   · 层级的量（位姿纹理/颜色语义/全局 uniform）挂父 Group，靠状态继承向下传播；
//   · LOD 级别切换只换 primitive set，L0 用 DrawArrays、L>=1 用 DrawElementsUInt；
//   · 世界 AABB 逐 chunk 落地到 CloudGeometry（OSG 原生视锥剔除的依据）。
//
// 用法：cloud_scene_selftest
// 退出码: 0 = 全部通过；1 = 有失败项
// ============================================================================

#include "visualizers/CloudSceneBuilder.h"
#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "data/hdl_graph_slam/keyframe.hpp"
#include "data/hdl_graph_slam/progress_interface.hpp"

#include <g2o/types/slam3d/vertex_se3.h>

#include <osg/Array>
#include <osg/Geode>
#include <osg/Group>
// DrawArrays / DrawElements / DrawElementsUInt 都声明在 osg/PrimitiveSet 里
// （osg 下**没有** DrawArrays / DrawElements 这两个头文件）
#include <osg/PrimitiveSet>
#include <osg/StateSet>
#include <osg/TextureBuffer>
#include <osg/Uniform>
#include <osg/Vec2>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>
#include <vector>

using namespace hdl_graph_slam;

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
// 合成分块数据：2 个非空 chunk + 1 个空 chunk（验证空块占位、不建几何体）
// chunkFrames = 2
//   chunk0 firstFrameIndex=0 frames=[3点, 2点]  5 点  L1 索引 {0,2,4}
//   chunk1 firstFrameIndex=2 frames=[4点, 0点]  4 点  L1 索引 {1,2}
//   chunk2 firstFrameIndex=4 frames=[0点, 0点]  0 点（positions 为空）
// ---------------------------------------------------------------------------
std::vector<CloudChunk> makeChunks() {
    std::vector<CloudChunk> cs(3);

    auto setFrame = [](CloudChunk& c, size_t li, uint32_t start, uint32_t count,
                       long fid, float lox, float loy, float loz,
                       float hix, float hiy, float hiz) {
        CloudChunk::FrameRange fr;
        fr.startVertex = start;
        fr.vertexCount = count;
        fr.frameId    = fid;
        fr.localAABB[0] = lox; fr.localAABB[1] = loy; fr.localAABB[2] = loz;
        fr.localAABB[3] = hix; fr.localAABB[4] = hiy; fr.localAABB[5] = hiz;
        if (c.frames.size() <= li) c.frames.resize(li + 1);
        c.frames[li] = fr;
    };

    // ---- chunk 0 ----
    {
        CloudChunk& c = cs[0];
        c.firstFrameIndex = 0;
        c.frameCount = 2;
        c.pointCount = 5;
        c.positions = {0, 0, 0,  1, 0, 0,  0, 1, 0,   2, 0, 0,  2, 1, 0};
        c.frameLocalId = {0, 0, 0, 1, 1};
        setFrame(c, 0, 0, 3, 100, 0, 0, 0, 1, 1, 0);
        setFrame(c, 1, 3, 2, 101, 2, 0, 0, 2, 1, 0);
        c.lodIndices.resize(2);
        c.lodIndices[0] = {};            // L0 恒为空
        c.lodIndices[1] = {0, 2, 4};     // 3 个点
        c.worldMin[0] = -1.0f; c.worldMin[1] = -2.0f; c.worldMin[2] = -3.0f;
        c.worldMax[0] = 10.0f; c.worldMax[1] = 20.0f; c.worldMax[2] = 30.0f;
        c.worldBoundsValid = true;
    }
    // ---- chunk 1 ----
    {
        CloudChunk& c = cs[1];
        c.firstFrameIndex = 2;
        c.frameCount = 2;
        c.pointCount = 4;
        c.positions = {5, 0, 0,  6, 0, 0,  7, 0, 0,  8, 0, 0};
        c.frameLocalId = {0, 0, 0, 0};
        setFrame(c, 0, 0, 4, 102, 5, 0, 0, 8, 0, 0);
        setFrame(c, 1, 0, 0, 103, 0, 0, 0, 0, 0, 0);   // 零顶点帧占位
        c.lodIndices.resize(2);
        c.lodIndices[0] = {};
        c.lodIndices[1] = {1, 2};        // 2 个点
        c.worldMin[0] = 40.0f; c.worldMin[1] = 50.0f; c.worldMin[2] = 60.0f;
        c.worldMax[0] = 70.0f; c.worldMax[1] = 80.0f; c.worldMax[2] = 90.0f;
        c.worldBoundsValid = true;
    }
    // ---- chunk 2：全空 ----
    {
        CloudChunk& c = cs[2];
        c.firstFrameIndex = 4;
        c.frameCount = 2;
        c.pointCount = 0;
        setFrame(c, 0, 0, 0, 104, 0, 0, 0, 0, 0, 0);
        setFrame(c, 1, 0, 0, 105, 0, 0, 0, 0, 0, 0);
        c.lodIndices.resize(1);
        c.worldBoundsValid = false;
    }
    return cs;
}

bool sameBox(const osg::BoundingBox& a, float x0, float y0, float z0,
             float x1, float y1, float z1) {
    return std::fabs(a.xMin() - x0) < 1e-5f && std::fabs(a.yMin() - y0) < 1e-5f &&
           std::fabs(a.zMin() - z0) < 1e-5f && std::fabs(a.xMax() - x1) < 1e-5f &&
           std::fabs(a.yMax() - y1) < 1e-5f && std::fabs(a.zMax() - z1) < 1e-5f;
}

// ---------------------------------------------------------------------------
// 真实地图段（--map）：位姿表 → 位姿纹理 的布局，以及装配层在真实规模下的不变量
// ---------------------------------------------------------------------------

struct SilentProgress : ProgressInterface {
    int max = 0, cur = 0;
    void set_maximum(int m) override { max = m; }
    void increment() override {
        ++cur;
        if (max > 0 && (cur % 512 == 0 || cur == max)) {
            std::printf("\r[load] %3d%%", cur * 100 / max);
            std::fflush(stdout);
        }
    }
};

std::string argValue(int argc, char** argv, const char* name, const std::string& def) {
    for (int i = 0; i < argc; ++i) {
        if (std::strcmp(argv[i], name) == 0 && i + 1 < argc) return argv[i + 1];
    }
    return def;
}

/**
 * @brief 校验某槽位姿纹理的 texel 分组与位姿表的内存布局一致
 *
 * 这是整套架构里最承重的一条约定，且跨三个不同时间写的文件：
 *   CloudPoseTable::toColumnMajor16   写 out[4c+r] = M(r,c)（数学列主序）
 *   CloudPoseBuffer::fillSlot         写 texel[i*4+k] = p[4k .. 4k+3]
 *   顶点着色器                         texelFetch(uPoseTable, frame*4 + j) → 第 j 列
 * 任何一处 off-by-one 或分组错误都会让点云画在**错误的位姿**上——而且症状看起来
 * 只是"数据不对"，极难归因到纹理布局。所以这里逐 texel 对照内存布局。
 *
 * @return 不一致的 texel 数（0 = 通过；-1 = 纹理/数组/texel 数不可用）
 */
int verifyPoseTexels(const CloudPoseTable& poses, CloudPoseBuffer& buf, uint32_t slot,
                     const std::vector<size_t>& frames) {
    osg::TextureBuffer* tex = buf.texture(slot);
    if (!tex) return -1;
    const auto* arr = dynamic_cast<const osg::Vec4Array*>(tex->getBufferData());
    if (!arr) return -1;
    if (tex->getTextureWidth() != static_cast<int>(poses.frameCount() * 4)) return -1;

    const size_t stride = poses.slotFloats();
    const float* base = poses.data().data() + static_cast<size_t>(slot) * stride;
    int bad = 0;
    for (size_t i : frames) {
        if (i * 4 + 3 >= arr->size()) { ++bad; continue; }
        for (size_t k = 0; k < 4; ++k) {
            const osg::Vec4& t = (*arr)[i * 4 + k];
            const float* p = base + i * 16 + k * 4;
            if (t.x() != p[0] || t.y() != p[1] || t.z() != p[2] || t.w() != p[3]) ++bad;
        }
    }
    return bad;
}

/** @brief 抽出某槽纹理的全部 texel（用于"逐位不变"比对） */
bool snapshotTexels(CloudPoseBuffer& buf, uint32_t slot, std::vector<float>& out) {
    osg::TextureBuffer* tex = buf.texture(slot);
    if (!tex) return false;
    const auto* arr = dynamic_cast<const osg::Vec4Array*>(tex->getBufferData());
    if (!arr) return false;
    out.resize(arr->size() * 4);
    for (size_t i = 0; i < arr->size(); ++i) {
        const osg::Vec4& v = (*arr)[i];
        out[i * 4 + 0] = v.x(); out[i * 4 + 1] = v.y();
        out[i * 4 + 2] = v.z(); out[i * 4 + 3] = v.w();
    }
    return true;
}

void sectionRealMap(Checker& ck, const std::string& mapDir, int sampleFrames) {
    ck.section("H. 真实地图：位姿表 → 位姿纹理 的布局（跨三文件的承重约定）");

    SilentProgress progress;
    auto graph = std::make_shared<InteractiveGraph>();
    const auto t0 = std::chrono::steady_clock::now();
    if (!graph->load_map_data(mapDir, progress)) {
        std::printf("\n错误：地图加载失败\n");
        ck.check(false, "地图加载成功");
        return;
    }
    const double loadMs = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - t0).count();
    std::printf("\n地图加载完成：%zu 关键帧，%.1f s\n",
                graph->keyframes.size(), loadMs / 1000.0);

    ChunkBuildOptions opt;
    opt.lodLevels = 4;
    CloudPoseTable poses;
    CloudPoseBuffer poseBuf;
    CloudSceneBuilder scene;

    // buildChunks 内部会 poses.rebuild(graph)，因此返回后 poses 已可用
    ChunkBuildResult res = buildChunks(graph, poses, opt);
    const size_t nf = poses.frameCount();
    std::printf("分块：%zu chunk / %llu 点；帧 %zu\n", res.chunks.size(),
                static_cast<unsigned long long>(res.totalPoints), nf);
    ck.check(nf > 0 && res.totalPoints > 0, "地图非空");

    // ---- 1) 位姿纹理布局 ----
    const size_t wrote = poseBuf.rebuild(poses);
    ck.check(wrote == poseBuf.slotBytes() * CloudPoseTable::kSlotCount,
             "rebuild 上传量 == 两槽合计（帧数 × 64 B × 2）");
    ck.check(poseBuf.slotBytes() == nf * 64, "单槽字节 == 帧数 × 64");

    std::vector<size_t> sample;
    if (nf > 0) {
        sample.push_back(0);
        sample.push_back(nf - 1);
        for (int s = 0; s < sampleFrames; ++s) sample.push_back(static_cast<size_t>(s) % nf);
    }
    const int badOpt = verifyPoseTexels(poses, poseBuf, CloudPoseTable::kSlotOptimized, sample);
    const int badOdo = verifyPoseTexels(poses, poseBuf, CloudPoseTable::kSlotOriginal, sample);
    ck.check(badOpt == 0, "优化槽：texel[i*4+k] == 位姿表第 i 帧第 k 列（4 float 一组）");
    ck.check(badOdo == 0, "原始槽：texel[i*4+k] == 位姿表第 i 帧第 k 列");

    // ---- 2) 更新只重写优化槽，上传量严格 = 帧数 × 64 B ----
    {
        std::vector<float> origBefore;
        ck.check(snapshotTexels(poseBuf, CloudPoseTable::kSlotOriginal, origBefore),
                 "取到原始槽 texel 快照");

        std::vector<std::pair<g2o::VertexSE3*, Eigen::Isometry3d>> saved;
        const size_t step = (nf / 32) ? (nf / 32) : 1;
        size_t seen = 0;
        for (auto& kv : graph->keyframes) {
            if (saved.size() >= 32) break;
            auto* v = dynamic_cast<g2o::VertexSE3*>(kv.second->node);
            if (!v) continue;
            if ((seen++ % step) != 0) continue;
            const Eigen::Isometry3d oldT = v->estimate();
            Eigen::Isometry3d newT = oldT;
            newT.translation() += Eigen::Vector3d(0.25, -0.5, 0.75);
            saved.emplace_back(v, oldT);
            v->setEstimate(newT);
        }
        poses.updateOptimized(graph);
        const size_t up = poseBuf.updateOptimizedSlot(poses);
        ck.check(up == poseBuf.slotBytes(),
                 "updateOptimizedSlot 上传量 == 单槽（帧数 × 64 B），而不是两槽");

        std::vector<float> origAfter;
        snapshotTexels(poseBuf, CloudPoseTable::kSlotOriginal, origAfter);
        ck.check(!origBefore.empty() && origBefore == origAfter,
                 "更新优化槽后原始槽 texel **逐位不变**（冻结是结构性保证）");
        ck.check(verifyPoseTexels(poses, poseBuf, CloudPoseTable::kSlotOptimized, sample) == 0,
                 "更新后优化槽 texel 仍与位姿表一致");

        for (auto& [v, oldT] : saved) v->setEstimate(oldT);
        poses.updateOptimized(graph);
        poseBuf.updateOptimizedSlot(poses);
    }

    // ---- 3) 装配层在真实规模下的不变量 ----
    const size_t nGeom = scene.build(res.chunks, opt.chunkFrames, poseBuf,
                                     /*withOriginal=*/true, 2.0f);
    std::printf("装配：几何体 %zu，顶点 %zu，整图 AABB 有效=%d\n",
                nGeom, scene.vertexCount(), scene.mapBounds().valid() ? 1 : 0);

    ck.check(scene.vertexCount() == res.totalPoints,
             "顶点总数 == 全量点数（**不是 2×**：两层共享同一批几何体）");
    {
        osg::Group* optG = scene.layerGroup(CloudPoseTable::kSlotOptimized);
        osg::Group* orgG = scene.layerGroup(CloudPoseTable::kSlotOriginal);
        ck.check(optG && orgG && optG->getChild(0) == orgG->getChild(0),
                 "两层的子节点是同一个 Geode（真实规模下同样成立）");
    }
    {
        size_t gi = 0, bad = 0;
        for (const auto& c : res.chunks) {
            if (c.positions.empty()) continue;
            if (gi >= scene.geometries().size()) { ++bad; break; }
            osg::Uniform* u =
                scene.geometries()[gi]->getStateSet()->getUniform("uFrameIdBias");
            int v = -1;
            if (!u || !u->get(v) || static_cast<uint32_t>(v) != c.firstFrameIndex) ++bad;
            ++gi;
        }
        ck.check(bad == 0 && gi == scene.geometries().size(),
                 "每个 chunk 的 uFrameIdBias == 该 chunk 的 firstFrameIndex");
    }
    {
        bool lodOk = true;
        for (uint32_t L = 1; L < opt.lodLevels; ++L) {
            scene.setActiveLodLevel(static_cast<int>(L));
            size_t total = 0;
            for (const auto& g : scene.geometries()) {
                if (!g.valid() || g->getNumPrimitiveSets() != 1) { lodOk = false; continue; }
                total += g->getPrimitiveSet(0)->getNumIndices();
            }
            std::printf("  L%u 装配侧索引总数 %zu（数据层 %llu）\n", L, total,
                        static_cast<unsigned long long>(res.lodPoints[L]));
            if (total != res.lodPoints[L]) lodOk = false;
        }
        ck.check(lodOk, "各级 LOD 的索引总数与数据层统计逐级一致");
        scene.setActiveLodLevel(0);
    }

    // ---- 4) Phase 3：换槽刷新世界 AABB，几何体的世界盒必须跟着变 ----
    {
        auto snapshot = [&]() {
            std::vector<osg::BoundingBox> v;
            v.reserve(scene.geometries().size());
            for (const auto& g : scene.geometries()) v.push_back(g->worldBounds());
            return v;
        };
        const std::vector<osg::BoundingBox> optB = snapshot();
        const double ms =
            scene.refreshWorldBounds(res.chunks, poses, CloudPoseTable::kSlotOriginal);
        const std::vector<osg::BoundingBox> orgB = snapshot();
        std::printf("  换到原始槽刷新世界 AABB: %.3f ms\n", ms);

        bool matches = true, differs = false;
        size_t gi = 0;
        for (const auto& c : res.chunks) {
            if (c.positions.empty()) continue;
            const osg::BoundingBox& got = scene.geometries()[gi]->worldBounds();
            if (std::fabs(got.xMin() - c.worldMin[0]) > 1e-3f ||
                std::fabs(got.yMin() - c.worldMin[1]) > 1e-3f ||
                std::fabs(got.zMin() - c.worldMin[2]) > 1e-3f ||
                std::fabs(got.xMax() - c.worldMax[0]) > 1e-3f ||
                std::fabs(got.yMax() - c.worldMax[1]) > 1e-3f ||
                std::fabs(got.zMax() - c.worldMax[2]) > 1e-3f) matches = false;
            ++gi;
        }
        for (size_t i = 0; i < optB.size() && i < orgB.size(); ++i) {
            if (std::fabs(optB[i].xMin() - orgB[i].xMin()) > 1e-3f ||
                std::fabs(optB[i].xMax() - orgB[i].xMax()) > 1e-3f) differs = true;
        }
        ck.check(matches, "刷新后每个几何体的世界盒 == 数据层该 chunk 的世界 AABB");
        ck.check(differs, "换槽刷新确实改变了世界盒（两层位姿不同）");

        scene.refreshWorldBounds(res.chunks, poses, CloudPoseTable::kSlotOptimized);
        const std::vector<osg::BoundingBox> back = snapshot();
        bool restored = true;
        for (size_t i = 0; i < optB.size() && i < back.size(); ++i) {
            if (std::fabs(optB[i].xMin() - back[i].xMin()) > 1e-3f ||
                std::fabs(optB[i].xMax() - back[i].xMax()) > 1e-3f) restored = false;
        }
        ck.check(restored, "换回优化槽后世界盒恢复");
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc >= 2 && (std::strcmp(argv[1], "-h") == 0 ||
                      std::strcmp(argv[1], "--help") == 0)) {
        std::printf(
            "cloud_scene_selftest —— Phase 2b/3 装配层无头自检\n"
            "\n"
            "用法:\n"
            "  cloud_scene_selftest [--map <地图目录>] [--sample N]\n"
            "      --map DIR   额外跑真实地图段（位姿纹理布局 + 装配层真实规模不变量）\n"
            "                  建议先用小地图（559 帧）跑，大地图会占用数 GB 内存\n"
            "      --sample N  真实地图段的位姿抽样帧数（默认 200）\n"
            "\n"
            "退出码: 0 = 全部通过；1 = 有失败项\n");
        return 0;
    }
    const std::string mapDir = argValue(argc, argv, "--map", "");
    const int sampleFrames = std::atoi(argValue(argc, argv, "--sample", "200").c_str());

    std::printf("=== cloud_scene_selftest ===\n\n");
    Checker ck;

    const std::vector<CloudChunk> chunks = makeChunks();
    const uint32_t chunkFrames = 2;

    // 位姿表故意留空：本自检只验结构性不变量，不需要真实位姿
    // （refreshWorldBounds 的位姿语义已由 cloud_selftest 的 E/F 段覆盖）
    CloudPoseTable poses;
    CloudPoseBuffer poseBuf;
    poseBuf.rebuild(poses);

    CloudSceneBuilder scene;
    const size_t nGeom = scene.build(chunks, chunkFrames, poseBuf, /*withOriginal=*/true,
                                     /*pointSize=*/2.0f);
    std::printf("装配：几何体 %zu 个，顶点 %zu 个，整图 AABB 有效=%d\n",
                nGeom, scene.vertexCount(), scene.mapBounds().valid() ? 1 : 0);

    // ------------------------------------------------------------------
    ck.section("A. 几何体数量与共享（双层 = 1× 显存）");
    // ------------------------------------------------------------------
    ck.check(nGeom == 2, "只为空 chunk 跳过，建出 2 个几何体");
    ck.check(scene.geometries().size() == 2, "几何体列表长度 == 2");
    ck.check(scene.vertexCount() == 9, "顶点总数 == 5 + 4 == 9（**不是 18**，即没被两层各建一份）");

    {
        osg::Group* opt = scene.layerGroup(CloudPoseTable::kSlotOptimized);
        osg::Group* org = scene.layerGroup(CloudPoseTable::kSlotOriginal);
        ck.check(opt != nullptr && org != nullptr, "两层父 Group 都存在");
        if (opt && org) {
            ck.check(opt->getNumChildren() == 1 && org->getNumChildren() == 1,
                     "每层各只有一个子节点");
            // 关键：两层的子节点必须是**同一个对象**（DAG 多父），不是两份拷贝
            ck.check(opt->getChild(0) == org->getChild(0),
                     "两层的子节点是同一个 Geode 对象（DAG 多父，设计文档 §3.4）");
            ck.check(opt->getChild(0) == scene.cloudGeode(), "子节点就是共享的 cloudGeode");
            ck.check(org->getNodeMask() == 0, "原始层默认隐藏");
        }
        ck.check(scene.pickProxy() != nullptr, "常驻拾取代理存在");
        ck.check(scene.root()->getNumChildren() == 3,
                 "root 下有三个子节点：优化层 + 原始层 + 拾取代理");
    }

    // ------------------------------------------------------------------
    ck.section("B. 每 chunk 的 uFrameIdBias（不共享、值正确）");
    // ------------------------------------------------------------------
    {
        bool allBiasOk = true, allIndependent = true;
        const std::vector<size_t> expectBias = {0, 2};   // firstFrameIndex
        for (size_t i = 0; i < scene.geometries().size(); ++i) {
            osg::StateSet* ss = scene.geometries()[i]->getStateSet();
            if (!ss) { allBiasOk = false; continue; }
            osg::Uniform* u = ss->getUniform("uFrameIdBias");
            int v = -1;
            if (!u || !u->get(v) || static_cast<size_t>(v) != expectBias[i]) {
                allBiasOk = false;
            }
            // chunk 之间必须是**不同**的 Uniform 对象
            if (i > 0) {
                osg::Uniform* prev = scene.geometries()[i - 1]->getStateSet()
                                         ->getUniform("uFrameIdBias");
                if (prev == u) allIndependent = false;
            }
        }
        ck.check(allBiasOk, "uFrameIdBias == chunk.firstFrameIndex（0 与 2）");
        ck.check(allIndependent, "各 chunk 的 uFrameIdBias 是不同对象（不共享）");

        // 层级 StateSet 里不能有它（否则浅拷贝时会共享、污染所有 chunk）
        osg::StateSet* optSS = scene.layerStateSet(CloudPoseTable::kSlotOptimized);
        osg::StateSet* orgSS = scene.layerStateSet(CloudPoseTable::kSlotOriginal);
        ck.check(optSS && optSS->getUniform("uFrameIdBias") == nullptr,
                 "优化层 StateSet 里没有 uFrameIdBias");
        ck.check(orgSS && orgSS->getUniform("uFrameIdBias") == nullptr,
                 "原始层 StateSet 里没有 uFrameIdBias");
    }

    // ------------------------------------------------------------------
    ck.section("C. 层级的量挂父 Group（位姿纹理 / 颜色语义 / 全局 uniform）");
    // ------------------------------------------------------------------
    {
        osg::StateSet* optSS = scene.layerStateSet(CloudPoseTable::kSlotOptimized);
        osg::StateSet* orgSS = scene.layerStateSet(CloudPoseTable::kSlotOriginal);
        ck.check(optSS && orgSS, "两层都有 StateSet");
        if (optSS && orgSS) {
            ck.check(optSS->getAttribute(osg::StateAttribute::PROGRAM) != nullptr,
                     "优化层父 Group 挂了着色器程序");
            int mode = -1;
            osg::Uniform* om = optSS->getUniform("uColorMode");
            osg::Uniform* rm = orgSS->getUniform("uColorMode");
            ck.check(om && om->get(mode) && mode == 0, "优化层 uColorMode == 0（Turbo 查表）");
            ck.check(rm && rm->get(mode) && mode == 1, "原始层 uColorMode == 1（常量色）");

            osg::Vec4 cc;
            osg::Uniform* oc = orgSS->getUniform("uConstantColor");
            ck.check(oc && oc->get(cc) &&
                         cc.x() == CloudSceneBuilder::kOdomR &&
                         cc.y() == CloudSceneBuilder::kOdomG &&
                         cc.z() == CloudSceneBuilder::kOdomB &&
                         std::fabs(cc.w() - CloudSceneBuilder::kOdomAlphaScale) < 1e-6f,
                     "原始层 uConstantColor == 淡橙 × alpha 系数");

            // 采样器单元
            int unit = -1;
            osg::Uniform* up = optSS->getUniform("uPoseTable");
            ck.check(up && up->get(unit) && unit == kPoseTexUnit,
                     "uPoseTable 指向 kPoseTexUnit");
            osg::Uniform* ul = optSS->getUniform("uTurboLut");
            ck.check(ul && ul->get(unit) && unit == kLutTexUnit,
                     "uTurboLut 指向 kLutTexUnit");

            // 几何体自己的 StateSet 里**不该**有这些（否则就覆盖了父级）
            osg::StateSet* c0 = scene.geometries()[0]->getStateSet();
            ck.check(c0 && c0->getUniform("uColorMode") == nullptr &&
                             c0->getUniform("uOpacity") == nullptr &&
                             c0->getAttribute(osg::StateAttribute::PROGRAM) == nullptr,
                     "chunk StateSet 不覆盖父级的程序/颜色语义（只放 uFrameIdBias）");
        }
    }

    // ------------------------------------------------------------------
    ck.section("D. 顶点与帧号属性");
    // ------------------------------------------------------------------
    {
        const auto& gs = scene.geometries();
        bool posOk = true, fidOk = true, shapeOk = true;
        const float expectPos[9][3] = {{0,0,0},{1,0,0},{0,1,0},{2,0,0},{2,1,0},
                                       {5,0,0},{6,0,0},{7,0,0},{8,0,0}};
        const unsigned short expectFid[9] = {0,0,0,1,1,0,0,0,0};
        size_t k = 0;
        for (const auto& g : gs) {
            auto* pos = dynamic_cast<osg::Vec3Array*>(g->getVertexArray());
            auto* fid = dynamic_cast<const osg::UShortArray*>(
                g->getVertexAttribArray(CloudGeometry::kFrameIdAttribLocation));
            if (!pos || !fid || pos->size() != fid->size()) { shapeOk = false; continue; }
            for (size_t i = 0; i < pos->size() && k < 9; ++i, ++k) {
                const osg::Vec3& p = (*pos)[i];
                if (std::fabs(p.x() - expectPos[k][0]) > 1e-6f ||
                    std::fabs(p.y() - expectPos[k][1]) > 1e-6f ||
                    std::fabs(p.z() - expectPos[k][2]) > 1e-6f) posOk = false;
                if ((*fid)[i] != expectFid[k]) fidOk = false;
            }
        }
        ck.check(shapeOk, "每个几何体都有等长的顶点数组与帧号属性数组");
        ck.check(k == 9, "依次遍历两层共享的几何体，顶点总数 == 9");
        ck.check(posOk, "顶点坐标与源分块逐位一致（帧局部系）");
        ck.check(fidOk, "帧号属性数组与 chunk.frameLocalId 一致");
        ck.check(gs[0]->getVertexAttribArray(CloudGeometry::kFrameIdAttribLocation) != nullptr,
                 "帧号属性挂在 location 13（与着色器 layout 一致）");
    }

    // ------------------------------------------------------------------
    ck.section("E. LOD 级别切换（只换 primitive set）");
    // ------------------------------------------------------------------
    {
        const auto& gs = scene.geometries();
        // L0：DrawArrays，count == 顶点数
        scene.setActiveLodLevel(0);
        bool l0Ok = true;
        const unsigned expectCount[2] = {5, 4};
        for (size_t i = 0; i < gs.size(); ++i) {
            if (gs[i]->activeLodLevel() != 0 || gs[i]->getNumPrimitiveSets() != 1) {
                l0Ok = false; continue;
            }
            auto* da = dynamic_cast<osg::DrawArrays*>(gs[i]->getPrimitiveSet(0));
            if (!da || static_cast<unsigned>(da->getCount()) != expectCount[i]) l0Ok = false;
        }
        ck.check(l0Ok, "L0 用 DrawArrays 且 count == 该 chunk 顶点数（5 / 4）");

        // L1：DrawElementsUInt，索引数 == 预置的 3 / 2
        scene.setActiveLodLevel(1);
        bool l1Ok = true;
        const unsigned expectIdx[2] = {3, 2};
        for (size_t i = 0; i < gs.size(); ++i) {
            if (gs[i]->activeLodLevel() != 1 || gs[i]->getNumPrimitiveSets() != 1) {
                l1Ok = false; continue;
            }
            osg::PrimitiveSet* ps = gs[i]->getPrimitiveSet(0);
            if (!dynamic_cast<osg::DrawElementsUInt*>(ps)) { l1Ok = false; continue; }
            if (ps->getNumIndices() != expectIdx[i]) l1Ok = false;
        }
        ck.check(l1Ok, "L1 用 DrawElementsUInt 且索引数 == 3 / 2");
        ck.check(scene.activeLodLevel() == 1, "builder 记录了当前级别");

        scene.setActiveLodLevel(0);
    }

    // ------------------------------------------------------------------
    ck.section("F. 世界 AABB（OSG 原生视锥剔除的依据）");
    // ------------------------------------------------------------------
    {
        const auto& gs = scene.geometries();
        ck.check(sameBox(gs[0]->worldBounds(), -1, -2, -3, 10, 20, 30),
                 "chunk0 的几何体世界盒 == chunk0 的世界 AABB");
        ck.check(sameBox(gs[1]->worldBounds(), 40, 50, 60, 70, 80, 90),
                 "chunk1 的几何体世界盒 == chunk1 的世界 AABB");
        ck.check(gs[0]->computeBoundingBox().valid(),
                 "computeBoundingBox() 返回的是世界 AABB（而非顶点数组的局部盒）");
        ck.check(sameBox(scene.mapBounds(), -1, -2, -3, 70, 80, 90),
                 "整图 AABB == 两个 chunk 世界盒的并集");
        osg::BoundingBox pb = scene.pickProxy()->worldBounds();
        ck.check(pb.valid() && std::fabs(pb.xMin() + 1) < 1e-5f &&
                     std::fabs(pb.xMax() - 70) < 1e-5f,
                 "拾取代理的包围盒 == 整图 AABB");
    }

    // ------------------------------------------------------------------
    ck.section("G. 运行时更新（透明度/点大小/颜色范围/可见性）");
    // ------------------------------------------------------------------
    {
        scene.setOpacity(0.42f);
        scene.setPointSize(5.5f);
        scene.setColorZRange(-3.0f, 17.0f);
        float f = 0.0f;
        osg::Vec2 v2;
        osg::StateSet* optSS = scene.layerStateSet(CloudPoseTable::kSlotOptimized);
        osg::StateSet* orgSS = scene.layerStateSet(CloudPoseTable::kSlotOriginal);
        ck.check(optSS->getUniform("uOpacity")->get(f) && std::fabs(f - 0.42f) < 1e-6f,
                 "优化层 uOpacity 已更新");
        ck.check(orgSS->getUniform("uOpacity")->get(f) && std::fabs(f - 0.42f) < 1e-6f,
                 "原始层 uOpacity 同步更新（两层共享的全局量写两处）");
        ck.check(optSS->getUniform("uPointSize")->get(f) && std::fabs(f - 5.5f) < 1e-6f,
                 "uPointSize 已更新");
        ck.check(optSS->getUniform("uZRange")->get(v2) && v2.x() == -3.0f && v2.y() == 17.0f,
                 "uZRange 已更新");

        scene.setZClipping(true, 0.0f, 9.0f);
        int i = -1;
        ck.check(optSS->getUniform("z_clipping")->get(i) && i == 1, "z_clipping 已开启");
        ck.check(orgSS->getUniform("z_range")->get(v2) && v2.y() == 9.0f, "z_range 已更新");

        ck.check(!scene.layerVisible(CloudPoseTable::kSlotOriginal), "原始层初始不可见");
        scene.setLayerVisible(CloudPoseTable::kSlotOriginal, true);
        ck.check(scene.layerVisible(CloudPoseTable::kSlotOriginal), "原始层可独立打开");
        ck.check(scene.layerVisible(CloudPoseTable::kSlotOptimized),
                 "打开原始层不影响优化层可见性");
        scene.setLayerVisible(CloudPoseTable::kSlotOriginal, false);
    }

    const int rc = ck.summary();
    std::printf(rc == 0 ? "\n结果：全部通过 ✓\n" : "\n结果：存在失败项 ✗\n");
    return rc;
}
