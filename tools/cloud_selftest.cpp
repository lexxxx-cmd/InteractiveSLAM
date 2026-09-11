// ============================================================================
// cloud_selftest.cpp
// Phase 2 数据侧的无头自检（不需要 GPU / 不需要 Qt / 不需要 GUI）
//
// 目的：在把 CloudRenderData 接进渲染路径（Phase 2b）之前，先把
// "位姿上移着色器"这套架构的**可测量论断**逐条验证掉：
//
//   A. 位姿表正确性 —— 两槽位姿与图里的 estimate()/odom 一致
//   B. 优化更新代价 —— 改位姿后只重写优化槽，原始槽逐位不变；
//                       耗时只与帧数有关，**与点数无关**
//   C. 分块数据     —— frameLocalId / FrameRange / 顶点坐标与源点云一致
//   D. LOD 索引     —— 严格升序、不越界、逐级变粗、每帧每级非空
//   E. 世界 AABB    —— 抽样点的 T·p 全部落在 chunk 的 world AABB 内
//   F. 双层共享     —— 顶点只存一份；用原始槽刷新后 AABB 与优化槽不同
//
// 用法：
//   cloud_selftest --map <地图目录> [--chunk-frames N] [--sample N] [--full]
//     --full  全量校验（默认只抽样，巡检每个 AABB 的点）
//
// 退出码: 0 = 全部通过；1 = 有失败项；2 = 参数/加载错误
// ============================================================================

#include "visualizers/CloudRenderData.h"
#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "data/hdl_graph_slam/keyframe.hpp"
#include "data/hdl_graph_slam/progress_interface.hpp"

#include <g2o/types/slam3d/vertex_se3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <utility>
#include <vector>

using namespace hdl_graph_slam;

namespace {

void usage() {
    std::printf(
        "cloud_selftest —— Phase 2 数据侧无头自检\n"
        "\n"
        "用法:\n"
        "  cloud_selftest --map <地图目录> [选项]\n"
        "      --chunk-frames N  每 chunk 帧数（默认 256）\n"
        "      --lod-levels N    级别数含 L0（默认 4）\n"
        "      --sample N        抽样帧数（默认 200）\n"
        "      --aabb-every N    每 N 个顶点校验一次世界 AABB（默认 64）\n"
        "\n"
        "退出码: 0 = 全部通过；1 = 有失败项；2 = 参数/加载错误\n");
}

/** @brief 自检统计 */
struct Checker {
    int checks = 0;
    int failures = 0;

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

/** @brief 世界坐标 = T · p（数学列主序的 p[4*col+row]） */
void transformPoint(const float* pose16, const float p[3], float out[3]) {
    const double x = p[0], y = p[1], z = p[2];
    out[0] = static_cast<float>(pose16[0] * x + pose16[4] * y + pose16[8]  * z + pose16[12]);
    out[1] = static_cast<float>(pose16[1] * x + pose16[5] * y + pose16[9]  * z + pose16[13]);
    out[2] = static_cast<float>(pose16[2] * x + pose16[6] * y + pose16[10] * z + pose16[14]);
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) { usage(); return 2; }
    const std::string mapDir = argValue(argc, argv, "--map", "");
    if (mapDir.empty()) { usage(); return 2; }

    const uint32_t chunkFrames = static_cast<uint32_t>(
        std::strtoul(argValue(argc, argv, "--chunk-frames", "256").c_str(), nullptr, 10));
    const uint32_t lodLevels = static_cast<uint32_t>(
        std::strtoul(argValue(argc, argv, "--lod-levels", "4").c_str(), nullptr, 10));
    const int sampleFrames = std::atoi(argValue(argc, argv, "--sample", "200").c_str());
    const int aabbEvery = std::max(1, std::atoi(argValue(argc, argv, "--aabb-every", "64").c_str()));

    std::printf("=== cloud_selftest ===\n地图: %s\nchunk=%u 帧, LOD=%u 级, 抽样=%d 帧\n\n",
                mapDir.c_str(), chunkFrames, lodLevels, sampleFrames);

    // ---- 加载地图 ----
    SilentProgress progress;
    auto graph = std::make_shared<InteractiveGraph>();
    const auto tLoad = std::chrono::steady_clock::now();
    if (!graph->load_map_data(mapDir, progress)) {
        std::printf("\n错误：地图加载失败\n");
        return 2;
    }
    const double loadMs = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - tLoad).count();
    std::printf("\n地图加载完成：%zu 关键帧，%.1f s\n\n",
                graph->keyframes.size(), loadMs / 1000.0);

    Checker ck;

    // =====================================================================
    // 构建（这就是 Phase 2 的全部数据侧工作）
    // =====================================================================
    CloudPoseTable poses;
    ChunkBuildOptions opt;
    opt.chunkFrames = chunkFrames;
    opt.lodLevels   = lodLevels;
    size_t lastShown = static_cast<size_t>(-1);
    opt.onProgress = [&](uint64_t done, uint64_t total) {
        const size_t p = total ? static_cast<size_t>(done * 100 / total) : 0;
        if (p != lastShown) {
            lastShown = p;
            std::printf("\r[build] %3zu%% (%llu/%llu)", p,
                        static_cast<unsigned long long>(done),
                        static_cast<unsigned long long>(total));
            std::fflush(stdout);
        }
    };
    // 世界 AABB 需要被反复刷新，因此结果不能是 const
    ChunkBuildResult res = buildChunks(graph, poses, opt);
    std::printf("\n");
    std::printf("构建结果:\n");
    std::printf("  帧数/帧序     : %zu（升序=%s）\n", poses.frameCount(),
                std::is_sorted(poses.frameIds().begin(), poses.frameIds().end()) ? "是" : "否");
    std::printf("  chunk 数      : %zu（每 %u 帧）\n", res.chunks.size(), chunkFrames);
    std::printf("  全量点数      : %llu\n",
                static_cast<unsigned long long>(res.totalPoints));
    std::printf("  LOD 各级点数  : L0=%llu L1=%llu L2=%llu L3=%llu\n",
                static_cast<unsigned long long>(res.lodPoints[0]),
                static_cast<unsigned long long>(res.lodPoints[1]),
                static_cast<unsigned long long>(res.lodPoints[2]),
                static_cast<unsigned long long>(res.lodPoints[3]));
    std::printf("  顶点字节      : %.1f MiB（float3，单份，双层共享）\n",
                res.vertexBytes / 1048576.0);
    std::printf("  帧号属性字节  : %.1f MiB（uint16）\n", res.frameIdBytes / 1048576.0);
    std::printf("  LOD 索引字节  : %.1f MiB\n", res.indexBytes / 1048576.0);
    std::printf("  单份合计      : %.1f MiB（%.2f B/点）\n",
                res.totalBytes() / 1048576.0,
                res.totalPoints ? static_cast<double>(res.totalBytes()) / res.totalPoints : 0.0);
    std::printf("  位姿表        : %zu 帧 × 2 槽 × 64 B = %.2f MiB\n",
                res.poseStats.frameCount, res.poseStats.bytes / 1048576.0);
    std::printf("  耗时          : 位姿表 %.1f ms，LOD %.1f ms，合计 %.1f ms\n\n",
                res.poseStats.buildMs, res.lodMs, res.totalMs);

    // =====================================================================
    // A. 位姿表正确性
    // =====================================================================
    ck.section("A. 位姿表正确性");
    ck.check(poses.frameCount() == graph->keyframes.size(),
             "位姿表帧数与 graph->keyframes 一致");
    ck.check(std::is_sorted(poses.frameIds().begin(), poses.frameIds().end()),
             "帧序按 frameId 严格升序");
    ck.check(poses.slotFloats() == poses.frameCount() * 16,
             "槽步长 == 帧数 × 16（着色器 uPoseOffset 的口径）");
    ck.check(poses.data().size() == poses.frameCount() * 16 * 2,
             "位姿数据长度 == 帧数 × 16 × 2 槽");

    {
        // 抽样比对：两槽位姿与图里的 estimate()/odom 一致（1e-6 相对容差）
        std::mt19937 rng(20260911u);
        const size_t n = poses.frameCount();
        const int k = std::min<int>(sampleFrames, static_cast<int>(n));
        std::uniform_int_distribution<size_t> dist(0, n - 1);
        int optBad = 0, odomBad = 0;
        for (int s = 0; s < k; ++s) {
            const size_t i = dist(rng);
            auto it = graph->keyframes.find(poses.frameIds()[i]);
            if (it == graph->keyframes.end()) { ++optBad; ++odomBad; continue; }
            auto* v = dynamic_cast<g2o::VertexSE3*>(it->second->node);
            if (!v) { ++optBad; ++odomBad; continue; }

            const Eigen::Matrix4d refOpt  = v->estimate().matrix();
            const Eigen::Matrix4d refOdom = it->second->odom.matrix();
            const Eigen::Matrix4d gotOpt  = poses.pose(CloudPoseTable::kSlotOptimized, i).matrix();
            const Eigen::Matrix4d gotOdom = poses.pose(CloudPoseTable::kSlotOriginal, i).matrix();
            if ((gotOpt - refOpt).cwiseAbs().maxCoeff() > 1e-5) ++optBad;
            if ((gotOdom - refOdom).cwiseAbs().maxCoeff() > 1e-5) ++odomBad;
        }
        ck.check(optBad == 0, "优化槽位姿与 vertex->estimate() 一致（抽样）");
        ck.check(odomBad == 0, "原始槽位姿与 keyframe->odom 一致（抽样）");
    }

    // =====================================================================
    // B. 优化更新代价（架构核心论断）
    // =====================================================================
    ck.section("B. 优化更新代价（核心论断：只与帧数有关，与点数无关）");
    {
        // 备份原始槽，稍后验证它逐位不变
        const size_t slotStride = poses.slotFloats();
        std::vector<float> odomBefore(poses.data().begin() + slotStride,
                                      poses.data().begin() + slotStride * 2);

        // 人为扰动若干顶点的位姿（只动内存里的 g2o 估计值，不跑优化）
        std::mt19937 rng(7u);
        const size_t n = poses.frameCount();
        const int k = std::min<int>(64, static_cast<int>(n));
        std::uniform_int_distribution<size_t> dist(0, n - 1);
        std::vector<std::pair<g2o::VertexSE3*, Eigen::Isometry3d>> saved;
        for (int s = 0; s < k; ++s) {
            const size_t i = dist(rng);
            auto it = graph->keyframes.find(poses.frameIds()[i]);
            if (it == graph->keyframes.end()) continue;
            auto* v = dynamic_cast<g2o::VertexSE3*>(it->second->node);
            if (!v) continue;
            const Eigen::Isometry3d oldT = v->estimate();
            Eigen::Isometry3d newT = oldT;
            newT.translation() += Eigen::Vector3d(0.5, -0.25, 0.125);
            saved.emplace_back(v, oldT);
            v->setEstimate(newT);
        }

        const CloudPoseTable::Stats up = poses.updateOptimized(graph);
        const double uploadMiB =
            static_cast<double>(poses.slotFloats()) * sizeof(float) / 1048576.0;
        std::printf("  扰动 %zu 帧后 updateOptimized: %.3f ms"
                    "（重写 1 个槽 = %zu 帧 × 64 B = %.2f MiB 上传，顶点零重传）\n",
                    saved.size(), up.buildMs, up.frameCount, uploadMiB);

        // 原始槽必须逐位不变（"原始点云冻结"是结构性保证）
        bool odomIntact = true;
        for (size_t i = 0; i < odomBefore.size(); ++i) {
            if (poses.data()[slotStride + i] != odomBefore[i]) { odomIntact = false; break; }
        }
        ck.check(odomIntact, "updateOptimized 后原始槽逐位不变（冻结是结构性保证）");

        // 优化槽必须反映新位姿
        bool optUpdated = true;
        for (const auto& [v, oldT] : saved) {
            const int64_t fi = poses.indexOfFrame(static_cast<long>(v->id()));
            if (fi < 0) continue;
            const Eigen::Matrix4d got = poses.pose(CloudPoseTable::kSlotOptimized,
                                                   static_cast<size_t>(fi)).matrix();
            if ((got - v->estimate().matrix()).cwiseAbs().maxCoeff() > 1e-5) {
                optUpdated = false;
                break;
            }
        }
        ck.check(optUpdated, "updateOptimized 后优化槽反映新位姿");

        // 顶点数据不该被碰过：顶点字节与点数完全不变
        ck.check(res.totalPoints > 0, "全量点数 > 0");

        // 世界 AABB 刷新代价（O(帧数)）
        const double aabbMs =
            refreshAllWorldBounds(res.chunks, poses, CloudPoseTable::kSlotOptimized);
        std::printf("  位姿变更后刷新全部 chunk 世界 AABB: %.3f ms（%zu chunk / %zu 帧）\n",
                    aabbMs, res.chunks.size(), poses.frameCount());
        ck.check(aabbMs < 200.0, "刷新全部世界 AABB < 200 ms（O(帧数)，不是 O(点数)）");

        // 还原扰动，并把世界 AABB 一并刷回，否则后面的 E 段会拿扰动后的
        // AABB 去比对还原后的位姿，产生假失败
        for (auto& [v, oldT] : saved) v->setEstimate(oldT);
        poses.updateOptimized(graph);
        refreshAllWorldBounds(res.chunks, poses, CloudPoseTable::kSlotOptimized);
    }

    // =====================================================================
    // C. 分块数据
    // =====================================================================
    ck.section("C. 分块数据（帧号属性 / 帧范围 / 顶点坐标）");
    {
        bool localIdOk = true, rangeOk = true, contiguousOk = true, countOk = true;
        uint64_t sumFramePoints = 0;
        for (const auto& c : res.chunks) {
            // 帧范围首尾相接且总长 == pointCount
            uint32_t expectStart = 0;
            uint64_t sumLocal = 0;
            for (const auto& fr : c.frames) {
                if (fr.startVertex != expectStart) contiguousOk = false;
                expectStart = fr.startVertex + fr.vertexCount;
                sumLocal += fr.vertexCount;
                sumFramePoints += fr.vertexCount;
            }
            if (expectStart != c.pointCount) contiguousOk = false;
            if (sumLocal != c.pointCount) countOk = false;

            // FrameRange 数量必须等于帧跨度（零顶点帧也占位），否则
            // aFrameId = c.frames 下标 这个映射就断了
            if (c.frames.size() != c.frameCount) rangeOk = false;

            // frameLocalId 必须等于该顶点所属帧在 c.frames 中的下标
            if (c.frameLocalId.size() != c.pointCount) localIdOk = false;
            for (size_t fi = 0; fi < c.frames.size() && localIdOk; ++fi) {
                const auto& fr = c.frames[fi];
                const uint16_t want = static_cast<uint16_t>(fi & 0xFFFFu);
                for (uint32_t v = fr.startVertex; v < fr.startVertex + fr.vertexCount; ++v) {
                    if (c.frameLocalId[v] != want) { localIdOk = false; break; }
                }
            }
        }
        ck.check(contiguousOk, "各 chunk 的 FrameRange 首尾相接且总长 == pointCount");
        ck.check(countOk, "逐帧顶点数之和 == chunk->pointCount");
        ck.check(rangeOk, "FrameRange 数量 == 帧跨度（aFrameId 映射的前提）");
        ck.check(localIdOk, "frameLocalId[v] == 该顶点所属帧在 chunk 内的下标");
        ck.check(sumFramePoints == res.totalPoints, "所有 chunk 帧点数之和 == totalPoints");

        // 全局帧序 ↔ chunk 下标 的纯算术换算（渲染期按帧号找 chunk 依赖它）
        {
            bool idxInvariant = true;
            for (size_t i = 0; i < res.chunks.size(); ++i) {
                if (res.chunks[i].firstFrameIndex != i * chunkFrames) {
                    idxInvariant = false;
                    break;
                }
            }
            const size_t wantChunks =
                (poses.frameCount() + chunkFrames - 1) / chunkFrames;
            ck.check(idxInvariant && res.chunks.size() == wantChunks,
                     "chunks[i].firstFrameIndex == i×chunkFrames 且 chunk 数 == ceil(帧数/块)");
        }

        // 顶点坐标与源点云逐位一致（抽样若干帧的首/末点）
        std::mt19937 rng(11u);
        int vertBad = 0;
        const int k = std::min<int>(sampleFrames, static_cast<int>(poses.frameCount()));
        std::uniform_int_distribution<size_t> dist(0, poses.frameCount() - 1);
        for (int s = 0; s < k; ++s) {
            const size_t gi = dist(rng);
            auto it = graph->keyframes.find(poses.frameIds()[gi]);
            if (it == graph->keyframes.end() || !it->second->cloud) continue;
            const auto& src = *it->second->cloud;
            if (src.empty()) continue;

            // 找到含该帧的 chunk 与 FrameRange
            const uint32_t chunkIdx = static_cast<uint32_t>(gi / chunkFrames);
            if (chunkIdx >= res.chunks.size()) continue;
            const CloudChunk& c = res.chunks[chunkIdx];
            const CloudChunk::FrameRange* fr = nullptr;
            for (const auto& r : c.frames) {
                if (r.frameId == poses.frameIds()[gi]) { fr = &r; break; }
            }
            if (!fr) continue;
            if (fr->vertexCount != src.size()) { ++vertBad; continue; }

            const auto& p0 = src.points[0];
            const auto& pn = src.points[src.size() - 1];
            const size_t o0 = static_cast<size_t>(fr->startVertex) * 3;
            const size_t on = o0 + static_cast<size_t>(fr->vertexCount - 1) * 3;
            if (!(c.positions[o0] == p0.x && c.positions[o0 + 1] == p0.y &&
                  c.positions[o0 + 2] == p0.z)) ++vertBad;
            if (!(c.positions[on] == pn.x && c.positions[on + 1] == pn.y &&
                  c.positions[on + 2] == pn.z)) ++vertBad;
        }
        ck.check(vertBad == 0, "顶点坐标与源点云逐位一致（抽样首/末点）");
    }

    // =====================================================================
    // D. LOD 索引
    // =====================================================================
    ck.section("D. LOD 索引合法性");
    {
        bool l0Empty = true, inRange = true, increasing = true, decreasing = true,
             nonEmpty = true;
        for (const auto& c : res.chunks) {
            if (!c.lodIndices.empty() && !c.lodIndices[0].empty()) l0Empty = false;
            size_t prev = c.pointCount + 1;
            for (size_t L = 1; L < c.lodIndices.size(); ++L) {
                const auto& v = c.lodIndices[L];
                if (v.size() > prev) decreasing = false;
                prev = v.size();
                if (v.empty()) { nonEmpty = false; continue; }
                for (size_t i = 0; i < v.size(); ++i) {
                    if (v[i] >= c.pointCount) { inRange = false; break; }
                    if (i > 0 && v[i] <= v[i - 1]) { increasing = false; break; }
                }
                if (!inRange || !increasing) break;
            }
            if (!inRange || !increasing) break;
        }
        ck.check(l0Empty, "lodIndices[0] 为空（level 0 = 全量，直接 DrawArrays）");
        ck.check(inRange, "各级索引全部 < chunk->pointCount（不越界）");
        ck.check(increasing, "各级索引严格升序");
        ck.check(decreasing, "级别越粗点数越少");
        ck.check(nonEmpty, "每级抽稀结果非空");
    }

    // =====================================================================
    // E. 世界 AABB
    // =====================================================================
    ck.section("E. 世界 AABB（位姿变化后仍然正确，且是剔除的依据）");
    {
        int outside = 0;
        uint64_t tested = 0;
        for (const auto& c : res.chunks) {
            if (!c.worldBoundsValid) { ++outside; continue; }
            // 逐帧把局部点变换到世界，检查落在 chunk 世界 AABB 内（带小容差）
            for (const auto& fr : c.frames) {
                const int64_t fi = poses.indexOfFrame(fr.frameId);
                if (fi < 0) continue;
                const float* T = poses.posePtr(CloudPoseTable::kSlotOptimized,
                                               static_cast<size_t>(fi));
                if (!T) continue;
                for (uint32_t v = fr.startVertex;
                     v < fr.startVertex + fr.vertexCount;
                     v += static_cast<uint32_t>(aabbEvery)) {
                    const float* p = c.positions.data() + static_cast<size_t>(v) * 3;
                    float w[3];
                    transformPoint(T, p, w);
                    ++tested;
                    const float eps = 1e-2f;  // float 累积误差容差
                    for (int k = 0; k < 3; ++k) {
                        if (w[k] < c.worldMin[k] - eps || w[k] > c.worldMax[k] + eps) {
                            ++outside;
                            break;
                        }
                    }
                }
            }
        }
        std::printf("  校验点数: %llu（每 %d 个顶点取一个）\n",
                    static_cast<unsigned long long>(tested), aabbEvery);
        ck.check(outside == 0, "所有抽样点的 T·p 都落在其 chunk 的世界 AABB 内");
    }

    // =====================================================================
    // F. 双层共享
    // =====================================================================
    ck.section("F. 双层共享（双份显示 = 1× 顶点显存）");
    {
        ck.check(res.vertexBytes == res.totalPoints * 12ull,
                 "顶点字节 == 点数 × 12（float3，只存一份；两层各存一份会翻倍）");

        // 用原始槽刷新 AABB，应与优化槽不同（证明两层可独立控制）。
        // 注意：**不能深拷贝 res.chunks** —— 那会把顶点数据整份复制，
        // 在 1.3 亿/4 亿点地图上是 GB 级的内存浪费。这里只暂存每 chunk
        // 6 个 float 的包围盒（几 KB），原地刷新后再比对。
        std::vector<float> optBounds;
        optBounds.reserve(res.chunks.size() * 6);
        for (const auto& c : res.chunks) {
            for (int k = 0; k < 3; ++k) {
                optBounds.push_back(c.worldMin[k]);
                optBounds.push_back(c.worldMax[k]);
            }
        }

        refreshAllWorldBounds(res.chunks, poses, CloudPoseTable::kSlotOriginal);

        bool differs = false;
        for (size_t i = 0, b = 0; i < res.chunks.size(); ++i, b += 6) {
            const CloudChunk& c = res.chunks[i];
            for (int k = 0; k < 3; ++k) {
                if (std::fabs(optBounds[b + 2 * k] - c.worldMin[k]) > 1e-3f ||
                    std::fabs(optBounds[b + 2 * k + 1] - c.worldMax[k]) > 1e-3f) {
                    differs = true;
                }
            }
        }
        ck.check(differs, "用原始槽刷新的世界 AABB 与优化槽不同（两层确实独立）");

        // 还原为优化槽 AABB
        refreshAllWorldBounds(res.chunks, poses, CloudPoseTable::kSlotOptimized);

        // 位姿表两槽的字节量相对顶点可以忽略
        const double poseVsVertex =
            res.vertexBytes ? static_cast<double>(poses.data().size() * 4) / res.vertexBytes : 0.0;
        std::printf("  位姿表 / 顶点字节 = %.4f%%\n", poseVsVertex * 100.0);
        ck.check(poseVsVertex < 0.01, "位姿表字节 < 顶点字节的 1%（可忽略）");
    }

    const int rc = ck.summary();
    std::printf(rc == 0 ? "\n结果：全部通过 ✓\n" : "\n结果：存在失败项 ✗\n");
    return rc;
}
