// ============================================================================
// isf_pack.cpp
// .isf 打包 / 校验 命令行工具（Phase 1 的独立验收入口）
//
// 用法：
//   isf_pack pack  <mapDir> [--out FILE] [--frames-per-page N] [--chunk-frames N]
//                           [--lod-levels N] [--no-preview] [--sample N]
//   isf_pack check <isfFile> [--sample N]
//
// pack  = 加载地图 → 打包 .isf/.isf.idx → 立即做全部校验（含与源点云逐位往返）
// check = 只做结构校验（不需要地图），用于事后复核文件是否完好
//
// 校验覆盖设计文档 §6 Phase 1 的验收标准：
//   · 帧 → 点范围映射与 cloud->size() 一致（抽样 --sample 帧）
//   · 坐标逐位往返（首点、末点都比）
//   · 页/chunk 归属与帧序号推导一致、页字节区间连续不重叠
//   · LOD 索引合法（越界检查、严格升序、逐级变粗）
// ============================================================================

#include "pointcloud/isf/IsfReader.h"
#include "pointcloud/isf/IsfWriter.h"
#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "data/hdl_graph_slam/keyframe.hpp"
#include "data/hdl_graph_slam/progress_interface.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <memory>
#include <random>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace hdl_graph_slam;

namespace {

void usage() {
    std::printf(
        "isf_pack —— .isf 点云帧存储打包/校验工具\n"
        "\n"
        "用法:\n"
        "  isf_pack pack  <mapDir> [选项]\n"
        "      --out FILE           输出 .isf 路径（默认 <mapDir>/cloud.isf）\n"
        "      --frames-per-page N  每页帧数（默认 %u）\n"
        "      --chunk-frames N     每 chunk 帧数（默认 %u）\n"
        "      --lod-levels N       生成级别数含 L0，1..%u（默认 %u）\n"
        "      --no-preview         不生成全局粗预览块\n"
        "      --sample N           抽样校验帧数（默认 100）\n"
        "  isf_pack check <isfFile> [--sample N]\n"
        "      只做结构校验，不需要原始地图\n"
        "\n"
        "退出码: 0 = 全部校验通过；1 = 有校验失败；2 = 参数/加载错误\n",
        isf::kDefaultFramesPerPage, isf::kDefaultChunkFrames, isf::kMaxLodLevels,
        isf::kMaxLodLevels);
}

/** @brief 控制台进度（load_map_data 会按帧回调） */
struct ConsoleProgress : ProgressInterface {
    int max = 0;
    int cur = 0;
    int lastShown = -1;

    void set_title(const std::string& title) override {
        std::printf("[load] %s\n", title.c_str());
        std::fflush(stdout);
    }
    void set_maximum(int m) override { max = m; }
    void set_current(int c) override { cur = c; report(); }
    void increment() override { ++cur; report(); }

    void report() {
        if (max <= 0) return;
        const int p = cur * 100 / max;
        if (p == lastShown) return;
        lastShown = p;
        std::printf("\r[load] %3d%% (%d/%d)", p, cur, max);
        std::fflush(stdout);
        if (p >= 100) std::printf("\n");
    }
};

/** @brief 校验器：只打印失败项，最后给汇总 */
struct Checker {
    int checks = 0;
    int failures = 0;
    bool verbose = false;

    void check(bool cond, const std::string& what) {
        ++checks;
        if (!cond) {
            ++failures;
            std::printf("  [FAIL] %s\n", what.c_str());
            std::fflush(stdout);
        } else if (verbose) {
            std::printf("  [ ok ] %s\n", what.c_str());
        }
    }
    void section(const char* name) {
        std::printf("-- %s\n", name);
        std::fflush(stdout);
    }
    int summary() const {
        std::printf("\n校验项 %d，失败 %d\n", checks, failures);
        return failures == 0 ? 0 : 1;
    }
};

/** @brief 结构一致性 + LOD 合法性（不依赖源地图） */
void checkStructure(const isf::IsfReader& r, Checker& ck, int sampleFrames) {
    const isf::FileHeader& h = r.header();
    const auto& frames = r.frames();
    const auto& pages = r.pages();

    ck.section("文件头与表规模");
    ck.check(h.l0Stride == isf::kL0Stride, "l0Stride == 12");
    ck.check(h.previewStride == isf::kPreviewStride, "previewStride == 16");
    ck.check(frames.size() == h.frameCount, "帧表长度 == header.frameCount");
    ck.check(frames.size() == r.indexHeader().frameCount,
             "帧表长度 == idxHeader.frameCount");
    ck.check(pages.size() == h.pageCount, "页表长度 == header.pageCount");
    ck.check(pages.size() == r.indexHeader().pageCount,
             "页表长度 == idxHeader.pageCount");

    uint64_t sumPoints = 0;
    for (const auto& f : frames) sumPoints += f.pointCount;
    ck.check(sumPoints == h.totalPoints, "各帧 pointCount 之和 == header.totalPoints");

    ck.section("帧表顺序与页/chunk 归属（与位姿无关的推导必须一致）");
    bool sorted = true;
    for (size_t i = 1; i < frames.size(); ++i) {
        if (frames[i].frameId <= frames[i - 1].frameId) { sorted = false; break; }
    }
    ck.check(sorted, "帧表按 frameId 严格递增（二分查找的前提）");

    bool pageOk = true, chunkOk = true;
    for (size_t i = 0; i < frames.size(); ++i) {
        const auto fi = static_cast<uint64_t>(i);
        if (frames[i].pageId != isf::pageIdOf(fi, h.framesPerPage)) pageOk = false;
        if (frames[i].chunkId != isf::chunkIdOf(fi, h.framesPerPage, h.chunkFrames))
            chunkOk = false;
        if (frames[i].chunkLocalFrame != isf::chunkLocalFrameOf(fi, h.chunkFrames))
            chunkOk = false;
    }
    ck.check(pageOk, "每帧 pageId == frameIndex / framesPerPage");
    ck.check(chunkOk, "每帧 chunkId/chunkLocalFrame 与帧序号推导一致");

    ck.section("页表连续性与字节区间");
    bool cover = !pages.empty() && pages.front().firstFrameIndex == 0;
    uint32_t expect = 0;
    for (const auto& p : pages) {
        if (p.firstFrameIndex != expect) cover = false;
        expect = p.firstFrameIndex + p.frameCount;
    }
    if (expect != frames.size()) cover = false;
    ck.check(cover, "页表连续覆盖全部帧且无空洞");

    bool nonOverlap = true;
    for (size_t i = 1; i < pages.size(); ++i) {
        if (pages[i].fileOffset < pages[i - 1].fileOffset + pages[i - 1].fileBytes)
            nonOverlap = false;
    }
    ck.check(nonOverlap, "页字节区间互不重叠且递增");

    const uint64_t fileEnd = static_cast<uint64_t>(h.headerBytes) + h.dataBytes;
    bool l0InRange = true;
    const uint64_t frameBytes = static_cast<uint64_t>(h.headerBytes);
    for (const auto& f : frames) {
        const uint64_t need = static_cast<uint64_t>(f.pointCount) * h.l0Stride;
        if (f.l0Offset < frameBytes || f.l0Offset + need > fileEnd) l0InRange = false;
    }
    ck.check(l0InRange, "所有帧的 L0 块都落在文件数据区内");

    ck.section("帧 ID 反查");
    bool lookup = true;
    for (size_t i = 0; i < frames.size(); ++i) {
        if (r.indexOfFrame(frames[i].frameId) != static_cast<int64_t>(i)) {
            lookup = false;
            break;
        }
    }
    ck.check(lookup, "indexOfFrame(frameId) 能找回每个帧的下标");

    // —— 抽样做 LOD 合法性检查（全量做会很慢） ——
    ck.section("LOD 索引合法性（抽样）");
    std::mt19937 rng(20260911u);  // 固定种子：结果可复现
    std::vector<uint32_t> lod;
    if (!frames.empty()) {
        const int n = std::min<int>(sampleFrames, static_cast<int>(frames.size()));
        std::uniform_int_distribution<size_t> dist(0, frames.size() - 1);
        bool idxInRange = true, increasing = true, monotonic = true, nonEmpty = true;
        for (int s = 0; s < n; ++s) {
            const size_t fi = dist(rng);
            const auto& fr = frames[fi];
            uint32_t prevCount = fr.pointCount;
            for (uint32_t L = 1; L < fr.lodCount; ++L) {
                uint32_t cnt = 0;
                if (!r.readFrameLodIndices(fi, L, lod, cnt)) { idxInRange = false; break; }
                if (cnt == 0) { nonEmpty = false; break; }
                if (cnt > prevCount) monotonic = false;
                prevCount = cnt;
                for (uint32_t k = 0; k < cnt; ++k) {
                    if (lod[k] >= fr.pointCount) { idxInRange = false; break; }
                    if (k > 0 && lod[k] <= lod[k - 1]) { increasing = false; break; }
                }
                if (!idxInRange || !increasing) break;
            }
        }
        ck.check(idxInRange, "LOD 索引全部 < 该帧 pointCount（不越界）");
        ck.check(increasing, "LOD 索引严格升序（抽稀保序）");
        ck.check(monotonic, "级别越粗点数越少");
        ck.check(nonEmpty, "每级抽稀结果非空");
    }
}

/** @brief 与源地图逐位往返比对（仅在 pack 之后可用） */
void checkAgainstGraph(const isf::IsfReader& r,
                       const std::shared_ptr<InteractiveGraph>& graph,
                       Checker& ck, int sampleFrames) {
    ck.section("与源地图逐位往返（抽样）");
    const auto& frames = r.frames();

    std::mt19937 rng(20260911u);
    const int n = std::min<int>(sampleFrames, static_cast<int>(frames.size()));
    std::uniform_int_distribution<size_t> dist(0, frames.size() - 1);

    int countMismatch = 0, firstMismatch = 0, lastMismatch = 0, aabbMismatch = 0;
    int cloudMissing = 0;
    std::vector<float> buf;

    for (int s = 0; s < n; ++s) {
        const size_t fi = dist(rng);
        const auto& fr = frames[fi];
        // 地图的帧字典键是 long，而 FrameRecord.frameId 是 int64_t（跨平台统一），
        // 显式转换以免 MSVC 在 /W3 下报窄化警告
        auto it = graph->keyframes.find(static_cast<long>(fr.frameId));
        if (it == graph->keyframes.end() || !it->second->cloud) { ++cloudMissing; continue; }
        const auto& src = *it->second->cloud;

        if (src.size() != fr.pointCount) { ++countMismatch; continue; }
        if (src.empty()) continue;  // 空帧不参与坐标比对（pack 阶段已过滤，防御性判断）
        if (!r.readFramePositions(fi, buf)) { ++firstMismatch; continue; }
        if (buf.size() != src.size() * 3) { ++firstMismatch; continue; }

        // 首点
        const auto& p0 = src.points[0];
        if (!(buf[0] == p0.x && buf[1] == p0.y && buf[2] == p0.z)) ++firstMismatch;
        // 末点（验证块长度正确、没有错位）
        const size_t m = src.size() - 1;
        const auto& pn = src.points[m];
        if (!(buf[3 * m + 0] == pn.x && buf[3 * m + 1] == pn.y &&
              buf[3 * m + 2] == pn.z)) {
            ++lastMismatch;
        }
        // AABB 一致性（用与之相同的浮点顺序累加，保证可比）
        float lo[3] = {src.points[0].x, src.points[0].y, src.points[0].z};
        float hi[3] = {lo[0], lo[1], lo[2]};
        for (const auto& p : src.points) {
            const float v[3] = {p.x, p.y, p.z};
            for (int k = 0; k < 3; ++k) {
                if (v[k] < lo[k]) lo[k] = v[k];
                if (v[k] > hi[k]) hi[k] = v[k];
            }
        }
        if (!(fr.localAABB[0] == lo[0] && fr.localAABB[1] == lo[1] &&
              fr.localAABB[2] == lo[2] && fr.localAABB[3] == hi[0] &&
              fr.localAABB[4] == hi[1] && fr.localAABB[5] == hi[2])) {
            ++aabbMismatch;
        }
    }

    ck.check(cloudMissing == 0, "抽样的帧都能在地图中找到点云");
    ck.check(countMismatch == 0, "FrameRecord.pointCount == cloud->size()");
    ck.check(firstMismatch == 0, "首点坐标逐位一致（buf 长度也对）");
    ck.check(lastMismatch == 0, "末点坐标逐位一致（块长度与偏移无误）");
    ck.check(aabbMismatch == 0, "localAABB 与源点云算出的一致");
}

std::string argValue(int argc, char** argv, const char* name, const std::string& def) {
    for (int i = 0; i < argc; ++i) {
        if (std::strcmp(argv[i], name) == 0 && i + 1 < argc) return argv[i + 1];
    }
    return def;
}
bool argFlag(int argc, char** argv, const char* name) {
    for (int i = 0; i < argc; ++i) {
        if (std::strcmp(argv[i], name) == 0) return true;
    }
    return false;
}

int doPack(int argc, char** argv) {
    const std::string mapDir = argv[2];
    if (!fs::exists(fs::path(mapDir) / "graph.g2o")) {
        std::printf("错误：%s 下没有 graph.g2o，不是合法的地图目录\n", mapDir.c_str());
        return 2;
    }

    isf::WriteOptions opt;
    opt.framesPerPage = static_cast<uint32_t>(
        std::strtoul(argValue(argc, argv, "--frames-per-page",
                              std::to_string(isf::kDefaultFramesPerPage)).c_str(), nullptr, 10));
    opt.chunkFrames = static_cast<uint32_t>(
        std::strtoul(argValue(argc, argv, "--chunk-frames",
                              std::to_string(isf::kDefaultChunkFrames)).c_str(), nullptr, 10));
    opt.lodLevels = static_cast<uint32_t>(
        std::strtoul(argValue(argc, argv, "--lod-levels",
                              std::to_string(isf::kMaxLodLevels)).c_str(), nullptr, 10));
    opt.buildPreview = !argFlag(argc, argv, "--no-preview");
    const int sampleFrames = std::atoi(argValue(argc, argv, "--sample", "100").c_str());

    std::string outPath = argValue(argc, argv, "--out", "");
    if (outPath.empty()) outPath = (fs::path(mapDir) / "cloud.isf").string();

    std::printf("=== isf_pack pack ===\n");
    std::printf("地图目录 : %s\n", mapDir.c_str());
    std::printf("输出     : %s (+ .idx)\n", outPath.c_str());
    std::printf("页/块    : %u 帧/页, %u 帧/chunk, %u 级 LOD, 预览=%s\n\n",
                opt.framesPerPage, opt.chunkFrames, opt.lodLevels,
                opt.buildPreview ? "on" : "off");

    // —— 1) 加载地图 ——
    ConsoleProgress progress;
    auto graph = std::make_shared<InteractiveGraph>();
    const auto tLoad = std::chrono::steady_clock::now();
    if (!graph->load_map_data(mapDir, progress)) {
        std::printf("\n错误：地图加载失败\n");
        return 2;
    }
    const double loadMs = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - tLoad).count();
    std::printf("地图加载完成：%zu 个关键帧，耗时 %.1f s\n\n",
                graph->keyframes.size(), loadMs / 1000.0);

    // —— 2) 打包 ——
    std::printf("=== 打包 ===\n");
    size_t lastShown = static_cast<size_t>(-1);
    opt.onProgress = [&](uint64_t done, uint64_t total) {
        const size_t p = total ? static_cast<size_t>(done * 100 / total) : 0;
        if (p != lastShown) {
            lastShown = p;
            std::printf("\r[pack] %3zu%% (%llu/%llu)", p,
                        static_cast<unsigned long long>(done),
                        static_cast<unsigned long long>(total));
            std::fflush(stdout);
        }
    };
    const isf::WriteResult wr = isf::write(graph, outPath, opt);
    std::printf("\n");
    if (!wr.ok) {
        std::printf("错误：打包失败 —— %s\n", wr.error.c_str());
        return 2;
    }

    std::printf("帧数        : %llu（%llu 页）\n",
                static_cast<unsigned long long>(wr.frameCount),
                static_cast<unsigned long long>(wr.pageCount));
    std::printf("全量点数    : %llu\n",
                static_cast<unsigned long long>(wr.totalPoints));
    std::printf("LOD 各级点数: L0=%llu  L1=%llu  L2=%llu  L3=%llu\n",
                static_cast<unsigned long long>(wr.lodPoints[0]),
                static_cast<unsigned long long>(wr.lodPoints[1]),
                static_cast<unsigned long long>(wr.lodPoints[2]),
                static_cast<unsigned long long>(wr.lodPoints[3]));
    std::printf("预览点数    : %llu\n",
                static_cast<unsigned long long>(wr.previewPoints));
    std::printf("文件大小    : .isf %.2f MiB（%.2f B/点） + .idx %.2f MiB\n",
                wr.isfBytes / 1048576.0,
                wr.totalPoints ? static_cast<double>(wr.isfBytes) / wr.totalPoints : 0.0,
                wr.idxBytes / 1048576.0);
    std::printf("耗时        : LOD %.1f ms，写帧 %.1f ms，预览 %.1f ms，合计 %.1f ms\n\n",
                wr.lodMs, wr.frameMs, wr.previewMs, wr.totalMs);

    // —— 3) 重新打开校验（走真实读取路径，而不是复用打包时的内存状态） ——
    std::printf("=== 校验（重新打开文件） ===\n");
    isf::IsfReader reader;
    std::string err;
    if (!reader.open(outPath, &err)) {
        std::printf("错误：重新打开失败 —— %s\n", err.c_str());
        return 2;
    }
    std::printf("mmap: %s\n", reader.hasMmap() ? "已建立（坐标走零拷贝）"
                                             : "不可用（坐标走顺序读兜底）");
    Checker ck;
    checkStructure(reader, ck, sampleFrames);
    checkAgainstGraph(reader, graph, ck, sampleFrames);
    const int rc = ck.summary();
    std::printf(rc == 0 ? "\n结果：全部通过 ✓\n" : "\n结果：存在失败项 ✗\n");
    return rc;
}

int doCheck(int argc, char** argv) {
    const std::string isfPath = argv[2];
    const int sampleFrames = std::atoi(argValue(argc, argv, "--sample", "100").c_str());

    std::printf("=== isf_pack check ===\n文件: %s\n\n", isfPath.c_str());
    isf::IsfReader reader;
    std::string err;
    if (!reader.open(isfPath, &err)) {
        std::printf("错误：打开失败 —— %s\n", err.c_str());
        return 2;
    }
    const isf::FileHeader& h = reader.header();
    std::printf("帧数 %zu，页数 %zu，全量点数 %llu，LOD 级数 %u，预览点数 %llu\n",
                reader.frameCount(), reader.pageCount(),
                static_cast<unsigned long long>(reader.totalPoints()),
                h.lodLevelCount,
                static_cast<unsigned long long>(reader.previewPointCount()));
    std::printf("世界包围盒（按打包时优化位姿）: [%.3f %.3f %.3f] .. [%.3f %.3f %.3f]\n",
                h.worldMin[0], h.worldMin[1], h.worldMin[2],
                h.worldMax[0], h.worldMax[1], h.worldMax[2]);
    std::printf("mmap: %s\n\n", reader.hasMmap() ? "已建立" : "不可用（顺序读兜底）");

    Checker ck;
    checkStructure(reader, ck, sampleFrames);
    const int rc = ck.summary();
    std::printf(rc == 0 ? "\n结果：全部通过 ✓\n" : "\n结果：存在失败项 ✗\n");
    return rc;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 3) {
        usage();
        return 2;
    }
    const std::string cmd = argv[1];
    if (cmd == "pack") return doPack(argc, argv);
    if (cmd == "check") return doCheck(argc, argv);
    usage();
    return 2;
}
