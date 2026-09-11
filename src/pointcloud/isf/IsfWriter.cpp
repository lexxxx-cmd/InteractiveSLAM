// ============================================================================
// IsfWriter.cpp
// .isf / .isf.idx 打包器实现
//
// 处理顺序：加锁快照 → 按 frameId 升序排序 → 写头占位 → 写全局粗预览块
//          → 逐帧写 L0 顶点块与 LOD 索引块 → 回填文件头 → 写索引文件。
//
// 内存策略：逐帧流式写入。峰值内存 ≈ 单帧点数 × (12 B 坐标 + 索引)，
// 与地图总点数无关 —— 1 亿点地图不会因为打包而爆内存。
// ============================================================================

#include "pointcloud/isf/IsfWriter.h"

#include "pointcloud/isf/IsfVoxel.h"
#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "data/hdl_graph_slam/keyframe.hpp"

#include <Eigen/Geometry>
#include <g2o/types/slam3d/vertex_se3.h>

#include <algorithm>
#include <cfloat>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace hdl_graph_slam {
namespace isf {

namespace {

using PerfClock = std::chrono::steady_clock;

/** @brief 距 t0 的毫秒数 */
double msSince(const PerfClock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(PerfClock::now() - t0).count();
}

/**
 * @brief 单帧快照
 *
 * 只持有 ConstPtr（点云内容加载后不可变），因此快照之外无需加锁。
 * 同时保留优化位姿（仅用于统计世界包围盒与位姿原点，不写进文件）与
 * 原始位姿（写进 FrameRecord.poseOdom，是"冻结的参照"）。
 */
struct FrameSnap {
    long                                      id       = 0;
    Eigen::Isometry3d                         poseOdom = Eigen::Isometry3d::Identity();
    Eigen::Isometry3d                         poseOpt  = Eigen::Isometry3d::Identity();
    pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud;
};

/**
 * @brief 位姿 → 数学列主序 16 float（每 4 float = 一列）
 *
 * 与设计文档 §4.4.4 的结论一致：位姿纹理每帧 4 个 texel，每 texel 一列。
 * 这里不经 OSG，直接用 Eigen 的数学下标，避免 OSG 行主序存储带来的转置混淆。
 */
void toColumnMajor16(const Eigen::Isometry3d& T, float out[16]) {
    const Eigen::Matrix4d M = T.matrix();
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            out[4 * c + r] = static_cast<float>(M(r, c));
        }
    }
}

/**
 * @brief 把一帧点云摊平成连续 float3（**帧局部系**坐标，不含强度）
 *
 * pcl::PointXYZI 含 4 字节强度且结构体有填充，不能整体 memcpy，
 * 必须逐点取 x/y/z。
 */
void flattenFrame(const pcl::PointCloud<pcl::PointXYZI>& cloud,
                  std::vector<float>& out) {
    const size_t n = cloud.size();
    out.resize(n * 3);
    float* d = out.data();
    for (size_t i = 0; i < n; ++i) {
        const auto& p = cloud.points[i];
        *d++ = p.x;
        *d++ = p.y;
        *d++ = p.z;
    }
}

/** @brief 写一段 POD 数组到流（失败时置位流状态） */
template <typename T>
void writePod(std::ofstream& os, const std::vector<T>& v) {
    if (!v.empty()) {
        os.write(reinterpret_cast<const char*>(v.data()),
                 static_cast<std::streamsize>(v.size() * sizeof(T)));
    }
}

}  // namespace

WriteResult write(const std::shared_ptr<InteractiveGraph>& graph,
                  const std::string& isfPath, const WriteOptions& opt) {
    WriteResult r;
    const auto tAll = PerfClock::now();

    if (!graph) {
        r.error = "graph 为空";
        return r;
    }

    // ---- 1) 加锁快照 + 按 frameId 升序排序 ----
    // 升序是关键：页/chunk 归属由"帧序号"推导（见 IsfFormat.h 的
    // pageIdOf/chunkIdOf），而 graph->keyframes 是 unordered_map，
    // 直接遍历得到的顺序不确定，会让页划分不可复现。
    std::vector<FrameSnap, Eigen::aligned_allocator<FrameSnap>> frames;
    {
        std::lock_guard<std::mutex> lock(graph->optimization_mutex);
        frames.reserve(graph->keyframes.size());
        for (auto& [id, kf] : graph->keyframes) {
            auto* v = dynamic_cast<g2o::VertexSE3*>(kf->node);
            if (!v || !kf->cloud || kf->cloud->empty()) continue;
            FrameSnap s;
            s.id       = id;
            s.poseOdom = kf->odom;
            s.poseOpt  = v->estimate();
            s.cloud    = kf->cloud;
            frames.push_back(std::move(s));
        }
    }
    if (frames.empty()) {
        r.error = "图中没有可用的关键帧点云";
        return r;
    }
    std::sort(frames.begin(), frames.end(),
              [](const FrameSnap& a, const FrameSnap& b) { return a.id < b.id; });

    const uint64_t frameCount = frames.size();
    uint64_t totalPoints = 0;
    for (const auto& f : frames) totalPoints += static_cast<uint64_t>(f.cloud->size());

    // ---- 2) 组装文件头 ----
    FileHeader fh{};
    std::memcpy(fh.magic, kMagic, sizeof(kMagic));
    fh.version        = kVersion;
    fh.headerBytes    = kHeaderBytes;
    fh.l0Stride       = kL0Stride;
    fh.previewStride  = kPreviewStride;
    fh.lodLevelCount  = std::min<uint32_t>(std::max<uint32_t>(opt.lodLevels, 1u),
                                           kMaxLodLevels);
    fh.framesPerPage  = opt.framesPerPage ? opt.framesPerPage : kDefaultFramesPerPage;
    fh.chunkFrames    = opt.chunkFrames ? opt.chunkFrames : kDefaultChunkFrames;
    fh.frameCount     = frameCount;
    fh.totalPoints    = totalPoints;
    fh.pageCount      = (frameCount + fh.framesPerPage - 1) / fh.framesPerPage;

    // 建议的位姿原点：首帧优化位姿平移。渲染期用它把位姿归零，避免
    // UTM 量级坐标在 float 下的精度损失（见设计文档 §4.4.4 的精度说明）。
    const Eigen::Vector3d shift = frames.front().poseOpt.translation();
    fh.originShift[0] = shift.x();
    fh.originShift[1] = shift.y();
    fh.originShift[2] = shift.z();

    // ---- 3) 打开文件、写头占位 ----
    std::ofstream out(isfPath, std::ios::binary | std::ios::trunc);
    if (!out) {
        r.error = "无法创建 .isf：" + isfPath;
        return r;
    }
    {
        const std::vector<char> placeholder(kHeaderBytes, 0);
        out.write(placeholder.data(), static_cast<std::streamsize>(placeholder.size()));
    }
    uint64_t pos = kHeaderBytes;  // 当前写入位置（手动跟踪，便于回填偏移）

    // ---- 4) 全局粗预览块（stride 均匀采样，借 3d-forest 的做法） ----
    fh.previewOffset = pos;
    if (opt.buildPreview && opt.previewTarget > 0 && totalPoints > 0) {
        const auto t0 = PerfClock::now();
        const uint64_t stride =
            std::max<uint64_t>(1, totalPoints / std::max<uint64_t>(1, opt.previewTarget));

        std::vector<float> xyz;
        std::vector<PreviewPoint> batch;
        batch.reserve(65536);
        uint64_t globalIdx = 0;
        uint64_t written = 0;

        auto flushBatch = [&]() {
            if (batch.empty()) return;
            out.write(reinterpret_cast<const char*>(batch.data()),
                      static_cast<std::streamsize>(batch.size() * sizeof(PreviewPoint)));
            written += batch.size();
            batch.clear();
        };

        for (uint64_t fi = 0; fi < frameCount; ++fi) {
            const FrameSnap& f = frames[fi];
            flattenFrame(*f.cloud, xyz);
            const size_t n = f.cloud->size();
            for (size_t i = 0; i < n; ++i, ++globalIdx) {
                if (globalIdx % stride != 0) continue;
                PreviewPoint pp;
                pp.x = xyz[3 * i + 0];
                pp.y = xyz[3 * i + 1];
                pp.z = xyz[3 * i + 2];
                // 注意用"帧序号"而非帧 ID：它直接索引 FrameRecord 数组，
                // 渲染期着色器据此索引位姿表
                pp.frameIndex = static_cast<uint32_t>(fi);
                batch.push_back(pp);
                if (batch.size() >= 65536) flushBatch();
            }
        }
        flushBatch();

        fh.previewPoints = written;
        pos += written * static_cast<uint64_t>(kPreviewStride);
        r.previewMs = msSince(t0);
    }

    // ---- 5) 逐帧：L0 顶点块 + LOD 索引块 ----
    std::vector<FrameRecord> records;
    records.reserve(static_cast<size_t>(frameCount));
    std::vector<PageRecord> pages;
    pages.reserve(static_cast<size_t>(fh.pageCount));

    std::vector<float>    xyz;
    std::vector<uint32_t> lodIdx;
    uint64_t lodAcc[kMaxLodLevels] = {0, 0, 0, 0};
    double wmin[3] = {DBL_MAX, DBL_MAX, DBL_MAX};
    double wmax[3] = {-DBL_MAX, -DBL_MAX, -DBL_MAX};

    for (uint64_t fi = 0; fi < frameCount; ++fi) {
        const FrameSnap& f = frames[fi];
        const auto tFrame = PerfClock::now();

        flattenFrame(*f.cloud, xyz);
        const size_t n = f.cloud->size();
        if (n > 0xFFFFFFFFull) {
            r.error = "单帧点数超出 uint32 表示范围";
            return r;
        }

        // 帧局部 AABB（层 C：与位姿无关，永不失效）
        float lo[3] = {FLT_MAX, FLT_MAX, FLT_MAX};
        float hi[3] = {-FLT_MAX, -FLT_MAX, -FLT_MAX};
        for (size_t i = 0; i < n; ++i) {
            for (int k = 0; k < 3; ++k) {
                const float v = xyz[3 * i + k];
                if (v < lo[k]) lo[k] = v;
                if (v > hi[k]) hi[k] = v;
            }
        }

        FrameRecord rec{};
        rec.frameId         = f.id;
        rec.pointCount      = static_cast<uint32_t>(n);
        rec.pageId          = pageIdOf(fi, fh.framesPerPage);
        rec.chunkId         = chunkIdOf(fi, fh.framesPerPage, fh.chunkFrames);
        rec.chunkLocalFrame = chunkLocalFrameOf(fi, fh.chunkFrames);
        rec.lodCount        = 1;
        rec.lodOffset[0]    = 0;  // L0 偏移由 l0Offset 承载
        rec.lodCounts[0]    = rec.pointCount;
        rec.localAABB[0] = lo[0];
        rec.localAABB[1] = lo[1];
        rec.localAABB[2] = lo[2];
        rec.localAABB[3] = hi[0];
        rec.localAABB[4] = hi[1];
        rec.localAABB[5] = hi[2];
        toColumnMajor16(f.poseOdom, rec.poseOdom);

        // 世界包围盒（按优化位姿，仅作参考信息写进文件头）
        {
            const Eigen::Matrix4d M = f.poseOpt.matrix();
            for (int c = 0; c < 8; ++c) {
                const double px = (c & 1) ? hi[0] : lo[0];
                const double py = (c & 2) ? hi[1] : lo[1];
                const double pz = (c & 4) ? hi[2] : lo[2];
                const Eigen::Vector4d w = M * Eigen::Vector4d(px, py, pz, 1.0);
                const double v[3] = {w.x(), w.y(), w.z()};
                for (int k = 0; k < 3; ++k) {
                    if (v[k] < wmin[k]) wmin[k] = v[k];
                    if (v[k] > wmax[k]) wmax[k] = v[k];
                }
            }
        }

        // L0 顶点块（帧局部系，float3）
        rec.l0Offset = pos;
        const uint64_t l0Bytes = static_cast<uint64_t>(n) * kL0Stride;
        out.write(reinterpret_cast<const char*>(xyz.data()),
                  static_cast<std::streamsize>(l0Bytes));
        pos += l0Bytes;
        lodAcc[0] += n;

        // LOD 索引块（指向本帧自己的 L0 顶点，索引 < n）
        for (uint32_t L = 1; L < fh.lodLevelCount; ++L) {
            const size_t target = n >> L;          // N/2, N/4, N/8
            // 逐帧门槛（IsfVoxel.h 的 kMinLodPoints）：帧太小就不再生成更粗级别
            if (target < kMinLodPoints) break;     // 太小不再生成更粗级别
            const auto tLod = PerfClock::now();
            decimateToTarget(xyz.data(), n, target, lo, hi, lodIdx);
            r.lodMs += msSince(tLod);

            rec.lodOffset[L] = pos;
            rec.lodCounts[L] = static_cast<uint32_t>(lodIdx.size());
            const uint64_t bytes = static_cast<uint64_t>(lodIdx.size()) * 4ull;
            writePod(out, lodIdx);
            pos += bytes;
            lodAcc[L] += lodIdx.size();
            rec.lodCount = static_cast<uint16_t>(L + 1);
        }

        records.push_back(rec);
        r.frameMs += msSince(tFrame);

        if (opt.onProgress && ((fi & 0x3F) == 0 || fi + 1 == frameCount)) {
            opt.onProgress(fi + 1, frameCount);
        }
        if (!out) {
            r.error = "写入 .isf 数据区失败（磁盘空间不足？）";
            return r;
        }
    }

    // ---- 6) 页表 ----
    for (uint64_t fi = 0; fi < frameCount;) {
        const uint32_t pid = pageIdOf(fi, fh.framesPerPage);
        const uint64_t first = fi;
        const uint64_t beginOffset = records[static_cast<size_t>(fi)].l0Offset;
        while (fi < frameCount && pageIdOf(fi, fh.framesPerPage) == pid) ++fi;
        PageRecord pr{};
        pr.pageId          = pid;
        pr.firstFrameIndex = static_cast<uint32_t>(first);
        pr.frameCount      = static_cast<uint32_t>(fi - first);
        pr.fileOffset      = beginOffset;
        pr.fileBytes       = pos - beginOffset;  // 本页区间 [begin, 下一页 begin)
        pages.push_back(pr);
    }

    // ---- 7) 回填文件头 ----
    fh.dataBytes    = pos - kHeaderBytes;
    fh.worldMin[0]  = wmin[0];
    fh.worldMin[1]  = wmin[1];
    fh.worldMin[2]  = wmin[2];
    fh.worldMax[0]  = wmax[0];
    fh.worldMax[1]  = wmax[1];
    fh.worldMax[2]  = wmax[2];
    out.seekp(0, std::ios::beg);
    out.write(reinterpret_cast<const char*>(&fh), sizeof(fh));
    out.flush();
    if (!out) {
        r.error = "回填 .isf 文件头失败";
        return r;
    }
    out.close();

    // ---- 8) 索引文件 ----
    const std::string idxPath = isfPath + ".idx";
    std::ofstream idxOut(idxPath, std::ios::binary | std::ios::trunc);
    if (!idxOut) {
        r.error = "无法创建 .isf.idx：" + idxPath;
        return r;
    }
    IndexHeader ih{};
    std::memcpy(ih.magic, kIndexMagic, sizeof(kIndexMagic));
    ih.version       = kVersion;
    ih.headerBytes   = kIndexHeaderBytes;
    ih.lodLevelCount = fh.lodLevelCount;
    ih.framesPerPage = fh.framesPerPage;
    ih.chunkFrames   = fh.chunkFrames;
    ih.frameCount    = frameCount;
    ih.pageCount     = pages.size();
    ih.totalPoints   = totalPoints;
    ih.firstFrameId  = records.front().frameId;
    ih.lastFrameId   = records.back().frameId;
    {
        std::vector<char> placeholder(kIndexHeaderBytes, 0);
        idxOut.write(placeholder.data(),
                     static_cast<std::streamsize>(placeholder.size()));
    }
    idxOut.seekp(0, std::ios::beg);
    idxOut.write(reinterpret_cast<const char*>(&ih), sizeof(ih));
    idxOut.seekp(static_cast<std::streamoff>(kIndexHeaderBytes), std::ios::beg);
    writePod(idxOut, records);
    writePod(idxOut, pages);
    idxOut.flush();
    if (!idxOut) {
        r.error = "写入 .isf.idx 失败";
        return r;
    }
    idxOut.close();

    // ---- 9) 汇总 ----
    r.ok            = true;
    r.isfPath       = isfPath;
    r.idxPath       = idxPath;
    r.frameCount    = frameCount;
    r.pageCount     = pages.size();
    r.totalPoints   = totalPoints;
    r.previewPoints = fh.previewPoints;
    r.isfBytes      = pos;  // 实际字节数（权威值）
    r.idxBytes      = static_cast<uint64_t>(kIndexHeaderBytes) +
                      static_cast<uint64_t>(records.size() * sizeof(FrameRecord)) +
                      static_cast<uint64_t>(pages.size() * sizeof(PageRecord));
    for (uint32_t L = 0; L < kMaxLodLevels; ++L) r.lodPoints[L] = lodAcc[L];
    r.totalMs  = msSince(tAll);
    return r;
}

}  // namespace isf
}  // namespace hdl_graph_slam
