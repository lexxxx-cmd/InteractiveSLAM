// ============================================================================
// CloudRenderData.cpp
// 位姿表 + 渲染分块 的实现
//
// 帧序约定：**按 frameId 升序**。这是三处保持一致的基础：
//   · CloudPoseTable 的槽内顺序（着色器 uFrameIdBias + 帧序索引位姿）
//   · CloudChunk 的 firstFrameIndex（全局帧序）
//   · .isf 的 FrameRecord 数组（IsfWriter.cpp 排序后写盘）
// graph->keyframes 是 unordered_map，遍历顺序不确定，因此每次都要显式排序。
// ============================================================================

#include "visualizers/CloudRenderData.h"

// 复用 Phase 1 的帧局部系体素抽稀（FNV-1a 体素键 + 自适应目标点数）
#include "pointcloud/isf/IsfVoxel.h"

#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "data/hdl_graph_slam/keyframe.hpp"

#include <g2o/types/slam3d/vertex_se3.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <vector>

namespace hdl_graph_slam {

namespace {

using PerfClock = std::chrono::steady_clock;

double msSince(const PerfClock::time_point& t0) {
    return std::chrono::duration<double, std::milli>(PerfClock::now() - t0).count();
}

/** @brief 位姿 → 数学列主序 16 float（每 4 float 一列，与 §4.4.4 一致） */
void toColumnMajor16(const Eigen::Isometry3d& T, float out[16]) {
    const Eigen::Matrix4d M = T.matrix();
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            out[4 * c + r] = static_cast<float>(M(r, c));
        }
    }
}

/**
 * @brief 位姿快照条目（POD，避免 std::vector<Eigen::Isometry3d> 的对齐讲究）
 */
struct PoseSnap {
    long  id = 0;
    float opt[CloudPoseTable::kFloatsPerPose]  = {0};
    float odom[CloudPoseTable::kFloatsPerPose] = {0};
};

/** @brief 在 optimization_mutex 保护下快照全部帧的两种位姿，并按 id 升序排序 */
std::vector<PoseSnap> snapshotPoses(const std::shared_ptr<InteractiveGraph>& graph) {
    std::vector<PoseSnap> snaps;
    if (!graph) return snaps;
    std::lock_guard<std::mutex> lock(graph->optimization_mutex);
    snaps.reserve(graph->keyframes.size());
    for (auto& [id, kf] : graph->keyframes) {
        auto* v = dynamic_cast<g2o::VertexSE3*>(kf->node);
        if (!v) continue;
        PoseSnap s;
        s.id = id;
        toColumnMajor16(v->estimate(), s.opt);
        toColumnMajor16(kf->odom, s.odom);
        snaps.push_back(s);
    }
    std::sort(snaps.begin(), snaps.end(),
              [](const PoseSnap& a, const PoseSnap& b) { return a.id < b.id; });
    return snaps;
}

}  // namespace

// ============================================================================
// CloudPoseTable
// ============================================================================

int64_t CloudPoseTable::indexOfFrame(long frameId) const {
    const auto it = std::lower_bound(m_frameIds.begin(), m_frameIds.end(), frameId);
    if (it == m_frameIds.end() || *it != frameId) return -1;
    return static_cast<int64_t>(it - m_frameIds.begin());
}

const float* CloudPoseTable::posePtr(uint32_t slot, size_t frameIndex) const {
    if (slot >= kSlotCount || frameIndex >= m_frameIds.size()) return nullptr;
    const size_t base = frameIndex * kFloatsPerPose;
    const size_t off  = static_cast<size_t>(slot) * slotFloats() + base;
    if (off + kFloatsPerPose > m_data.size()) return nullptr;
    return m_data.data() + off;
}

Eigen::Isometry3d CloudPoseTable::pose(uint32_t slot, size_t frameIndex) const {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    const float* p = posePtr(slot, frameIndex);
    if (!p) return T;
    // 输入是数学列主序：元素 [row][col] = p[4*col + row]
    Eigen::Matrix4d M;
    for (int c = 0; c < 4; ++c) {
        for (int r = 0; r < 4; ++r) {
            M(r, c) = static_cast<double>(p[4 * c + r]);
        }
    }
    T.matrix() = M;
    return T;
}

bool CloudPoseTable::needsOriginShift() const {
    return m_originShift.norm() > 1.0e5;  // UTM 量级
}

CloudPoseTable::Stats CloudPoseTable::rebuild(
    const std::shared_ptr<InteractiveGraph>& graph) {
    const auto t0 = PerfClock::now();
    Stats st;

    const std::vector<PoseSnap> snaps = snapshotPoses(graph);
    st.frameCount = snaps.size();
    st.slotFloats = st.frameCount * kFloatsPerPose;

    m_frameIds.clear();
    m_frameIds.reserve(snaps.size());
    m_data.assign(st.frameCount * kFloatsPerPose * kSlotCount, 0.0f);

    const size_t slotStride = st.slotFloats;
    for (size_t i = 0; i < snaps.size(); ++i) {
        m_frameIds.push_back(snaps[i].id);
        std::memcpy(m_data.data() + i * kFloatsPerPose, snaps[i].opt,
                    sizeof(float) * kFloatsPerPose);
        std::memcpy(m_data.data() + slotStride * kSlotOriginal + i * kFloatsPerPose,
                    snaps[i].odom, sizeof(float) * kFloatsPerPose);
    }

    // 建议原点：首帧优化位姿的平移（0 号槽第 0 帧的第 4 列前三个元素）
    if (!snaps.empty()) {
        m_originShift = Eigen::Vector3d(snaps.front().opt[12], snaps.front().opt[13],
                                       snaps.front().opt[14]);
    } else {
        m_originShift.setZero();
    }

    st.bytes   = m_data.size() * sizeof(float);
    st.buildMs = msSince(t0);
    return st;
}

CloudPoseTable::Stats CloudPoseTable::updateOptimized(
    const std::shared_ptr<InteractiveGraph>& graph) {
    const auto t0 = PerfClock::now();
    Stats st;

    const std::vector<PoseSnap> snaps = snapshotPoses(graph);

    // 帧集合变了（新增/删除关键帧）→ 槽位布局失效，退回全量重建
    bool sameSet = (snaps.size() == m_frameIds.size());
    if (sameSet) {
        for (size_t i = 0; i < snaps.size(); ++i) {
            if (snaps[i].id != m_frameIds[i]) { sameSet = false; break; }
        }
    }
    if (!sameSet) return rebuild(graph);

    // 只重写优化槽位：代价 = O(帧数) × 64 B，**与点数无关**
    const size_t slotStride = slotFloats();
    for (size_t i = 0; i < snaps.size(); ++i) {
        std::memcpy(m_data.data() + i * kFloatsPerPose, snaps[i].opt,
                    sizeof(float) * kFloatsPerPose);
    }

    st.frameCount = snaps.size();
    st.slotFloats = slotStride;
    st.bytes      = m_data.size() * sizeof(float);
    st.buildMs    = msSince(t0);
    return st;
}

// ============================================================================
// CloudChunk
// ============================================================================

size_t CloudChunk::indexBytes() const {
    size_t n = 0;
    for (const auto& v : lodIndices) n += v.size() * sizeof(uint32_t);
    return n;
}

// ============================================================================
// 世界 AABB
// ============================================================================

void refreshWorldBounds(CloudChunk& chunk, const CloudPoseTable& poses,
                        uint32_t slot) {
    chunk.worldBoundsValid = false;
    if (chunk.frames.empty()) return;

    bool first = true;
    float wmin[3] = {0, 0, 0};
    float wmax[3] = {0, 0, 0};

    for (const auto& fr : chunk.frames) {
        // 零顶点帧（点云缺失/为空）不参与包围盒，否则会把 (0,0,0) 拉进来
        if (fr.vertexCount == 0) continue;
        const int64_t fi = poses.indexOfFrame(fr.frameId);
        if (fi < 0) continue;
        const float* p = poses.posePtr(slot, static_cast<size_t>(fi));
        if (!p) continue;

        // 数学列主序：p[4*col + row]
        // 旋转 R 的 3x3 部分：R[row][col] = p[4*col + row]（row,col ∈ [0,3)）
        const double R[3][3] = {
            {p[0], p[4], p[8]},
            {p[1], p[5], p[9]},
            {p[2], p[6], p[10]},
        };
        const double t[3] = {p[12], p[13], p[14]};

        const double lo[3] = {fr.localAABB[0], fr.localAABB[1], fr.localAABB[2]};
        const double hi[3] = {fr.localAABB[3], fr.localAABB[4], fr.localAABB[5]};
        const double c[3]  = {(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5,
                              (lo[2] + hi[2]) * 0.5};
        const double e[3]  = {(hi[0] - lo[0]) * 0.5, (hi[1] - lo[1]) * 0.5,
                              (hi[2] - lo[2]) * 0.5};

        // 世界中心与半径扩展：c' = R·c + t, e' = |R|·e
        const double cx = R[0][0] * c[0] + R[0][1] * c[1] + R[0][2] * c[2] + t[0];
        const double cy = R[1][0] * c[0] + R[1][1] * c[1] + R[1][2] * c[2] + t[1];
        const double cz = R[2][0] * c[0] + R[2][1] * c[1] + R[2][2] * c[2] + t[2];
        const double cc[3] = {cx, cy, cz};

        double ex = 0, ey = 0, ez = 0;
        for (int k = 0; k < 3; ++k) {
            ex += std::fabs(R[0][k]) * e[k];
            ey += std::fabs(R[1][k]) * e[k];
            ez += std::fabs(R[2][k]) * e[k];
        }
        const double ee[3] = {ex, ey, ez};

        for (int k = 0; k < 3; ++k) {
            const double mn = cc[k] - ee[k];
            const double mx = cc[k] + ee[k];
            if (first) {
                wmin[k] = static_cast<float>(mn);
                wmax[k] = static_cast<float>(mx);
            } else {
                if (mn < wmin[k]) wmin[k] = static_cast<float>(mn);
                if (mx > wmax[k]) wmax[k] = static_cast<float>(mx);
            }
        }
        first = false;
    }

    if (first) return;  // 没有任何帧能查到位姿
    for (int k = 0; k < 3; ++k) {
        chunk.worldMin[k] = wmin[k];
        chunk.worldMax[k] = wmax[k];
    }
    chunk.worldBoundsValid = true;
}

double refreshAllWorldBounds(std::vector<CloudChunk>& chunks,
                             const CloudPoseTable& poses, uint32_t slot) {
    const auto t0 = PerfClock::now();
    for (auto& c : chunks) refreshWorldBounds(c, poses, slot);
    return msSince(t0);
}

// ============================================================================
// buildChunks
// ============================================================================

namespace {

/** @brief 把一帧点云摊平成连续 float3（帧局部系） */
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

/** @brief 一帧的局部 AABB（xyz 连续数组） */
void localAABBOf(const std::vector<float>& xyz, float lo[3], float hi[3]) {
    lo[0] = lo[1] = lo[2] =  std::numeric_limits<float>::max();
    hi[0] = hi[1] = hi[2] = -std::numeric_limits<float>::max();
    const size_t n = xyz.size() / 3;
    for (size_t i = 0; i < n; ++i) {
        for (int k = 0; k < 3; ++k) {
            const float v = xyz[3 * i + k];
            if (v < lo[k]) lo[k] = v;
            if (v > hi[k]) hi[k] = v;
        }
    }
}

}  // namespace

ChunkBuildResult buildChunks(const std::shared_ptr<InteractiveGraph>& graph,
                             CloudPoseTable& poses, const ChunkBuildOptions& opt) {
    ChunkBuildResult res;
    const auto tAll = PerfClock::now();

    // ---- 1) 重建位姿表（同时确定帧序） ----
    res.poseStats = poses.rebuild(graph);
    if (poses.empty()) return res;

    // ---- 2) 按帧序快照点云指针（帧序与位姿表一致，才能用 firstFrameIndex 索引） ----
    std::vector<pcl::PointCloud<pcl::PointXYZI>::ConstPtr> clouds;
    clouds.reserve(poses.frameCount());
    {
        std::lock_guard<std::mutex> lock(graph->optimization_mutex);
        for (long id : poses.frameIds()) {
            auto it = graph->keyframes.find(id);
            if (it == graph->keyframes.end() || !it->second->cloud) {
                clouds.emplace_back();  // 占位，保持与帧序一一对应
            } else {
                clouds.push_back(it->second->cloud);
            }
        }
    }

    const uint32_t chunkFrames =
        opt.chunkFrames ? opt.chunkFrames : 256;
    const uint32_t lodLevels =
        std::min<uint32_t>(std::max<uint32_t>(opt.lodLevels, 1u), 4u);

    std::vector<float>    xyz;   // 单帧临时缓冲（峰值内存 = 单帧）
    std::vector<uint32_t> frameIdx;

    const size_t totalFrames = clouds.size();
    for (size_t first = 0; first < totalFrames; first += chunkFrames) {
        const size_t last = std::min(first + chunkFrames, totalFrames);

        CloudChunk chunk;
        chunk.firstFrameIndex = static_cast<uint32_t>(first);
        chunk.frameCount      = static_cast<uint32_t>(last - first);
        chunk.lodIndices.resize(lodLevels);

        for (size_t fi = first; fi < last; ++fi) {
            // 注意：**即使点云缺失/为空也要推入 FrameRange**。帧号属性是
            // "chunk 内帧序号"，只有 c.frames 的下标与 (fi - first) 严格对齐，
            // 着色器的 aFrameId 才能正确索引位姿表。零顶点帧在
            // refreshWorldBounds 里被跳过，不影响包围盒。
            const bool hasCloud = clouds[fi] && !clouds[fi]->empty();

            float lo[3] = {0, 0, 0};
            float hi[3] = {0, 0, 0};
            uint32_t count = 0;
            if (hasCloud) {
                flattenFrame(*clouds[fi], xyz);
                localAABBOf(xyz, lo, hi);
                count = static_cast<uint32_t>(xyz.size() / 3);
            }

            CloudChunk::FrameRange fr;
            fr.startVertex = static_cast<uint32_t>(chunk.positions.size() / 3);
            fr.vertexCount = count;
            fr.frameId     = poses.frameIds()[fi];
            std::memcpy(fr.localAABB, lo, sizeof(float) * 3);
            std::memcpy(fr.localAABB + 3, hi, sizeof(float) * 3);

            if (hasCloud) {
                // 顶点与帧号属性（帧号 = chunk 内帧序号，即着色器 aFrameId）
                chunk.positions.insert(chunk.positions.end(), xyz.begin(), xyz.end());
                const uint16_t localId =
                    static_cast<uint16_t>((fi - first) & 0xFFFFu);
                chunk.frameLocalId.insert(chunk.frameLocalId.end(), count, localId);

                // 逐帧 LOD：索引经 startVertex 平移后并入 chunk 级索引数组。
                // 逐帧（而非整 chunk）抽稀有两个好处：每帧至少保留一个点
                // （高亮/播放时该帧不会消失），且每帧的 LOD 不依赖邻帧是否驻留。
                for (uint32_t L = 1; L < lodLevels; ++L) {
                    const size_t target = count >> L;  // N/2, N/4, N/8
                    // 与 .isf 打包共用同一逐帧门槛（IsfVoxel.h 的 kMinLodPoints）：
                    // 两条路径必须产生一致的逐帧 LOD，否则"从 .isf 分页"与"直接构建
                    // chunk"在同一个切换级别上会显示不同点数（原先这里是写死的 8）
                    if (target < isf::kMinLodPoints) break;
                    const auto tLod = PerfClock::now();
                    isf::decimateToTarget(xyz.data(), count, target, lo, hi, frameIdx);
                    res.lodMs += msSince(tLod);
                    auto& dst = chunk.lodIndices[L];
                    dst.reserve(dst.size() + frameIdx.size());
                    for (uint32_t k : frameIdx) dst.push_back(fr.startVertex + k);
                }
            }

            chunk.pointCount += count;
            chunk.frames.push_back(fr);
        }

        // **即使 pointCount == 0 也要推入**，以维持不变量
        // chunks[i].firstFrameIndex == i * chunkFrames（见头文件说明）。
        // 空 chunk 的 worldBoundsValid 保持 false，绘制 0 个点，无副作用。
        refreshWorldBounds(chunk, poses, CloudPoseTable::kSlotOptimized);
        res.chunks.push_back(std::move(chunk));

        if (opt.onProgress) opt.onProgress(last, totalFrames);
    }

    // ---- 3) 汇总 ----
    for (const auto& c : res.chunks) {
        res.totalPoints   += c.pointCount;
        res.vertexBytes   += c.vertexBytes();
        res.frameIdBytes  += c.frameIdBytes();
        res.indexBytes    += c.indexBytes();
        res.lodPoints[0]  += c.pointCount;
        for (size_t L = 1; L < c.lodIndices.size() && L < 4; ++L) {
            res.lodPoints[L] += c.lodIndices[L].size();
        }
    }
    res.totalMs = msSince(tAll);
    return res;
}

}  // namespace hdl_graph_slam
