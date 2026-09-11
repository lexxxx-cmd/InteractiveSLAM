// ============================================================================
// CloudRenderData.h
// 位姿表 + 渲染分块 —— "位姿上移着色器"架构的数据侧
//
// 本文件是设计文档 §3.2 / §4.4 的实现核心，落地两个因子分解：
//
//     world(p) = T_frame(p) · p
//     └── 可变：位姿表（每帧 64 B，O(帧数) 可重建）
//     └── 不可变：chunk 顶点（帧局部系坐标，与位姿无关，写一次永不重传）
//
// 由此得到三个可测量的结论（tools/cloud_selftest 会逐条验证）：
//   1. 位姿优化后只需重建位姿表（O(帧数)，3 万帧 ≈ 2 MB 上传），
//      顶点数据零重传 —— 对比现有 PointCloudBuilder 的 O(1 亿) 重变换；
//   2. 双份显示（优化层 + 原始层）共享同一批 chunk 顶点，只绑不同的
//      位姿槽位（uPoseOffset），显存 1× 而不是 2×；
//   3. chunk 的世界 AABB 由"帧局部 AABB × 当前位姿"算出，因此视锥剔除在
//      位姿变化后仍然正确（只是需要 O(帧数) 刷新，不需要碰顶点）。
//
// 与 .isf 的关系：chunk 顶点布局（float3 局部坐标，按帧序连续）与
// .isf 的 L0 块完全一致，因此 Phase 6 的分页可以直接 mmap .isf 后零拷贝
// 填进 chunk，而不必再走一遍 PCD 解码。
// ============================================================================

#pragma once

#include <Eigen/Geometry>
#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

namespace hdl_graph_slam {

class InteractiveGraph;

// ============================================================================
// 位姿表
// ============================================================================

/**
 * @brief 双槽位姿表（槽 0 = 优化位姿，槽 1 = 原始冻结位姿）
 *
 * 内存布局：`[槽0 的 N 帧 × 16 float][槽1 的 N 帧 × 16 float]`，帧序按
 * frameId 升序（与 .isf 帧表、chunk 分块顺序一致）。
 *
 * 每个位姿 16 个 float 为**数学列主序**（每 4 float 一列），因此可以直接作为
 * 4 个 vec4 texel 上传到 `samplerBuffer`，着色器里 `mat4(c0,c1,c2,c3)` 即可用。
 * 约定来源见设计文档 §4.4.4（`osg::Matrix::_mat[r][c] = M_math[c][r]`，
 * `ptr()` 即数学列主序）。这里不经 OSG，直接用 Eigen 数学下标，避免转置混淆。
 *
 * 槽位选择：着色器 uniform `uPoseOffset` = 0 或 `frameCount`（= 单槽 float 数
 * `slotFloats()`），配合 `uFrameIdBias` 索引。两层共享顶点，只改这两个 uniform。
 */
class CloudPoseTable {
public:
    /** @brief 每个位姿的 float 数（4×4 数学列主序） */
    static constexpr size_t kFloatsPerPose = 16;
    /** @brief 槽位：优化位姿（随 g2o 优化变化） */
    static constexpr uint32_t kSlotOptimized = 0;
    /** @brief 槽位：原始位姿（冻结，加载后不再写） */
    static constexpr uint32_t kSlotOriginal = 1;
    /** @brief 槽位总数 */
    static constexpr uint32_t kSlotCount = 2;

    /** @brief 重建/更新统计（Phase 0 基线口径） */
    struct Stats {
        size_t frameCount = 0;   ///< 帧数
        size_t slotFloats = 0;   ///< 单槽 float 数（= 着色器 uPoseOffset 步长）
        size_t bytes      = 0;   ///< 两槽合计字节数
        double buildMs    = 0.0; ///< 本次重建耗时
    };

    /**
     * @brief 从图重建两个槽位（加载地图/首次构建时调用一次）
     *
     * 在 `optimization_mutex` 保护下做毫秒级快照，之后无锁填表。
     * 同时记录帧 ID 顺序，供 chunk 构建与二分查找使用。
     */
    Stats rebuild(const std::shared_ptr<InteractiveGraph>& graph);

    /**
     * @brief 只更新优化槽位（每次 g2o 优化完成后调用）
     *
     * 这是本架构的核心收益所在：**代价只与帧数有关，与点数无关**。
     * 3 万帧 = 1.92 MB 的浮点写入 + 一次 1.92 MB 的显存上传。
     * 原始槽位不动，因此"原始点云冻结"是结构性保证。
     */
    Stats updateOptimized(const std::shared_ptr<InteractiveGraph>& graph);

    /** @brief 连续位姿缓冲（上传到纹理/UBO 用） */
    const std::vector<float>& data() const { return m_data; }

    /** @brief 帧数 */
    size_t frameCount() const { return m_frameIds.size(); }

    /** @brief 单槽 float 数（= 着色器 uPoseOffset 步长） */
    size_t slotFloats() const { return m_frameIds.size() * kFloatsPerPose; }

    /** @brief 帧 ID 顺序（升序） */
    const std::vector<long>& frameIds() const { return m_frameIds; }

    /** @brief 是否已构建 */
    bool empty() const { return m_frameIds.empty(); }

    /**
     * @brief 按 frameId 查帧序（二分；不存在返回 -1）
     *
     * 帧序与 `.isf` 的 FrameRecord 下标、chunk 内 frameLocalId 基准一致，
     * 因此三方可以互相索引。
     */
    int64_t indexOfFrame(long frameId) const;

    /**
     * @brief 建议的位姿原点（首帧优化位姿的平移）
     *
     * 仅作诊断：本表**不做**坐标平移，因为地图位姿本来就位于地图局部系。
     * 若某地图的位姿量级达到 UTM 级（1e6 米），float 在世界系下的分辨率会
     * 退化到厘米级，那时应先用该原点归零（`needsOriginShift()` 为 true 时）。
     */
    const Eigen::Vector3d& originShift() const { return m_originShift; }

    /** @brief 位姿量级是否已大到需要原点归零（|平移| > 1e5 米） */
    bool needsOriginShift() const;

    /** @brief 取某槽某帧的位姿指针（越界返回 nullptr） */
    const float* posePtr(uint32_t slot, size_t frameIndex) const;

    /** @brief 把某槽某帧的位姿还原成 Eigen 位姿（验证/调试用） */
    Eigen::Isometry3d pose(uint32_t slot, size_t frameIndex) const;

private:
    std::vector<float> m_data;      ///< [槽0 的 N×16][槽1 的 N×16]，按帧序
    std::vector<long>  m_frameIds;  ///< 升序帧 ID（与槽内顺序一致）
    Eigen::Vector3d    m_originShift = Eigen::Vector3d::Zero();
};

// ============================================================================
// 渲染分块
// ============================================================================

/**
 * @brief 一个渲染分块（chunk = 连续的帧段）
 *
 * 顶点按帧序连续存放，与 `.isf` 的 L0 块布局一致；`frameLocalId` 给出每个
 * 顶点属于 chunk 内哪一帧（着色器的 `aFrameId`）。**顶点与位姿无关**，
 * 因此位姿优化后不需要重建 chunk 顶点，只需要刷新 `worldMin/worldMax`。
 */
struct CloudChunk {
    /** @brief 一帧在本 chunk 顶点数组内的范围（等价于现有 CloudRange） */
    struct FrameRange {
        uint32_t startVertex = 0;  ///< 在 positions 中的起始顶点
        uint32_t vertexCount = 0;  ///< 顶点数
        long     frameId     = 0;  ///< 关键帧顶点 ID
        float    localAABB[6] = {0, 0, 0, 0, 0, 0};  ///< 帧局部系 AABB
    };

    std::vector<float>    positions;    ///< float3 × N（帧局部系）
    std::vector<uint16_t> frameLocalId; ///< chunk 内帧序号（= aFrameId）
    std::vector<FrameRange> frames;     ///< 逐帧范围（渲染期 aFrameId 基准）

    /**
     * @brief LOD 索引子集：lodIndices[L] 指向 positions 的顶点下标
     *
     * `lodIndices[0]` 恒为空 —— level 0 即全量，直接 `glDrawArrays(0, pointCount)`。
     * level L>=1 的元素严格升序且 < pointCount（`IsfVoxel.h` 保证）。
     */
    std::vector<std::vector<uint32_t>> lodIndices;

    /** @brief 本 chunk 覆盖的帧在全局帧序中的起始下标（= 页面/帧表下标） */
    uint32_t firstFrameIndex = 0;
    /**
     * @brief 帧跨度（含点云缺失/为空的帧）
     *
     * `frames.size()` **恒等于** `frameCount` —— 零顶点帧也占一个 FrameRange。
     * 这是必须的：着色器的 `aFrameId` 就是 `frames` 的下标，只有两者严格对齐，
     * `uFrameIdBias + aFrameId` 才能正确索引位姿表。零顶点帧在
     * `refreshWorldBounds` 中被跳过，不影响包围盒。
     */
    uint32_t frameCount = 0;
    /** @brief 本 chunk 的顶点总数 */
    uint32_t pointCount = 0;

    /** @brief 动态世界 AABB（由位姿表算出；位姿变化后调用 refreshWorldBounds） */
    float worldMin[3] = {0, 0, 0};
    float worldMax[3] = {0, 0, 0};
    /** @brief 世界 AABB 是否有效 */
    bool  worldBoundsValid = false;

    /** @brief 顶点字节数（float3） */
    size_t vertexBytes() const { return positions.size() * sizeof(float); }
    /** @brief 帧号属性字节数（uint16） */
    size_t frameIdBytes() const { return frameLocalId.size() * sizeof(uint16_t); }
    /** @brief LOD 索引字节数（各级合计） */
    size_t indexBytes() const;
};

/**
 * @brief 分块构建选项
 */
struct ChunkBuildOptions {
    uint32_t chunkFrames = 256;  ///< 每 chunk 帧数（= 着色器 aFrameId 的模）
    uint32_t lodLevels   = 4;    ///< 生成级别数（含 L0）
    /** @brief 进度回调（可选）：已处理帧数 / 总帧数 */
    std::function<void(uint64_t done, uint64_t total)> onProgress;
};

/**
 * @brief 分块构建结果与规模统计
 */
struct ChunkBuildResult {
    /**
     * @brief 全部 chunk（按帧序连续划分）
     *
     * **不变量**：`chunks[i].firstFrameIndex == i * chunkFrames`，且
     * `chunks.size() == ceil(frameCount / chunkFrames)`。
     * 即使某个 chunk 的顶点数为 0（该段内所有点云都缺失）也保留占位，
     * 这样"全局帧序 ↔ chunk 下标"的换算是纯算术，不需要额外索引表。
     */
    std::vector<CloudChunk> chunks;

    CloudPoseTable::Stats poseStats;  ///< 位姿表重建统计

    uint64_t totalPoints = 0;                 ///< 全量点数
    uint64_t lodPoints[4] = {0, 0, 0, 0};     ///< 各级点数（[0] = 全量）
    size_t   vertexBytes = 0;                 ///< 顶点字节（float3）
    size_t   frameIdBytes = 0;                ///< 帧号属性字节
    size_t   indexBytes = 0;                  ///< LOD 索引字节
    /** @brief 顶点+帧号+LOD 索引合计字节（单份，双层共享同一份） */
    size_t   totalBytes() const { return vertexBytes + frameIdBytes + indexBytes; }

    double totalMs = 0.0;  ///< 端到端耗时
    double lodMs   = 0.0;  ///< LOD 抽稀累计耗时
};

/**
 * @brief 从图构建全部渲染分块（帧局部系坐标 + LOD 索引 + 动态世界 AABB）
 *
 * 线程安全：只在开始时于 `optimization_mutex` 保护下做快照；之后无锁计算。
 * 顶点与位姿无关，因此**位姿优化不需要重新调用本函数**，只需
 * `CloudPoseTable::updateOptimized()` + `refreshWorldBounds()`。
 *
 * @param graph 已加载地图
 * @param poses 位姿表（**会被本函数重建**，以保证帧序与 chunk 分块顺序一致）
 * @param opt   构建选项
 */
ChunkBuildResult buildChunks(const std::shared_ptr<InteractiveGraph>& graph,
                             CloudPoseTable& poses,
                             const ChunkBuildOptions& opt = ChunkBuildOptions());

/**
 * @brief 刷新单个 chunk 的世界 AABB（位姿变化后调用，O(chunk 帧数)）
 *
 * 用"旋转绝对值矩阵"技巧：对 AABB 只需 `center' = R·c + t`、
 * `extent' = |R|·extent`，不必变换 8 个角点。整图刷新是 O(帧数)，
 * 3 万帧约几毫秒，远低于重传顶点。
 *
 * @param chunk 目标 chunk（原地更新 worldMin/worldMax/worldBoundsValid）
 * @param poses 当前位姿表
 * @param slot  槽位（kSlotOptimized = 优化层，kSlotOriginal = 原始层）
 */
void refreshWorldBounds(CloudChunk& chunk, const CloudPoseTable& poses, uint32_t slot);

/**
 * @brief 刷新全部 chunk 的世界 AABB（返回耗时毫秒）
 */
double refreshAllWorldBounds(std::vector<CloudChunk>& chunks,
                             const CloudPoseTable& poses, uint32_t slot);

}  // namespace hdl_graph_slam
