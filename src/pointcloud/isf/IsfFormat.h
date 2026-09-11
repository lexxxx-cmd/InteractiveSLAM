// ============================================================================
// IsfFormat.h
// .isf（InteractiveSLAM Frame Store）点云帧存储格式 —— 磁盘布局定义
//
// 设计目标（对应 docs/point_cloud_lod_paging_design.md §4.2）：
//   1. **只存帧局部系坐标**。世界坐标 = T_帧 · 局部坐标，位姿不进文件、
//      由运行时的位姿表提供。因此一次写盘后，无论 g2o 怎么优化都不需要
//      重写文件、不需要重传顶点。
//   2. **存储索引非空间化**。页号、chunk 号、字节偏移全部由"帧序号"推导，
//      与位姿无关，永不失效。
//   3. **帧 → 点范围**由 FrameRecord{ l0Offset, pointCount } 给出，语义等价
//      于现有 PointCloudBuilder 的 CloudRange。
//   4. 块内按帧连续、页对齐，可直接 mmap 后零拷贝上传为 VBO。
//
// 实现期修正（相对设计文档 §4.4.1 的 18 B/点）：
//   **不存颜色、不存帧号** → L0 步长 12 B/点（仅 float3）。
//   - 颜色：CPU 的 turboColor() 是 256 项查表（TurboColormap.h:39-104），
//     Phase 2 把该表作为 256×1 纹理上传，着色器按世界 Z 采样即可得到
//     **逐位一致**的结果；好处是颜色随位姿自动正确（烘焙颜色在优化后会失真），
//     并且颜色范围变成 uniform，消灭现有 recolorAll() 的 O(N) 重着色路径。
//   - 帧号：chunk 由帧段构成，帧号可由帧表逐帧累加推导，无需逐顶点存储。
//   1 亿点因此为 1.2 GB 而非 1.8 GB。
// ============================================================================

#pragma once

#include <cstdint>

namespace hdl_graph_slam {
namespace isf {

/** @brief .isf 文件魔数（含结尾版本号，便于肉眼识别） */
constexpr char     kMagic[8]         = {'I', 'S', 'F', '1', '\0', '\0', '\0', '\0'};
/** @brief .isf.idx 文件魔数 */
constexpr char     kIndexMagic[8]    = {'I', 'S', 'F', '1', 'I', 'D', 'X', '\0'};
/** @brief 当前格式版本 */
constexpr uint32_t kVersion          = 1;
/** @brief LOD 级别上限（L0 全分辨率 + 至多 3 级降采样） */
constexpr uint32_t kMaxLodLevels     = 4;
/** @brief 文件头预留长度（数据区从此偏移开始，便于向前兼容扩字段） */
constexpr uint32_t kHeaderBytes      = 512;
/** @brief 索引文件头预留长度 */
constexpr uint32_t kIndexHeaderBytes = 512;
/** @brief L0 每点字节数（float x,y,z） */
constexpr uint32_t kL0Stride         = 12;
/** @brief 全局粗预览块每点字节数（float x,y,z + uint32 全局帧序号） */
constexpr uint32_t kPreviewStride    = 16;
/** @brief 全局粗预览块的目标点数（常驻显存，约 16 MB） */
constexpr uint64_t kPreviewTargetPoints = 1000000ull;
// 注意：逐帧 LOD 的最低目标点数 kMinLodPoints **不在本文件**——它是抽稀策略而非
// 格式属性（文件里只记录每帧实际生成了几级，故级别数可变、不影响兼容性）。
// 它已移到 IsfVoxel.h，与渲染侧 CloudRenderData 共用同一个门槛。
/** @brief 默认每页帧数（页 = 文件中的一块连续区间） */
constexpr uint32_t kDefaultFramesPerPage = 4096;
/** @brief 默认每 chunk 帧数（chunk = 渲染期一个 Drawable 覆盖的帧段） */
constexpr uint32_t kDefaultChunkFrames   = 256;

#pragma pack(push, 1)

/**
 * @brief .isf 文件头（写盘时补齐到 kHeaderBytes 字节）
 *
 * 只含"整文件"级别的信息；逐帧信息在 .isf.idx 里。世界包围盒与位姿原点
 * 仅作参考信息：位姿的权威来源始终是运行时的 g2o 图。
 */
struct FileHeader {
    char     magic[8];        ///< kMagic
    uint32_t version;         ///< kVersion
    uint32_t headerBytes;     ///< 本头实际占用（数据区起始偏移）
    uint32_t l0Stride;        ///< L0 每点字节数（kL0Stride）
    uint32_t previewStride;   ///< 预览块每点字节数（kPreviewStride）
    uint32_t lodLevelCount;   ///< 本文件实际使用的级别数（含 L0）
    uint32_t framesPerPage;   ///< 每页帧数
    uint32_t chunkFrames;     ///< 每 chunk 帧数
    uint64_t frameCount;      ///< 帧总数
    uint64_t pageCount;       ///< 页总数
    uint64_t totalPoints;     ///< 全量点数（所有帧 L0 之和）
    uint64_t previewOffset;   ///< 预览块在 .isf 中的字节偏移
    uint64_t previewPoints;   ///< 预览块点数
    uint64_t dataBytes;       ///< 数据区（含预览块）总字节数
    double   originShift[3];  ///< 建议的位姿原点（打包时首帧优化位姿的平移）
    double   worldMin[3];     ///< 打包时按优化位姿统计的世界包围盒最小值（参考）
    double   worldMax[3];     ///< 同上，最大值（参考）
};

/**
 * @brief 逐帧记录（.isf.idx 的 FrameRecord 数组元素）
 *
 * `l0Offset` + `pointCount` 即"帧 → 点范围"映射：给定帧在 .isf 中的
 * 顶点字节区间为 [l0Offset, l0Offset + pointCount * l0Stride)。
 */
struct FrameRecord {
    int64_t  frameId;                  ///< 关键帧顶点 ID（g2o vertex id）
    uint64_t l0Offset;                 ///< 该帧 L0 顶点块字节偏移
    uint32_t pointCount;               ///< L0 点数
    uint32_t pageId;                   ///< 所属页号（= frameIndex / framesPerPage）
    uint32_t chunkId;                  ///< 页内 chunk 序号
    uint16_t chunkLocalFrame;          ///< chunk 内帧序号（渲染期 aFrameId）
    uint16_t lodCount;                 ///< 实际生成级别数（含 L0，1..kMaxLodLevels）
    uint64_t lodOffset[kMaxLodLevels]; ///< 各级索引块偏移（[0] 未用，填 0）
    uint32_t lodCounts[kMaxLodLevels]; ///< 各级点数（[0] = pointCount）
    float    localAABB[6];             ///< 帧局部系 AABB：minX,minY,minZ,maxX,maxY,maxZ
    float    poseOdom[16];             ///< 原始（里程计）位姿，数学列主序 float
};

/**
 * @brief 逐页记录（.isf.idx 的 PageRecord 数组元素）
 *
 * 成员关系与字节范围完全由帧序号推导，因此位姿优化不会使其失效。
 */
struct PageRecord {
    uint32_t pageId;          ///< 页号
    uint32_t firstFrameIndex; ///< 在 FrameRecord 数组中的起始下标
    uint32_t frameCount;      ///< 本页帧数
    uint32_t reserved;        ///< 对齐保留
    uint64_t fileOffset;      ///< 本页首块在 .isf 中的字节偏移
    uint64_t fileBytes;       ///< 本页数据字节数
};

/**
 * @brief 全局粗预览块的单点记录（kPreviewStride = 16 字节）
 *
 * 预览块是全图按固定步长均匀采样得到的一份常驻点集（目标 100 万点 / 16 MB），
 * 承担三件事：远景整体轮廓、**保证任意被高亮的帧都至少有一部分点可见**
 * （即使该帧所在页未驻留）、以及拾取失败时的兜底。
 *
 * 与其他块不同，预览点自带 `frameIndex`（FrameRecord 数组下标），
 * 因此着色器可以对同一块内的点施加各自不同的位姿。
 */
struct PreviewPoint {
    float    x, y, z;    ///< 帧局部系坐标
    uint32_t frameIndex; ///< 该点所属帧在 FrameRecord 数组中的下标
};

/** @brief .isf.idx 文件头（写盘时补齐到 kIndexHeaderBytes 字节） */
struct IndexHeader {
    char     magic[8];        ///< kIndexMagic
    uint32_t version;         ///< kVersion
    uint32_t headerBytes;     ///< 本头实际占用（FrameRecord 数组起始偏移）
    uint32_t lodLevelCount;   ///< 与 FileHeader 一致
    uint32_t framesPerPage;   ///< 与 FileHeader 一致
    uint32_t chunkFrames;     ///< 与 FileHeader 一致
    uint32_t reserved;        ///< 对齐保留
    uint64_t frameCount;      ///< 帧总数
    uint64_t pageCount;       ///< 页总数
    uint64_t totalPoints;     ///< 全量点数
    int64_t  firstFrameId;    ///< 最小帧 ID（日志/校验用）
    int64_t  lastFrameId;     ///< 最大帧 ID
};

#pragma pack(pop)

static_assert(sizeof(FileHeader) <= kHeaderBytes, "FileHeader 超出预留头长度");
static_assert(sizeof(IndexHeader) <= kIndexHeaderBytes, "IndexHeader 超出预留头长度");
// 逐字段求和（pack(1) 下无填充）：
//   frameId 8 + l0Offset 8 + 3×uint32 12 + 2×uint16 4
//   + lodOffset 32 + lodCounts 16 + localAABB 24 + poseOdom 64 = 168
static_assert(sizeof(FrameRecord) == 8 + 8 + 4 * 3 + 2 * 2 +
                                       8 * kMaxLodLevels + 4 * kMaxLodLevels +
                                       24 + 64,
              "FrameRecord 布局被编译器改动了（#pragma pack 失效？）");
static_assert(sizeof(PageRecord) == 4 * 4 + 8 * 2, "PageRecord 布局异常");
static_assert(sizeof(PreviewPoint) == kPreviewStride, "PreviewPoint 步长与格式不一致");

/** @brief 页号推导：序号 → 页号（与位姿无关，永不失效） */
inline uint32_t pageIdOf(uint64_t frameIndex, uint32_t framesPerPage) {
    return static_cast<uint32_t>(frameIndex / framesPerPage);
}

/** @brief chunk 号推导：序号 → 页内 chunk 号 */
inline uint32_t chunkIdOf(uint64_t frameIndex, uint32_t framesPerPage,
                          uint32_t chunkFrames) {
    return static_cast<uint32_t>((frameIndex % framesPerPage) / chunkFrames);
}

/** @brief chunk 内帧序号推导（渲染期着色器的 aFrameId） */
inline uint16_t chunkLocalFrameOf(uint64_t frameIndex, uint32_t chunkFrames) {
    return static_cast<uint16_t>(frameIndex % chunkFrames);
}

}  // namespace isf
}  // namespace hdl_graph_slam
