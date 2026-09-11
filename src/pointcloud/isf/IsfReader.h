// ============================================================================
// IsfReader.h
// .isf / .isf.idx 读取器
//
// 两件事分开：
//   1. `.isf.idx` 全量读进内存（几 MB）—— 帧表与页表常驻，与显存驻留无关；
//      这是"拾取/包围盒/适配视图不依赖页驻留"的基础（设计文档 §4.6/§4.7）。
//   2. `.isf` 以只读方式 mmap —— 帧坐标块可以直接零拷贝上传为 VBO，
//      也可以按帧顺序读（`readFramePositions`），后者在 mmap 不可用时兜底。
//
// 帧表在打包时按 frameId 升序写入，因此 `indexOfFrame` 用二分查找，
// 不需要额外的 hash 表（3 万帧省下约 1 MB 且更快）。
// ============================================================================

#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "pointcloud/isf/IsfFormat.h"

namespace hdl_graph_slam {
namespace isf {

/**
 * @brief .isf 只读读取器
 *
 * 用法：
 *   IsfReader r;
 *   std::string err;
 *   if (!r.open("map/cloud.isf", &err)) { ... }
 *   for (size_t i = 0; i < r.frameCount(); ++i) {
 *       const FrameRecord& fr = r.frames()[i];   // 帧 → 点范围
 *       const float* p = r.framePositions(i);     // 零拷贝（mmap 可用时）
 *   }
 */
class IsfReader {
public:
    IsfReader() = default;
    ~IsfReader();
    IsfReader(const IsfReader&) = delete;
    IsfReader& operator=(const IsfReader&) = delete;

    /**
     * @brief 打开 .isf（同时按 `<isfPath>.idx` 找索引文件）
     * @param isfPath .isf 文件路径
     * @param error   失败原因（可空）
     * @return 成功返回 true
     */
    bool open(const std::string& isfPath, std::string* error = nullptr);

    /** @brief 关闭并释放 mmap / 文件句柄 */
    void close();

    /** @brief 是否已成功打开（索引已加载；即使 mmap 失败也算打开） */
    bool isOpen() const { return m_loaded; }

    /** @brief .isf 文件头 */
    const FileHeader& header() const { return m_header; }

    /** @brief 索引文件头 */
    const IndexHeader& indexHeader() const { return m_indexHeader; }

    /** @brief 帧表（按 frameId 升序） */
    const std::vector<FrameRecord>& frames() const { return m_frames; }

    /** @brief 页表（按 pageId 升序） */
    const std::vector<PageRecord>& pages() const { return m_pages; }

    /** @brief 帧数 */
    size_t frameCount() const { return m_frames.size(); }

    /** @brief 页数 */
    size_t pageCount() const { return m_pages.size(); }

    /** @brief 全量点数（所有帧 L0 之和） */
    uint64_t totalPoints() const { return m_header.totalPoints; }

    /** @brief .isf 路径 */
    const std::string& isfPath() const { return m_isfPath; }

    // ------------------------------------------------------------------
    // 帧 → 点范围（等价于现有 CloudRange 的语义）
    // ------------------------------------------------------------------

    /**
     * @brief 按 frameId 查帧表下标（二分；不存在返回 -1）
     *
     * 依据：打包时帧表按 frameId 升序写入（见 IsfWriter.cpp 的 std::sort）。
     */
    int64_t indexOfFrame(long frameId) const;

    /** @brief 按 frameId 取帧记录（不存在返回 nullptr） */
    const FrameRecord* findFrame(long frameId) const;

    // ------------------------------------------------------------------
    // 零拷贝 / 顺序读
    // ------------------------------------------------------------------

    /**
     * @brief 该帧 L0 顶点块指针（帧局部系 float3），mmap 不可用或越界返回 nullptr
     *
     * 返回的指针位于只读映射内，生命周期与本对象一致，**不要释放**。
     * 可直接用于 glBufferData/glBufferSubData 或复制进 VBO。
     */
    const float* framePositions(size_t frameIndex) const;

    /**
     * @brief 该帧某一级 LOD 的索引数组（指向本帧 L0 顶点，元素 < pointCount）
     * @param frameIndex 帧表下标
     * @param level      LOD 级别（1..lodCount-1；0 表示全量，无索引）
     * @param outCount   输出：索引个数
     * @return 指针；level 非法或越界返回 nullptr
     */
    const uint32_t* frameLodIndices(size_t frameIndex, uint32_t level,
                                    uint32_t& outCount) const;

    /** @brief 全局粗预览块指针（每个元素自带帧序号） */
    const PreviewPoint* previewPoints() const;

    /** @brief 全局粗预览块点数 */
    uint64_t previewPointCount() const { return m_header.previewPoints; }

    /**
     * @brief 顺序读某帧的 L0 坐标（mmap 不可用时的兜底路径）
     *
     * const 语义：本方法只移动内部读游标（`m_stream` 的 seek 位置），
     * 不改变读取器的**逻辑**状态（帧表/页表/映射都不变），因此声明为 const，
     * 以便校验器能以 `const IsfReader&` 调用。游标成员 `m_stream` 标为 mutable。
     *
     * @param frameIndex 帧表下标
     * @param out        输出 float3 数组（会被 resize 到 pointCount*3）
     * @return 成功返回 true
     */
    bool readFramePositions(size_t frameIndex, std::vector<float>& out) const;

    /**
     * @brief 顺序读某帧某一级 LOD 索引（兜底路径，const 语义同上）
     * @return 成功返回 true（level=0 时返回空数组并通过 outCount=0 表示全量）
     */
    bool readFrameLodIndices(size_t frameIndex, uint32_t level,
                             std::vector<uint32_t>& out, uint32_t& outCount) const;

    /** @brief 映射是否建立（false 时只能用 readFrame* 兜底路径） */
    bool hasMmap() const { return m_mapped != nullptr; }

private:
    /** @brief 只读读取 .isf.idx（填充 m_frames / m_pages / m_indexHeader） */
    bool loadIndex(const std::string& idxPath, std::string* error);

    std::string  m_isfPath;
    std::string  m_idxPath;

    FileHeader  m_header{};
    IndexHeader m_indexHeader{};
    std::vector<FrameRecord> m_frames;
    std::vector<PageRecord>  m_pages;

    // —— mmap ——
    const uint8_t* m_mapped = nullptr;   ///< 映射基址（nullptr = 未映射）
    uint64_t       m_mapSize = 0;        ///< 映射长度（越界检查用）
    bool           m_loaded = false;     ///< 索引是否已加载

    // —— 顺序读兜底（始终打开） ——
    // mutable：readFrame* 只移动读游标，属于实现细节而非逻辑状态，
    // 因此这两个方法可以声明为 const（校验器需要以 const 引用调用）。
    mutable void* m_stream = nullptr;    ///< std::ifstream*（避免头文件引入 <fstream>）

    // —— 平台句柄 ——
#ifdef _WIN32
    void* m_fileHandle = nullptr;        ///< HANDLE
    void* m_mapHandle  = nullptr;        ///< HANDLE
#else
    int   m_fd = -1;
#endif
};

}  // namespace isf
}  // namespace hdl_graph_slam
