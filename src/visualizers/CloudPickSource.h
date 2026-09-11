// ============================================================================
// CloudPickSource.h
// 拾取用的**帧级**数据源 —— 与 GPU 驻留状态、Drawable 可见性彻底解耦
//
// 设计文档 §4.7：新架构下顶点是**帧局部系**、射线是世界系，原生相交器量出来的
// 交点全错（而且"看起来合理"，比没有命中更危险）。因此拾取不走 Drawable 的顶点，
// 而是按**帧**取数据：帧位姿 + 帧局部点 + 帧局部 AABB。
//
// 这正是拾取能"不受可见性/LOD 级别/是否驻留影响"的原因（§4.7 的三点）：
//   · LOD 切到 L2/L3 时，按全分辨率点数返回结果；
//   · 页被淘汰时照样精确（数据在文件/内存里，不在显存里）；
//   · 双层显示时能明确命中的是哪一层。
//
// 抽象出来的意义：Phase 4 用内存里的 CloudChunk 实现（ChunkPickSource，见下）；
// Phase 6 换成 mmap 的 .isf 帧表实现（CloudFrameStore），
// CloudRayIntersector 一行都不用改。
// ============================================================================

#pragma once

#include "visualizers/CloudRenderData.h"

#include <Eigen/Geometry>
#include <cstddef>
#include <cstdint>

namespace hdl_graph_slam {

/**
 * @brief 帧级拾取数据源（纯接口，与存储形态无关）
 *
 * 帧序约定与 CloudPoseTable 完全一致（**按 frameId 升序**），因此
 * `frameIndex` 可以直接喂给 `CloudPoseTable::pose()` / `indexOfFrame()`。
 */
class CloudPickSource {
public:
    virtual ~CloudPickSource() = default;

    /** @brief 帧数 */
    virtual size_t frameCount() const = 0;

    /** @brief 本数据源使用的位姿槽（kSlotOptimized / kSlotOriginal） */
    virtual uint32_t poseSlot() const = 0;

    /** @brief 第 frameIndex 帧的当前位姿 */
    virtual Eigen::Isometry3d framePose(size_t frameIndex) const = 0;

    /**
     * @brief 第 frameIndex 帧的帧局部点（float3 连续）及其局部 AABB
     *
     * @param frameIndex 帧序（0..frameCount()-1）
     * @param count      输出：点数（0 表示该帧无点云）
     * @param lo,hi      输出：帧局部 AABB（count == 0 时无意义）
     * @return 指向 count×3 个 float 的指针；无数据返回 nullptr
     *
     * 返回指针的生命周期由实现方保证（内存态为 chunk 内部数组，
     * 分页态为 mmap 映射区），拾取期间不得失效。
     */
    virtual const float* framePoints(size_t frameIndex, size_t& count,
                                     float lo[3], float hi[3]) const = 0;

    /**
     * @brief 层标识（0 = 优化层，1 = 原始层）
     *
     * 用于双层显示时向 UI 说明"命中的是哪一层"（Phase 4 验收第 4 条）。
     */
    virtual int layerId() const { return 0; }
};

/**
 * @brief 基于内存 CloudChunk 的帧级数据源（Phase 4 使用）
 *
 * 帧序 → (chunk, chunk 内序号) 是**纯算术**换算，依据是 CloudChunk 的不变量
 * `chunks[i].firstFrameIndex == i * chunkFrames` 且 `frames.size() == frameCount`
 * （零顶点帧也占一个 FrameRange）—— 见 CloudRenderData.h 的注释。
 * 因此不需要任何额外的帧索引表。
 */
class ChunkPickSource : public CloudPickSource {
public:
    /**
     * @param chunks      分块数组（buildChunks 的产物，须在拾取期间存活）
     * @param chunkFrames 每 chunk 帧数（ChunkBuildOptions::chunkFrames）
     * @param poses       位姿表（须在拾取期间存活）
     * @param slot        位姿槽（kSlotOptimized / kSlotOriginal）
     * @param layerId     层标识（0 = 优化层，1 = 原始层）
     */
    ChunkPickSource(const std::vector<CloudChunk>* chunks, uint32_t chunkFrames,
                    const CloudPoseTable* poses, uint32_t slot, int layerId = 0)
        : m_chunks(chunks),
          m_chunkFrames(chunkFrames ? chunkFrames : 1u),
          m_poses(poses),
          m_slot(slot),
          m_layer(layerId) {}

    size_t frameCount() const override {
        return m_poses ? m_poses->frameCount() : 0;
    }

    uint32_t poseSlot() const override { return m_slot; }

    int layerId() const override { return m_layer; }

    Eigen::Isometry3d framePose(size_t frameIndex) const override {
        if (!m_poses) return Eigen::Isometry3d::Identity();
        // 刻意走 pose() 而不是另存一份 double 位姿：拾取必须与**渲染所见一致**，
        // 而渲染用的就是这张 float 位姿表。另存一份 double 会让拾取与画面
        // 在大范围地图上出现亚毫米级不一致（虽小，但没有理由引入）。
        return m_poses->pose(m_slot, frameIndex);
    }

    const float* framePoints(size_t frameIndex, size_t& count,
                             float lo[3], float hi[3]) const override {
        count = 0;
        if (!m_chunks || !m_poses) return nullptr;
        if (frameIndex >= frameCount()) return nullptr;

        const size_t ci = frameIndex / m_chunkFrames;
        const size_t li = frameIndex % m_chunkFrames;
        if (ci >= m_chunks->size()) return nullptr;

        const CloudChunk& c = (*m_chunks)[ci];
        if (li >= c.frames.size()) return nullptr;

        const CloudChunk::FrameRange& fr = c.frames[li];
        for (int k = 0; k < 3; ++k) {
            lo[k] = fr.localAABB[k];
            hi[k] = fr.localAABB[3 + k];
        }
        if (fr.vertexCount == 0) return nullptr;

        const size_t off = static_cast<size_t>(fr.startVertex) * 3;
        if (off + static_cast<size_t>(fr.vertexCount) * 3 > c.positions.size()) {
            count = 0;   // 结构不一致，按"无点"处理而不是越界读
            return nullptr;
        }
        count = fr.vertexCount;
        return c.positions.data() + off;
    }

private:
    const std::vector<CloudChunk>* m_chunks = nullptr;
    size_t   m_chunkFrames = 1;
    const CloudPoseTable* m_poses = nullptr;
    uint32_t m_slot  = CloudPoseTable::kSlotOptimized;
    int      m_layer = 0;
};

}  // namespace hdl_graph_slam
