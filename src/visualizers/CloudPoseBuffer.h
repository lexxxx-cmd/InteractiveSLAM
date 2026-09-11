// ============================================================================
// CloudPoseBuffer.h
// 位姿表 → OpenGL 纹理缓冲（samplerBuffer），供顶点着色器按帧索引取位姿
//
// 这是"位姿上移着色器"的传输层：CloudPoseTable（CPU，每帧 16 float）被搬进
// 一张 `GL_TEXTURE_BUFFER`，着色器用 `texelFetch(uPoseTable, 4*frame + k)`
// 取出 4 个 vec4 组成一个 mat4。
//
// **每个槽位一张独立纹理**（而不是一张大纹理 + uPoseOffset 偏移）：
//   · 优化后只需更新优化槽 → 上传量严格等于 `帧数 × 64 B`
//     （3 万帧 = 1.92 MB）；若两槽共用一张数组，OSG 的 dirty() 会把两槽
//     一起重传（3.84 MB），代价翻倍且与"位姿表极小"的论断不符；
//   · 双层显示因此不需要 uPoseOffset 参与着色器逻辑 —— 两个层各自把自己
//     槽位的纹理绑到同一个采样器单元即可，着色器更简单、更不容易错。
//
// 每帧 4 个 texel（12 B/texel 的 RGB32F 存不下 4×4，必须 RGBA32F = 16 B/texel）。
// 数学列主序的 16 个 float 正好按 4 个一组切成 4 个 vec4，**与
// CloudPoseTable::data() 的内存顺序完全一致**，所以填充就是顺序搬运。
// ============================================================================

#pragma once

#include <osg/Array>
#include <osg/TextureBuffer>
#include <osg/ref_ptr>

#include <cstddef>
#include <cstdint>

#include "visualizers/CloudRenderData.h"

namespace hdl_graph_slam {

/**
 * @brief 双槽位姿纹理（槽 0 = 优化位姿，槽 1 = 原始冻结位姿）
 *
 * 用法：
 *   CloudPoseBuffer buf;
 *   buf.rebuild(poses);                 // 加载/首次构建：两槽一次性上传
 *   ... 位姿优化后 ...
 *   buf.updateOptimizedSlot(poses);      // 只上传优化槽，O(帧数)
 */
class CloudPoseBuffer {
public:
    CloudPoseBuffer() = default;

    /**
     * @brief 从位姿表重建两槽纹理（加载地图/首次构建时调用）
     * @return 本次上传的字节数（两槽合计）
     */
    size_t rebuild(const CloudPoseTable& poses);

    /**
     * @brief 只更新优化槽位纹理（每次 g2o 优化完成后调用）
     *
     * 上传量 = 帧数 × 64 B，**与点数无关**。
     * 若位姿表的帧集合变了（新增/删除关键帧），自动退回全量 rebuild。
     *
     * @return 本次上传的字节数
     */
    size_t updateOptimizedSlot(const CloudPoseTable& poses);

    /**
     * @brief 取某槽的纹理（尚未构建时返回 nullptr）
     * @param slot CloudPoseTable::kSlotOptimized / kSlotOriginal
     */
    osg::TextureBuffer* texture(uint32_t slot) const;

    /** @brief 已构建的帧数（0 = 尚未构建） */
    size_t frameCount() const { return m_frameCount; }

    /** @brief 单槽字节数（= 帧数 × 64 B），用于日志与验收 */
    size_t slotBytes() const;

    /** @brief 最近一次上传的字节数 */
    size_t lastUploadBytes() const { return m_lastUploadBytes; }

private:
    /** @brief 一个槽位的 CPU 数组 + 纹理 */
    struct Slot {
        osg::ref_ptr<osg::Vec4Array>     buffer;   ///< 每帧 4 个 vec4
        osg::ref_ptr<osg::TextureBuffer> texture;
    };

    /** @brief 把位姿表中的某一槽顺序搬进对应 slot.buffer 并标脏 */
    void fillSlot(uint32_t slot, const float* slotPoses, size_t frameCount);

    /** @brief 建立（或保持）某槽的纹理对象与 buffer 绑定 */
    void ensureTexture(uint32_t slot, size_t frameCount);

    Slot   m_slots[CloudPoseTable::kSlotCount];
    size_t m_frameCount      = 0;
    size_t m_lastUploadBytes = 0;
};

}  // namespace hdl_graph_slam
