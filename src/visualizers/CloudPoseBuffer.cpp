// ============================================================================
// CloudPoseBuffer.cpp
// CloudPoseBuffer 实现
// ============================================================================

#include "visualizers/CloudPoseBuffer.h"

// kInternalFormatRGBA32F（含"为什么用字面量而不是 GL_RGBA32F"的完整说明）统一在
// CoreShaders.h —— 位姿纹理与 Turbo 色表用的是同一个格式，只应有一处定义。
#include "visualizers/CoreShaders.h"

#include <osg/GL>
#include <osg/Texture>

#include <cstddef>
#include <cstdint>

namespace hdl_graph_slam {

namespace {

/** @brief 每帧的 texel 数（16 float / 每 texel 4 float） */
constexpr size_t kTexelsPerPose = CloudPoseTable::kFloatsPerPose / 4;

// kInternalFormatRGBA32F 的定义与完整理由已移到 CoreShaders.h（位姿纹理与
// Turbo 色表共用同一个格式，只应有一处定义）。

}  // namespace

size_t CloudPoseBuffer::slotBytes() const {
    return m_frameCount * CloudPoseTable::kFloatsPerPose * sizeof(float);
}

osg::TextureBuffer* CloudPoseBuffer::texture(uint32_t slot) const {
    if (slot >= CloudPoseTable::kSlotCount) return nullptr;
    return m_slots[slot].texture.get();
}

void CloudPoseBuffer::ensureTexture(uint32_t slot, size_t frameCount) {
    Slot& s = m_slots[slot];
    const size_t texels = frameCount * kTexelsPerPose;

    if (s.buffer.valid() && s.buffer->size() == texels) return;  // 尺寸未变，复用

    s.buffer = new osg::Vec4Array(texels);
    if (!s.texture.valid()) {
        s.texture = new osg::TextureBuffer;
        // RGBA32F：每 texel 16 B。显式指定内部格式会同时把
        // internalFormatMode 设为 USE_USER_DEFINED_FORMAT，从而**绕开**
        // TextureBuffer::computeInternalFormat() 的自动推导 —— 是本文件需要的
        // 确定性行为（见 kInternalFormatRGBA32F 的说明）。
        s.texture->setInternalFormat(kInternalFormatRGBA32F);
        s.texture->setFilter(osg::Texture::MIN_FILTER, osg::Texture::NEAREST);
        s.texture->setFilter(osg::Texture::MAG_FILTER, osg::Texture::NEAREST);
    }
    // textureWidth 对 GL_TEXTURE_BUFFER 而言就是 texel 数
    s.texture->setTextureWidth(static_cast<int>(texels));
    s.texture->setBufferData(s.buffer.get());
}

void CloudPoseBuffer::fillSlot(uint32_t slot, const float* slotPoses,
                              size_t frameCount) {
    Slot& s = m_slots[slot];
    if (!s.buffer.valid()) return;
    // 位姿表的内存顺序就是数学列主序的 16 个 float，每 4 个一组正好一个 vec4，
    // 因此这里是纯顺序搬运，不做任何转置（见设计文档 §4.4.4 的结论 A）。
    for (size_t i = 0; i < frameCount; ++i) {
        const float* p = slotPoses + i * CloudPoseTable::kFloatsPerPose;
        // 每帧占 4 个连续 texel —— 下标必须是 i*4 + k（写成 i*0+k 会反复覆写
        // 第 0 帧的 texel，把整张位姿表写坏）
        (*s.buffer)[i * 4 + 0].set(p[0], p[1], p[2], p[3]);
        (*s.buffer)[i * 4 + 1].set(p[4], p[5], p[6], p[7]);
        (*s.buffer)[i * 4 + 2].set(p[8], p[9], p[10], p[11]);
        (*s.buffer)[i * 4 + 3].set(p[12], p[13], p[14], p[15]);
    }
    s.buffer->dirty();  // 触发该槽纹理的重新上传
}

size_t CloudPoseBuffer::rebuild(const CloudPoseTable& poses) {
    const size_t frameCount = poses.frameCount();
    m_frameCount = frameCount;
    if (frameCount == 0) {
        m_lastUploadBytes = 0;
        return 0;
    }

    const float* base = poses.data().data();
    const size_t slotFloats = poses.slotFloats();

    for (uint32_t slot = 0; slot < CloudPoseTable::kSlotCount; ++slot) {
        ensureTexture(slot, frameCount);
        fillSlot(slot, base + static_cast<size_t>(slot) * slotFloats, frameCount);
    }

    m_lastUploadBytes = slotBytes() * CloudPoseTable::kSlotCount;
    return m_lastUploadBytes;
}

size_t CloudPoseBuffer::updateOptimizedSlot(const CloudPoseTable& poses) {
    const size_t frameCount = poses.frameCount();

    // 帧集合变了（新增/删除关键帧）→ 槽位布局失效，退回全量重建
    if (frameCount != m_frameCount || !m_slots[CloudPoseTable::kSlotOptimized].buffer.valid()) {
        return rebuild(poses);
    }
    if (frameCount == 0) {
        m_lastUploadBytes = 0;
        return 0;
    }

    // 只碰优化槽：上传量 = 帧数 × 64 B
    fillSlot(CloudPoseTable::kSlotOptimized, poses.data().data(), frameCount);
    m_lastUploadBytes = slotBytes();
    return m_lastUploadBytes;
}

}  // namespace hdl_graph_slam
