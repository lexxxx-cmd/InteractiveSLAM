// ============================================================================
// IsfVoxel.h
// 帧局部系体素抽稀 —— LOD 金字塔的构建原语（header-only）
//
// 关键性质：**在帧局部系做体素抽稀，等价于在世界系做同样边长的体素抽稀。**
// 刚性变换保距，所以局部系内间距 >= s 的点集合，施加 T 之后在世界系内间距
// 仍然 >= s，只是网格朝向随帧旋转。因此 LOD 金字塔一旦按局部系建好，就与
// 位姿无关、永不失效 —— 这正是消解"位姿优化导致全局空间索引失效"的关键。
//
// 体素键算法与 PointCloudBuilder.h 的 voxelKey()（FNV-1a 64 位）保持一致，
// 便于两处结果可比对。后续 Phase 5 清理时可将 PointCloudBuilder 也切到本文件。
// ============================================================================

#pragma once

#include <cmath>
#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace hdl_graph_slam {
namespace isf {

/**
 * @brief 三维整数体素坐标 → 64 位键（FNV-1a）
 *
 * 哈希碰撞概率极低；即使发生也只会合并两个相邻体素，视觉上无影响。
 * 与 PointCloudBuilder::voxelKey() 同算法。
 */
inline uint64_t voxelKey(float x, float y, float z, float invLeaf) {
    const int64_t ix = static_cast<int64_t>(std::floor(x * invLeaf));
    const int64_t iy = static_cast<int64_t>(std::floor(y * invLeaf));
    const int64_t iz = static_cast<int64_t>(std::floor(z * invLeaf));
    uint64_t h = 14695981039346656037ull;  // FNV-1a 64 位偏移基值
    h = (h ^ static_cast<uint64_t>(ix)) * 1099511628211ull;
    h = (h ^ static_cast<uint64_t>(iy)) * 1099511628211ull;
    h = (h ^ static_cast<uint64_t>(iz)) * 1099511628211ull;
    return h;
}

/**
 * @brief 统计给定体素边长下抽稀后的点数（≈体素数）
 *
 * @param xyz  连续存放的 float3 数组（帧局部系坐标）
 * @param count 点数
 * @param leaf 体素边长（米）
 */
inline size_t countVoxelsAtLeaf(const float* xyz, size_t count, float leaf) {
    const float inv = 1.0f / leaf;
    std::unordered_set<uint64_t> seen;
    seen.reserve(count < 4u * 1024u * 1024u ? count : 4u * 1024u * 1024u);
    for (size_t i = 0; i < count; ++i) {
        const float* p = xyz + 3 * i;
        seen.insert(voxelKey(p[0], p[1], p[2], inv));
    }
    return seen.size();
}

/**
 * @brief 按体素边长抽稀，每个体素保留**最先出现**的点
 *
 * 由于按输入顺序扫描并只在首次遇到某体素时记录，输出索引天然升序，
 * 因此"每帧至少保留一个点"以及"逐帧范围可切分"这两个性质都能保持。
 *
 * @param xyz        连续存放的 float3 数组（帧局部系坐标）
 * @param count      点数
 * @param leaf       体素边长（米）
 * @param outIndices 输出：被保留点在 [0, count) 内的升序索引
 */
inline void decimateToLeaf(const float* xyz, size_t count, float leaf,
                           std::vector<uint32_t>& outIndices) {
    outIndices.clear();
    if (count == 0) return;
    const float inv = 1.0f / leaf;
    std::unordered_map<uint64_t, uint32_t> kept;
    kept.reserve(count / 2 < 4096 ? 4096 : count / 2);
    outIndices.reserve(count / 2 + 1);
    for (size_t i = 0; i < count; ++i) {
        const float* p = xyz + 3 * i;
        const uint64_t k = voxelKey(p[0], p[1], p[2], inv);
        if (kept.emplace(k, static_cast<uint32_t>(i)).second) {
            outIndices.push_back(static_cast<uint32_t>(i));
        }
    }
    // 极端参数下的兜底：抽稀不应把一帧抽空
    if (outIndices.empty() && count > 0) outIndices.push_back(0);
}

/**
 * @brief 从帧局部 AABB 与目标点数反推初始体素边长
 *
 * 刻意偏大（×2）：首轮体素数偏小，避免极端分布下临时哈希表占用过大内存。
 * 与 PointCloudBuilder::estimateLeafFromBounds() 同思路。
 */
inline float estimateInitialLeaf(const float lo[3], const float hi[3],
                                 size_t target) {
    const double ex = static_cast<double>(hi[0]) - lo[0];
    const double ey = static_cast<double>(hi[1]) - lo[1];
    const double ez = static_cast<double>(hi[2]) - lo[2];
    double vol = ex * ey * ez;
    if (!(vol > 0.0)) vol = 1.0;
    const double budget = static_cast<double>(target > 1 ? target : 1);
    float leaf = static_cast<float>(std::cbrt(vol / budget));
    if (!(leaf > 1e-4f)) leaf = 0.1f;
    return leaf * 2.0f;
}

/**
 * @brief 自适应抽稀到目标点数：迭代放大体素边长直至点数不超过目标
 *
 * @param xyz         连续存放的 float3 数组
 * @param count       点数
 * @param targetCount 目标点数（>=1）
 * @param lo,hi       帧局部 AABB（用于反推初始边长）
 * @param outIndices  输出：被保留点的升序索引
 * @return 实际使用的体素边长
 */
inline float decimateToTarget(const float* xyz, size_t count, size_t targetCount,
                              const float lo[3], const float hi[3],
                              std::vector<uint32_t>& outIndices) {
    if (count <= targetCount) {
        // 无需抽稀：直接给出全量升序索引
        outIndices.resize(count);
        for (size_t i = 0; i < count; ++i) outIndices[i] = static_cast<uint32_t>(i);
        return 0.0f;
    }
    float leaf = estimateInitialLeaf(lo, hi, targetCount);
    for (int iter = 0; iter < 8; ++iter) {
        const size_t c = countVoxelsAtLeaf(xyz, count, leaf);
        if (c == 0 || c <= targetCount) break;  // 已满足目标
        const double ratio = static_cast<double>(c) / static_cast<double>(targetCount);
        leaf = static_cast<float>(leaf * std::pow(ratio, 0.5) * 1.03);
    }
    decimateToLeaf(xyz, count, leaf, outIndices);
    return leaf;
}

}  // namespace isf
}  // namespace hdl_graph_slam
