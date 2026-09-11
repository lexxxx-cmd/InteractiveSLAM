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
 * decimateToTarget() 已不再调用本函数——它改用 decimateToLeaf 的产出长度当点数，
 * 一趟遍历同时得到"当前点数"与候选结果，比"先数一遍再抽一遍"省一趟。本函数保留
 * 给只需要计数、不需要索引的诊断/校验场景（如工具里的结构自检）。
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
 * @brief 从帧局部 AABB 与目标点数反推**初始**体素边长（仅作搜索种子）
 *
 * 刻意偏大（×2）：首轮体素数偏小，避免极端分布下临时哈希表占用过大内存。
 * 与 PointCloudBuilder::estimateLeafFromBounds() 同思路。
 *
 * 注意它只是种子、不是答案：该式假设点均匀填满整个 AABB 体积，而激光扫描点云
 * 近似二维流形，算出的边长往往偏大近一个量级。因此 decimateToTarget() 从这个
 * 种子出发做双向搜索——种子偏大在那边是无害的。
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
 * @brief 自适应抽稀到目标点数：在 (log 边长, log 点数) 空间双向搜索体素边长
 *
 * @param xyz         连续存放的 float3 数组
 * @param count       点数
 * @param targetCount 目标点数（>=1）
 * @param lo,hi       帧局部 AABB（仅用于反推搜索种子）
 * @param outIndices  输出：被保留点的升序索引
 * @return 实际使用的体素边长
 *
 * 为什么必须**双向**：初始边长由"包围盒体积 / 目标点数"反推（见
 * estimateInitialLeaf），该式假设点均匀填满整个 AABB 体积；而激光扫描点云近似
 * 二维流形，体积法会把边长估大近一个量级。旧实现的迭代只会**放大**边长
 * （c <= target 即 break），所以初值一旦偏大就永远纠正不回来——实测整图 LOD1
 * 目标 39,736,974 点只出 75,141 点，差 528 倍。
 *
 * 现在改为：每趟试探直接用 decimateToLeaf 的产出长度当"当前点数"（体素数 ==
 * 保留点数），于是一趟遍历同时得到点数和候选结果；再用最近两趟在 log-log 空间
 * 割线外推到目标——体素数对边长近似幂律 c ≈ A·leaf^(-d)，故 2~3 趟即收敛。
 * 两个方向都能走，偏大偏小都会被纠正。
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
    if (targetCount < 1) targetCount = 1;
    const double tgt = static_cast<double>(targetCount);

    std::vector<uint32_t> probe;
    auto measure = [&](float leaf) -> size_t {
        decimateToLeaf(xyz, count, leaf, probe);
        return probe.size();
    };
    // 选优准则："不超过目标且最大"优先（从下方逼近目标）；都不满足时取最小
    auto better = [&](size_t a, size_t b) -> bool {
        const bool aOk = (a > 0 && a <= targetCount);
        const bool bOk = (b > 0 && b <= targetCount);
        if (aOk != bOk) return aOk;
        return aOk ? (a > b) : (a < b);
    };

    float leaf = estimateInitialLeaf(lo, hi, targetCount);  // 刻意偏大的种子
    size_t c = measure(leaf);
    float  bestLeaf  = leaf;
    size_t bestCount = c;

    float  prevLeaf  = 0.0f;
    size_t prevCount = 0;
    bool   havePrev  = false;

    for (int iter = 0; iter < 8; ++iter) {
        if (c > 0 && better(c, bestCount)) { bestLeaf = leaf; bestCount = c; }
        if (bestCount == targetCount) break;  // 精确命中
        // 已经从下方进入目标 2% 以内就收手，避免为几个点反复遍历
        if (bestCount > 0 && bestCount <= targetCount &&
            static_cast<double>(targetCount - bestCount) <= tgt * 0.02) {
            break;
        }
        if (c == 0) break;  // 防御：边长过大导致无输出，沿用 best

        // 下一趟的边长：有两点就做 log-log 割线外推；只有一点时按二维流形的
        // 幂律 c ∝ leaf^-2 先跳一步（偏差大时这一跳比二分快得多）
        double next = 0.0;
        if (havePrev && prevCount != c && prevLeaf != leaf) {
            const double x1 = std::log(static_cast<double>(prevLeaf));
            const double y1 = std::log(static_cast<double>(prevCount) + 1.0);
            const double x2 = std::log(static_cast<double>(leaf));
            const double y2 = std::log(static_cast<double>(c) + 1.0);
            const double dy = y2 - y1;
            if (std::fabs(dy) > 1e-9) {
                const double slope = (x2 - x1) / dy;  // d(log leaf) / d(log count)
                next = std::exp(x2 + slope * (std::log(tgt + 1.0) - y2));
            } else {
                next = static_cast<double>(leaf) * 1.5;  // 退化：该区间无分辨率
            }
        } else {
            next = static_cast<double>(leaf) *
                   std::pow(static_cast<double>(c) / tgt, 0.5);
        }

        prevLeaf = leaf; prevCount = c; havePrev = true;

        // 防外推发散：限制单步幅度，并保底一个极小边长
        const double loBound = static_cast<double>(leaf) / 16.0;
        const double hiBound = static_cast<double>(leaf) * 16.0;
        if (!(next > loBound)) next = loBound;
        if (next > hiBound) next = hiBound;
        if (next < 1e-4) next = 1e-4;

        const float nextLeaf = static_cast<float>(next);
        if (nextLeaf == leaf) break;  // float 精度已经推不动了
        leaf = nextLeaf;
        c = measure(leaf);
    }

    // probe 里是最后一趟的结果，未必是 best；若不同则按 bestLeaf 再算一次收尾
    if (bestLeaf != leaf) decimateToLeaf(xyz, count, bestLeaf, probe);
    outIndices.swap(probe);
    // 兜底：抽稀不应把一帧抽空（"每帧每级非空"是逐帧 LOD 与高亮语义的前提）
    if (outIndices.empty() && count > 0) outIndices.push_back(0);
    return bestLeaf;
}

}  // namespace isf
}  // namespace hdl_graph_slam
