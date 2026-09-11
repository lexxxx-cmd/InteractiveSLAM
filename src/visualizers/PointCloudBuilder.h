// ============================================================================
// PointCloudBuilder.h
// 点云构建器 —— 后台线程纯计算单元
//
// 功能：将 InteractiveGraph 的关键帧点云构建为可直接交换进
//       KeyframePointCloudVisualizer 的渲染数据。本类只做 CPU 计算，
//       不触碰 OSG 场景图 / 渲染状态，因此可以在任意工作线程执行，
//       用于"点云重建后台化"（避免上千万点重建阻塞 UI 线程）。
//
// 构建流程：
//   1. 加锁快照：在 optimization_mutex 保护下，将每个关键帧的位姿
//      （v->estimate()）与点云指针（ConstPtr，内容不可变）快速拷出，
//      锁只覆盖毫秒级快照，不阻塞优化/删边等图操作；
//   2. 无锁变换：将每个关键帧的点按位姿变换到世界坐标系，统计
//      Z 值范围与包围盒，记录每帧全量范围；
//   3. 孤立杂点滤波（阶段 2.5）：全局细体素占据计数，仅被少于阈值个
//      点占据的格子判为杂点（多帧重叠互证）。全量数组与 cloudRanges
//      保持完整（索引锚点，选中高亮仍用全量），滤波只体现在派生的
//      主级别渲染索引/范围上——存活的点按原顺序组成升序子序列；
//   4. 主级别（全量或滤波后存活点）+ 一级 LOD：当 lodEnabled 时，在
//      主级别子集上生成第一层降采样（目标点数 N/2，不低于 5 万点），
//      同样映射回同一个全量点数组——渲染固定为"全量 + 第一层"两种；
//   5. 构建渲染顶点数组（osg::Vec3Array，仅普通容器填充，无 GL 调用）。
//
// 输出 PointCloudBuildResult 由主线程 KeyframePointCloudVisualizer::
// commitBuild() 以 swap 方式换入场景（O(1) 交换，旧几何体渲染到新数据
// 就绪，避免闪烁与撕裂）。
// ============================================================================

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include <Eigen/Geometry>
#include <osg/Array>
#include <pcl/point_types.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "visualizers/TurboColormap.h"

namespace hdl_graph_slam {

/**
 * @brief 每个关键帧点云的范围结构
 *
 * 用于快速定位每个关键帧对应的顶点范围，避免重新着色时遍历所有顶点。
 * 同时用于全量数据（m_cloudRanges）与渲染数据（m_renderRanges）。
 */
struct CloudRange {
    size_t startVertex;   ///< 该关键帧在（渲染/全量）顶点数组中的起始索引
    size_t vertexCount;   ///< 该关键帧的（渲染/全量）顶点数量
    long   vertexId;      ///< 关键帧对应的顶点 ID
};

/**
 * @brief 单个 LOD 级别（level 0 = 全量 / 预算降采样主级别）
 *
 * 每级渲染数据都映射回同一个全量点数组（renderIndices / renderRanges），
 * 因此选中高亮、Z 统计等在任意级别均正确。
 * 顶点/颜色按固定块大小拆分（分块上传，避免单次超大 VBO 上传卡顿），
 * 逻辑索引连续：第 i 个渲染点位于 chunk = i / chunkSize，
 * 块内偏移 = i % chunkSize（chunkSize 由 builder 常量决定）。
 * colors 由 builder 按高程 Turbo 着色 + 透明度生成（后台线程完成）。
 */
struct LodLevel {
    std::vector<osg::ref_ptr<osg::Vec3Array>> vertexChunks;  ///< 分块顶点数组
    std::vector<osg::ref_ptr<osg::Vec4Array>> colorChunks;   ///< 分块颜色数组
    std::vector<size_t>     renderIndices;      ///< 渲染索引 -> 全量索引（level0 全量时为空）
    std::vector<CloudRange> renderRanges;       ///< 每个关键帧在该级渲染数组中的范围
    bool renderFullRes = true;                  ///< true = 渲染索引与全量索引一致
};

/**
 * @brief 点云构建选项
 */
struct BuildOptions {
    bool lodEnabled = false;    ///< 是否生成第一层 LOD（渲染固定为全量 + 第一层）
    bool showOriginalLayer = false; ///< 是否生成原始层（里程计位姿点云，参照底图）

    // —— 孤立杂点滤波（全局体素占据计数，见 build() 阶段 2.5） ——
    bool  outlierFilterEnabled = true;  ///< 是否启用孤立杂点滤波
    float outlierFilterLeaf    = 0.10f; ///< 滤波体素边长（米）
    int   outlierFilterMinPts  = 2;     ///< 体素内点数低于该值判为孤立杂点

    // —— 颜色生成参数（构建时直接生成颜色，主线程 commit 时零遍历） ——
    bool  useAutoColorRange = true;  ///< true = 用构建统计的 zMin/zMax 映射颜色
    float colorZMin = 0.0f;          ///< 手动颜色范围下限（useAutoColorRange=false 时生效）
    float colorZMax = 1.0f;          ///< 手动颜色范围上限（useAutoColorRange=false 时生效）
    float opacity   = 1.0f;          ///< 点云透明度（per-vertex alpha）
};

/**
 * @brief 点云构建结果（纯数据，可在线程间移动）
 *
 * 由 PointCloudBuilder::build() 在工作线程生成，
 * 由主线程 KeyframePointCloudVisualizer::commitBuild() 消费。
 */
struct PointCloudBuildResult {
    std::vector<Eigen::Vector3d,
                Eigen::aligned_allocator<Eigen::Vector3d>> allWorldPoints; ///< 全量世界坐标点
    std::vector<CloudRange> cloudRanges;    ///< 每个关键帧的全量范围
    std::vector<size_t>     renderIndices;  ///< 主级别渲染索引（降采样时使用）
    std::vector<CloudRange> renderRanges;   ///< 主级别逐帧渲染范围
    bool renderFullRes = true;              ///< 主级别是否全量

    std::vector<osg::ref_ptr<osg::Vec3Array>> vertexChunks;  ///< 主级别分块顶点数组
    std::vector<osg::ref_ptr<osg::Vec4Array>> colorChunks;   ///< 主级别分块颜色数组

    std::vector<LodLevel> lodLevels;        ///< LOD 级别（不含主级别，至多一级）

    // —— 原始层（里程计位姿变换的点云，参照底图，可选） ——
    // allOdomWorldPoints 与 allWorldPoints 逐点对齐（同一帧、同一局部点顺序），
    // 因此各级渲染索引可直接复用优化层的选取结果。
    std::vector<Eigen::Vector3d,
                Eigen::aligned_allocator<Eigen::Vector3d>> allOdomWorldPoints; ///< 里程计位姿世界坐标点
    std::vector<LodLevel> odomLevels;       ///< 原始层各级别（[0]=主级别镜像，其后与 lodLevels 一一对应）

    float zMin =  std::numeric_limits<float>::max();  ///< 数据 Z 最小值
    float zMax = -std::numeric_limits<float>::max();  ///< 数据 Z 最大值
    Eigen::Vector3d bMin;  ///< 世界包围盒最小值
    Eigen::Vector3d bMax;  ///< 世界包围盒最大值

    // —— 构建时所用的颜色参数（commit 后用于判断是否需要重新着色） ——
    bool  colorUsedAuto = true;  ///< 构建时是否使用自动颜色范围
    float colorZMinUsed = 0.0f;  ///< 构建时的颜色映射 Z 下限
    float colorZMaxUsed = 1.0f;  ///< 构建时的颜色映射 Z 上限
    float opacityUsed   = 1.0f;  ///< 构建时的透明度
};

/**
 * @brief 点云构建器
 *
 * 将 InteractiveGraph 的关键帧点云构建为 PointCloudBuildResult。
 * 纯计算、无 QObject / OSG 场景图依赖，可安全运行在 QtConcurrent 工作线程。
 */
class PointCloudBuilder {
public:
    using PointT = pcl::PointXYZI;  ///< 点类型（XYZ + 强度）

    /**
     * @brief 分块大小（渲染点数/块）
     *
     * 每块约 64 万点（顶点 12B + 颜色 16B ≈ 18MB），提交后逐帧上传，
     * 避免单次超大 VBO 上传造成的一帧卡顿。块间逻辑索引连续。
     */
    static constexpr size_t kChunkPoints = 640000;

    /**
     * @brief 构建点云渲染数据（可在后台线程调用）
     * @param graph   交互式图数据（读取 keyframes 与位姿）
     * @param options 构建选项（LOD 开关）
     * @return 构建结果（全量点 + 渲染索引/范围 + 顶点数组 + LOD 级别 + 统计）
     *
     * 线程安全：开始时在 optimization_mutex 保护下做毫秒级位姿/点云
     * 快照，之后无锁处理（点云内容加载后不可变）。
     */
    static PointCloudBuildResult build(
        const std::shared_ptr<InteractiveGraph>& graph, const BuildOptions& options) {
        PointCloudBuildResult r;
        r.bMin = Eigen::Vector3d( std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max());
        r.bMax = Eigen::Vector3d(-std::numeric_limits<double>::max(),
                                 -std::numeric_limits<double>::max(),
                                 -std::numeric_limits<double>::max());

        if (!graph) return r;

        // ---- 阶段 1：加锁快照（毫秒级，不长时间占用图锁） ----
        struct FrameSnapshot {
            long id;  ///< 关键帧对应的顶点 ID
            Eigen::Isometry3d pose;        ///< 当前优化估计位姿
            Eigen::Isometry3d odomPose;    ///< 里程计（原始）位姿
            pcl::PointCloud<PointT>::ConstPtr cloud;
        };
        std::vector<FrameSnapshot, Eigen::aligned_allocator<FrameSnapshot>> frames;
        {
            std::lock_guard<std::mutex> lock(graph->optimization_mutex);
            for (auto& [id, kf] : graph->keyframes) {
                auto* v = dynamic_cast<g2o::VertexSE3*>(kf->node);
                if (!v || !kf->cloud || kf->cloud->empty()) continue;
                frames.push_back({id, v->estimate(), kf->odom, kf->cloud});
            }
        }

        // ---- 阶段 2：无锁变换到世界坐标系 ----
        // 优化层（estimate 位姿）与原始层（odom 位姿）逐点同步生成，
        // 两层点序完全一致——原始层无需独立的索引体系。
        for (const auto& frame : frames) {
            size_t start = r.allWorldPoints.size();
            for (const auto& pt : frame.cloud->points) {
                Eigen::Vector3d local(pt.x, pt.y, pt.z);
                Eigen::Vector3d wp = frame.pose * local;
                float wz = static_cast<float>(wp.z());
                if (wz < r.zMin) r.zMin = wz;
                if (wz > r.zMax) r.zMax = wz;
                if (wp.x() < r.bMin.x()) r.bMin.x() = wp.x();
                if (wp.y() < r.bMin.y()) r.bMin.y() = wp.y();
                if (wp.z() < r.bMin.z()) r.bMin.z() = wp.z();
                if (wp.x() > r.bMax.x()) r.bMax.x() = wp.x();
                if (wp.y() > r.bMax.y()) r.bMax.y() = wp.y();
                if (wp.z() > r.bMax.z()) r.bMax.z() = wp.z();
                r.allWorldPoints.push_back(wp);
                if (options.showOriginalLayer) {
                    r.allOdomWorldPoints.push_back(frame.odomPose * local);
                }
            }
            r.cloudRanges.push_back({start, r.allWorldPoints.size() - start, frame.id});
        }

        if (r.allWorldPoints.empty()) {
            r.vertexChunks.emplace_back(new osg::Vec3Array);  // 空结果也提供空块
            r.colorChunks.emplace_back(new osg::Vec4Array);
            return r;
        }

        // 防止 Z 值范围过小导致的颜色映射异常
        if (r.zMax - r.zMin < 0.001f) {
            r.zMin -= 0.5f;
            r.zMax += 0.5f;
        }

        // ---- 阶段 2.5：全局体素占据滤波（剔除孤立杂点） ----
        // 在全量世界坐标上按细体素统计每格点数：占据点数低于阈值的格子
        // 判为孤立杂点（多帧重叠区域互相印证，真实表面计数 >= 阈值；
        // 传感器串扰/飘点通常只出现一次）。全量数组与 cloudRanges 保持
        // 完整（它们是索引体系的锚点，选中高亮仍用全量数据），滤波只
        // 体现在派生层：存活点按原顺序组成主级别 renderIndices——对有序
        // 序列过滤仍是升序子序列，逐帧范围一遍循环即可重切。
        const size_t n = r.allWorldPoints.size();
        std::vector<size_t> mainIndices;    // 主级别渲染索引（空 = 全量直通）
        std::vector<CloudRange> mainRanges; // 主级别逐帧范围（空 = 用 cloudRanges）
        if (options.outlierFilterEnabled && options.outlierFilterMinPts > 1) {
            const float invLeaf = 1.0f / options.outlierFilterLeaf;
            std::unordered_map<uint64_t, int> occ;
            occ.reserve(std::min<size_t>(n, 4u * 1024u * 1024u));
            for (const auto& p : r.allWorldPoints) {
                ++occ[voxelKey(static_cast<float>(p.x()),
                               static_cast<float>(p.y()),
                               static_cast<float>(p.z()), invLeaf)];
            }

            // 先探测是否存在孤立体素，干净数据零额外开销直通
            bool hasIsolated = false;
            for (const auto& [key, count] : occ) {
                if (count < options.outlierFilterMinPts) { hasIsolated = true; break; }
            }
            if (hasIsolated) {
                std::vector<size_t> survivors;
                survivors.reserve(n);
                mainRanges.reserve(r.cloudRanges.size());
                for (const auto& range : r.cloudRanges) {
                    const size_t rstart = survivors.size();  // 该帧存活点段的起点
                    for (size_t j = range.startVertex;
                         j < range.startVertex + range.vertexCount; ++j) {
                        const auto& p = r.allWorldPoints[j];
                        auto it = occ.find(voxelKey(static_cast<float>(p.x()),
                                                    static_cast<float>(p.y()),
                                                    static_cast<float>(p.z()), invLeaf));
                        if (it != occ.end() && it->second >= options.outlierFilterMinPts)
                            survivors.push_back(j);
                    }
                    mainRanges.push_back(
                        {rstart, survivors.size() - rstart, range.vertexId});
                }
                // 安全兜底：极端参数下若全军覆没则放弃滤波（回退全量）
                if (!survivors.empty() && survivors.size() < n) {
                    mainIndices = std::move(survivors);
                } else {
                    mainRanges.clear();
                }
            }
        }

        // ---- 阶段 3：主级别（全量或滤波后的存活点，LOD 分级在阶段 5 生成） ----
        r.renderRanges = mainRanges.empty() ? r.cloudRanges : mainRanges;
        r.renderFullRes = mainIndices.empty();
        r.renderIndices = std::move(mainIndices);

        // ---- 阶段 4：构建主级别分块顶点/颜色数组 ----
        {
            const float czMin = options.useAutoColorRange ? r.zMin : options.colorZMin;
            const float czMax = options.useAutoColorRange ? r.zMax : options.colorZMax;
            buildChunkedArrays(r.allWorldPoints, r.renderIndices, r.renderFullRes,
                               czMin, czMax, options.opacity,
                               r.vertexChunks, r.colorChunks);
        }
        r.colorUsedAuto = options.useAutoColorRange;
        r.colorZMinUsed = options.useAutoColorRange ? r.zMin : options.colorZMin;
        r.colorZMaxUsed = options.useAutoColorRange ? r.zMax : options.colorZMax;
        r.opacityUsed   = options.opacity;

        // ---- 阶段 5：第一层 LOD ----
        // 目标点数 N/2（不低于 minLodPoints），仅生成一个降采样级别——
        // 渲染固定为"全量 + 第一层"两种。以主级别（滤波后存活点）为
        // 基数，在子集上抽稀；基数本就不大于目标点数时不生成任何级别。
        if (options.lodEnabled) {
            const size_t minLodPoints = 50000;
            const size_t baseCount = r.renderFullRes ? n : r.renderIndices.size();
            size_t target = baseCount / 2;
            if (target >= minLodPoints) {
                r.lodLevels.push_back(
                    buildLodLevel(r.allWorldPoints, r.renderRanges,
                                  r.renderIndices, r.renderFullRes,
                                  r.bMin, r.bMax,
                                  target, options, r.zMin, r.zMax));
            }
        }

        // ---- 阶段 6：原始层（里程计位姿点云，参照底图） ----
        // 与优化层逐点对齐，直接复用优化层的各级渲染索引/范围，
        // 仅以里程计世界坐标生成顶点/颜色数组。固定淡橙色，
        // 作为"未优化原始状态"的参照底图。
        if (options.showOriginalLayer && !r.allOdomWorldPoints.empty()) {
            r.odomLevels.reserve(1 + r.lodLevels.size());
            // 主级别镜像
            r.odomLevels.push_back(
                buildOdomLevel(r.allOdomWorldPoints, r.renderIndices, r.renderFullRes,
                               r.renderRanges, options));
            // LOD 级别镜像（与 r.lodLevels 一一对应）
            for (const auto& lod : r.lodLevels) {
                r.odomLevels.push_back(
                    buildOdomLevel(r.allOdomWorldPoints, lod.renderIndices,
                                   lod.renderFullRes, lod.renderRanges, options));
            }
        }

        return r;
    }

private:
    /**
     * @brief 构建原始层单个级别（复用优化层的渲染索引，换用里程计世界坐标）
     *
     * @param odomPoints    里程计位姿世界坐标点（与优化层全量点逐点对齐）
     * @param renderIndices 优化层该级别的渲染索引（renderFullRes 时忽略）
     * @param renderFullRes 优化层该级别是否全量直通
     * @param renderRanges  优化层该级别的逐帧渲染范围（原样复制）
     *
     * 原始层不参与降采样计算——降采样索引属于"选哪些点"的决策，
     * 两层点序一致故直接共享；颜色固定为淡橙色半透明参照底图。
     */
    static LodLevel buildOdomLevel(
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& odomPoints,
        const std::vector<size_t>& renderIndices,
        bool renderFullRes,
        const std::vector<CloudRange>& renderRanges,
        const BuildOptions& options) {
        LodLevel lod;
        lod.renderIndices = renderIndices;  // 复制（levels 持有自己的索引）
        lod.renderRanges = renderRanges;
        lod.renderFullRes = renderFullRes;

        const size_t count = renderFullRes ? odomPoints.size() : renderIndices.size();
        if (count == 0) return lod;

        // 淡橙色参照底图（与优化层 Turbo 着色形成色相区分）
        const osg::Vec4 gray(0.98f, 0.72f, 0.45f, options.opacity * 0.35f);

        const size_t chunkCount = (count + kChunkPoints - 1) / kChunkPoints;
        lod.vertexChunks.reserve(chunkCount);
        lod.colorChunks.reserve(chunkCount);
        for (size_t c = 0; c < chunkCount; ++c) {
            const size_t begin = c * kChunkPoints;
            const size_t end   = std::min(count, begin + kChunkPoints);
            auto* varr = new osg::Vec3Array;
            auto* carr = new osg::Vec4Array;
            varr->reserve(end - begin);
            carr->reserve(end - begin);
            for (size_t i = begin; i < end; ++i) {
                const Eigen::Vector3d& wp = odomPoints[renderFullRes ? i : renderIndices[i]];
                varr->push_back(osg::Vec3(static_cast<float>(wp.x()),
                                          static_cast<float>(wp.y()),
                                          static_cast<float>(wp.z())));
                carr->push_back(gray);
            }
            lod.vertexChunks.push_back(varr);
            lod.colorChunks.push_back(carr);
        }
        return lod;
    }

    /**
     * @brief 体素键哈希（FNV-1a 64 位）
     *
     * 将三维整数体素坐标混合为 64 位键。哈希碰撞概率极低，
     * 即使发生也只会合并两个相邻体素（视觉上无影响）。
     */
    static uint64_t voxelKey(float x, float y, float z, float invLeaf) {
        int64_t ix = static_cast<int64_t>(std::floor(x * invLeaf));
        int64_t iy = static_cast<int64_t>(std::floor(y * invLeaf));
        int64_t iz = static_cast<int64_t>(std::floor(z * invLeaf));
        uint64_t h = 14695981039346656037ull;  // FNV-1a 64 位偏移基值
        h = (h ^ static_cast<uint64_t>(ix)) * 1099511628211ull;
        h = (h ^ static_cast<uint64_t>(iy)) * 1099511628211ull;
        h = (h ^ static_cast<uint64_t>(iz)) * 1099511628211ull;
        return h;
    }

    /**
     * @brief 统计给定体素边长下子集的体素数（≈渲染点数）
     *
     * 遍历 ranges 描述的子集（主级别可能经过孤立杂点滤波，
     * 全量下标不再连续），体素数为该级降采样的实际点数估计。
     * 子集非全量时，range.startVertex 指向 subsetIndices 的位置，
     * 需经其解析为全量下标。
     */
    static size_t countVoxels(
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& pts,
        const std::vector<CloudRange>& ranges,
        const std::vector<size_t>& subsetIndices,
        bool subsetFullRes,
        float leaf) {
        const float inv = 1.0f / leaf;
        std::unordered_set<uint64_t> seen;
        seen.reserve(std::min<size_t>(pts.size(), 4u * 1024u * 1024u));
        for (const auto& range : ranges) {
            for (size_t i = 0; i < range.vertexCount; ++i) {
                const size_t j = subsetFullRes
                                     ? range.startVertex + i
                                     : subsetIndices[range.startVertex + i];
                seen.insert(voxelKey(static_cast<float>(pts[j].x()),
                                     static_cast<float>(pts[j].y()),
                                     static_cast<float>(pts[j].z()), inv));
            }
        }
        return seen.size();
    }

    /**
     * @brief 按给定体素边长构建降采样后的渲染索引与逐帧渲染范围
     *
     * 在 ranges 描述的子集上扫描（子集非全量时经 subsetIndices 解析为
     * 全量下标），每个体素保留按子集顺序最先出现的点，因此渲染索引
     * 天然升序、逐帧范围可直接按关键帧切分计算。每帧至少保留一个点。
     */
    static void decimateTo(
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& pts,
        const std::vector<CloudRange>& cloudRanges,
        const std::vector<size_t>& subsetIndices,
        bool subsetFullRes,
        float leaf,
        std::vector<size_t>& outIndices,
        std::vector<CloudRange>& outRanges) {
        const float inv = 1.0f / leaf;
        // 体素键 -> 该体素首个点的全量索引
        std::unordered_map<uint64_t, size_t> kept;
        kept.reserve(std::min<size_t>(pts.size(),
                                      std::max<size_t>(pts.size() / 2, 4096)));

        std::vector<size_t> sel;
        sel.reserve(std::min<size_t>(pts.size(), pts.size() / 2 + 1));

        outRanges.clear();
        outRanges.reserve(cloudRanges.size());

        for (const auto& range : cloudRanges) {
            size_t rstart = sel.size();
            for (size_t i = 0; i < range.vertexCount; ++i) {
                // 子集位置 -> 全量下标（全量直通时为恒等映射）
                const size_t j = subsetFullRes
                                     ? range.startVertex + i
                                     : subsetIndices[range.startVertex + i];
                const Eigen::Vector3d& p = pts[j];
                uint64_t k = voxelKey(static_cast<float>(p.x()),
                                      static_cast<float>(p.y()),
                                      static_cast<float>(p.z()), inv);
                if (kept.emplace(k, j).second) {
                    sel.push_back(j);
                }
            }
            // 每帧至少保留一个点，保证选中/播放高亮时该帧仍有可见点
            if (sel.size() == rstart && range.vertexCount > 0) {
                sel.push_back(subsetFullRes
                                  ? range.startVertex
                                  : subsetIndices[range.startVertex]);
            }
            outRanges.push_back({rstart, sel.size() - rstart, range.vertexId});
        }

        outIndices.swap(sel);
    }

    /**
     * @brief 构建分块顶点/颜色数组（从全量点按渲染索引取值）
     *
     * 将渲染点数按 kChunkPoints 拆成若干块，每块一个独立的顶点数组与
     * 颜色数组（Turbo 高程着色 + 透明度）。逻辑索引连续：
     * 第 i 个渲染点位于块 i / kChunkPoints、块内偏移 i % kChunkPoints。
     * 全部在后台线程完成，主线程提交时零遍历。
     */
    static void buildChunkedArrays(
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& pts,
        const std::vector<size_t>& renderIndices,
        bool renderFullRes,
        float colorZMin,
        float colorZMax,
        float opacity,
        std::vector<osg::ref_ptr<osg::Vec3Array>>& outVertices,
        std::vector<osg::ref_ptr<osg::Vec4Array>>& outColors) {
        const size_t count = renderFullRes ? pts.size() : renderIndices.size();
        outVertices.clear();
        outColors.clear();
        if (count == 0) return;

        const size_t chunkCount = (count + kChunkPoints - 1) / kChunkPoints;
        outVertices.reserve(chunkCount);
        outColors.reserve(chunkCount);

        for (size_t c = 0; c < chunkCount; ++c) {
            const size_t begin = c * kChunkPoints;
            const size_t end   = std::min(count, begin + kChunkPoints);
            auto* varr = new osg::Vec3Array;
            auto* carr = new osg::Vec4Array;
            varr->reserve(end - begin);
            carr->reserve(end - begin);
            for (size_t i = begin; i < end; ++i) {
                const Eigen::Vector3d& wp = pts[renderFullRes ? i : renderIndices[i]];
                varr->push_back(osg::Vec3(static_cast<float>(wp.x()),
                                          static_cast<float>(wp.y()),
                                          static_cast<float>(wp.z())));
                osg::Vec4 col = turboColor(static_cast<float>(wp.z()),
                                           colorZMin, colorZMax);
                col.a() = opacity;
                carr->push_back(col);
            }
            outVertices.push_back(varr);
            outColors.push_back(carr);
        }
    }

    /**
     * @brief 构建单个 LOD 级别（在主级别子集上做目标点数的自适应体素降采样）
     *
     * @param pts          全量点数组（ranges/indices 中的下标指向它）
     * @param ranges       主级别逐帧范围（可能是滤波后的存活点范围）
     * @param subsetIndices 主级别渲染索引（subsetFullRes=false 时有效）
     * @param subsetFullRes 主级别是否全量直通
     */
    static LodLevel buildLodLevel(
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& pts,
        const std::vector<CloudRange>& ranges,
        const std::vector<size_t>& subsetIndices,
        bool subsetFullRes,
        const Eigen::Vector3d& bMin,
        const Eigen::Vector3d& bMax,
        size_t targetCount,
        const BuildOptions& options,
        float dataZMin,
        float dataZMax) {
        LodLevel lod;
        const size_t subsetCount = subsetFullRes ? pts.size() : subsetIndices.size();
        if (subsetCount <= targetCount) {
            lod.renderFullRes = true;
            lod.renderRanges = ranges;
        } else {
            float leaf = estimateLeafFromBounds(bMin, bMax, targetCount);
            for (int iter = 0; iter < 6; ++iter) {
                size_t c = countVoxels(pts, ranges, subsetIndices, subsetFullRes, leaf);
                if (c == 0 || c <= targetCount) break;  // 已满足目标
                double ratio = (double)c / (double)targetCount;
                leaf *= (float)(std::pow(ratio, 0.5) * 1.03);
            }
            decimateTo(pts, ranges, subsetIndices, subsetFullRes, leaf,
                       lod.renderIndices, lod.renderRanges);
            lod.renderFullRes = false;
        }
        {
            const float czMin = options.useAutoColorRange ? dataZMin : options.colorZMin;
            const float czMax = options.useAutoColorRange ? dataZMax : options.colorZMax;
            buildChunkedArrays(pts, lod.renderIndices, lod.renderFullRes,
                               czMin, czMax, options.opacity,
                               lod.vertexChunks, lod.colorChunks);
        }
        return lod;
    }

    /**
     * @brief 估计初始体素边长（从包围盒体积与目标点数反推）
     *
     * 初始值刻意偏大（×2），使首轮体素数偏小，避免极端分布下
     * 临时哈希表占用过大的内存峰值。
     */
    static float estimateLeafFromBounds(const Eigen::Vector3d& bMin,
                                        const Eigen::Vector3d& bMax,
                                        size_t targetCount) {
        double extX = bMax.x() - bMin.x();
        double extY = bMax.y() - bMin.y();
        double extZ = bMax.z() - bMin.z();
        double vol = extX * extY * extZ;
        if (!(vol > 0.0)) vol = 1.0;
        double budget = (double)std::max<size_t>(1, targetCount);
        float leaf = static_cast<float>(std::cbrt(vol / budget));
        if (!(leaf > 1e-4f)) leaf = 0.1f;
        return leaf * 2.0f;
    }
};

}  // namespace hdl_graph_slam
