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
//   3. 点预算降采样：全量点数超过预算时，自适应迭代选取体素边长，
//      构建渲染索引与逐帧渲染范围；
//   4. 多级 LOD（仅全量模式）：当 maxRenderPoints ≤ 0 且 lodEnabled 时，
//      在全量点基础上按递增体素边长生成 level1~5（目标点数 N/2、N/4、
//      N/8、N/16、N/32，级间 2×，每级不低于 5 万点），每级都映射回
//      同一个全量点数组，供相机距离切换或手动选择使用；
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
 * colors 数组由 KeyframePointCloudVisualizer 生成（本类不产生颜色）。
 */
struct LodLevel {
    osg::ref_ptr<osg::Vec3Array> vertices;      ///< 该级渲染顶点数组
    osg::ref_ptr<osg::Vec4Array> colors;        ///< 该级渲染颜色数组（由可视化器生成）
    std::vector<size_t>     renderIndices;      ///< 渲染索引 -> 全量索引（level0 全量时为空）
    std::vector<CloudRange> renderRanges;       ///< 每个关键帧在该级渲染数组中的范围
    bool renderFullRes = true;                  ///< true = 渲染索引与全量索引一致
};

/**
 * @brief 点云构建选项
 */
struct BuildOptions {
    int  maxRenderPoints = -1;  ///< 渲染点预算（≤0 = 全量）
    bool lodEnabled = false;    ///< 是否生成多级 LOD（仅在 maxRenderPoints ≤ 0 时生效）
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

    osg::ref_ptr<osg::Vec3Array> vertices;  ///< 主级别渲染顶点数组

    std::vector<LodLevel> lodLevels;        ///< LOD 级别（不含主级别，仅全量模式生成）

    float zMin =  std::numeric_limits<float>::max();  ///< 数据 Z 最小值
    float zMax = -std::numeric_limits<float>::max();  ///< 数据 Z 最大值
    Eigen::Vector3d bMin;  ///< 世界包围盒最小值
    Eigen::Vector3d bMax;  ///< 世界包围盒最大值
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
     * @brief 构建点云渲染数据（可在后台线程调用）
     * @param graph   交互式图数据（读取 keyframes 与位姿）
     * @param options 构建选项（点预算 + LOD 开关）
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
            Eigen::Isometry3d pose;
            pcl::PointCloud<PointT>::ConstPtr cloud;
        };
        std::vector<FrameSnapshot, Eigen::aligned_allocator<FrameSnapshot>> frames;
        {
            std::lock_guard<std::mutex> lock(graph->optimization_mutex);
            for (auto& [id, kf] : graph->keyframes) {
                auto* v = dynamic_cast<g2o::VertexSE3*>(kf->node);
                if (!v || !kf->cloud || kf->cloud->empty()) continue;
                frames.push_back({id, v->estimate(), kf->cloud});
            }
        }

        // ---- 阶段 2：无锁变换到世界坐标系 ----
        for (const auto& frame : frames) {
            size_t start = r.allWorldPoints.size();
            for (const auto& pt : frame.cloud->points) {
                Eigen::Vector3d wp = frame.pose * Eigen::Vector3d(pt.x, pt.y, pt.z);
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
            }
            r.cloudRanges.push_back({start, r.allWorldPoints.size() - start, frame.id});
        }

        if (r.allWorldPoints.empty()) {
            r.vertices = new osg::Vec3Array;  // 空结果也提供空数组，避免 commit 时解引用空指针
            return r;
        }

        // 防止 Z 值范围过小导致的颜色映射异常
        if (r.zMax - r.zMin < 0.001f) {
            r.zMin -= 0.5f;
            r.zMax += 0.5f;
        }

        // ---- 阶段 3：主级别（预算降采样或全量） ----
        const size_t n = r.allWorldPoints.size();
        const bool decimate = (options.maxRenderPoints > 0 &&
                               n > (size_t)options.maxRenderPoints);
        if (decimate) {
            float leaf = estimateLeafFromBounds(r.bMin, r.bMax, options.maxRenderPoints);
            for (int iter = 0; iter < 6; ++iter) {
                size_t c = countVoxels(r.allWorldPoints, leaf);
                if (c == 0 || c <= (size_t)options.maxRenderPoints) break;  // 已满足预算
                double ratio = (double)c / (double)options.maxRenderPoints;
                leaf *= (float)(std::pow(ratio, 0.5) * 1.03);
            }
            decimateTo(r.allWorldPoints, r.cloudRanges, leaf,
                       r.renderIndices, r.renderRanges);
            r.renderFullRes = false;
        } else {
            r.renderRanges = r.cloudRanges;
            r.renderFullRes = true;
        }

        // ---- 阶段 4：构建主级别顶点数组 ----
        buildVertices(r.allWorldPoints, r.renderIndices, r.renderFullRes, r.vertices);

        // ---- 阶段 5：多级 LOD（仅全量模式） ----
        // 目标点数 N/2、N/4、…、N/32（级间 2×，最多 5 级；每级不低于 minLodPoints）
        // 级间 2× 使相机距离切换过渡平滑，避免 4× 时"差一级就跳回全量"的突兀感
        if (options.lodEnabled && options.maxRenderPoints <= 0) {
            const size_t minLodPoints = 50000;
            const int    maxLodLevels = 5;
            size_t target = n / 2;
            while (target >= minLodPoints && (int)r.lodLevels.size() < maxLodLevels) {
                r.lodLevels.push_back(
                    buildLodLevel(r.allWorldPoints, r.cloudRanges, r.bMin, r.bMax, target));
                target /= 2;
            }
        }

        return r;
    }

private:
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
     * @brief 统计给定体素边长下点集的体素数（≈渲染点数）
     */
    static size_t countVoxels(
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& pts,
        float leaf) {
        const float inv = 1.0f / leaf;
        std::unordered_set<uint64_t> seen;
        seen.reserve(std::min<size_t>(pts.size(), 4u * 1024u * 1024u));
        for (const auto& p : pts) {
            seen.insert(voxelKey(static_cast<float>(p.x()),
                                 static_cast<float>(p.y()),
                                 static_cast<float>(p.z()), inv));
        }
        return seen.size();
    }

    /**
     * @brief 按给定体素边长构建降采样后的渲染索引与逐帧渲染范围
     *
     * 每个体素保留按原始顺序最先出现的点，因此渲染索引天然升序、
     * 逐帧范围可直接按关键帧切分计算。每帧至少保留一个点。
     */
    static void decimateTo(
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& pts,
        const std::vector<CloudRange>& cloudRanges,
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
            for (size_t j = range.startVertex;
                 j < range.startVertex + range.vertexCount; ++j) {
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
                sel.push_back(range.startVertex);
            }
            outRanges.push_back({rstart, sel.size() - rstart, range.vertexId});
        }

        outIndices.swap(sel);
    }

    /**
     * @brief 构建顶点数组（从全量点按渲染索引取值）
     */
    static void buildVertices(
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& pts,
        const std::vector<size_t>& renderIndices,
        bool renderFullRes,
        osg::ref_ptr<osg::Vec3Array>& outVertices) {
        const size_t count = renderFullRes ? pts.size() : renderIndices.size();
        outVertices = new osg::Vec3Array;
        outVertices->reserve(count);
        for (size_t i = 0; i < count; ++i) {
            const Eigen::Vector3d& wp = pts[renderFullRes ? i : renderIndices[i]];
            outVertices->push_back(osg::Vec3(
                static_cast<float>(wp.x()),
                static_cast<float>(wp.y()),
                static_cast<float>(wp.z())));
        }
    }

    /**
     * @brief 构建单个 LOD 级别（目标点数 targetCount 的自适应体素降采样）
     */
    static LodLevel buildLodLevel(
        const std::vector<Eigen::Vector3d,
                          Eigen::aligned_allocator<Eigen::Vector3d>>& pts,
        const std::vector<CloudRange>& cloudRanges,
        const Eigen::Vector3d& bMin,
        const Eigen::Vector3d& bMax,
        size_t targetCount) {
        LodLevel lod;
        if (pts.size() <= targetCount) {
            lod.renderFullRes = true;
            lod.renderRanges = cloudRanges;
        } else {
            float leaf = estimateLeafFromBounds(bMin, bMax, targetCount);
            for (int iter = 0; iter < 6; ++iter) {
                size_t c = countVoxels(pts, leaf);
                if (c == 0 || c <= targetCount) break;  // 已满足目标
                double ratio = (double)c / (double)targetCount;
                leaf *= (float)(std::pow(ratio, 0.5) * 1.03);
            }
            decimateTo(pts, cloudRanges, leaf, lod.renderIndices, lod.renderRanges);
            lod.renderFullRes = false;
        }
        buildVertices(pts, lod.renderIndices, lod.renderFullRes, lod.vertices);
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
