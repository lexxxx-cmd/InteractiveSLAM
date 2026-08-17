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
//      构建渲染索引与逐帧渲染范围（与 KeyframePointCloudVisualizer
//      原实现相同的算法，独立成纯数据版本）；
//   4. 构建渲染顶点数组（osg::Vec3Array，仅普通容器填充，无 GL 调用）。
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
 * @brief 点云构建结果（纯数据，可在线程间移动）
 *
 * 由 PointCloudBuilder::build() 在工作线程生成，
 * 由主线程 KeyframePointCloudVisualizer::commitBuild() 消费。
 */
struct PointCloudBuildResult {
    std::vector<Eigen::Vector3d,
                Eigen::aligned_allocator<Eigen::Vector3d>> allWorldPoints; ///< 全量世界坐标点
    std::vector<CloudRange> cloudRanges;    ///< 每个关键帧的全量范围
    std::vector<size_t>     renderIndices;  ///< 渲染索引 -> 全量索引（降采样时使用）
    std::vector<CloudRange> renderRanges;   ///< 每个关键帧在渲染数组中的范围
    bool renderFullRes = true;              ///< true = 渲染索引与全量索引一致

    osg::ref_ptr<osg::Vec3Array> vertices;  ///< 渲染顶点数组（降采样后）

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
     * @param graph         交互式图数据（读取 keyframes 与位姿）
     * @param maxRenderPoints 渲染点预算（≤0 = 全量）
     * @return 构建结果（全量点 + 渲染索引/范围 + 顶点数组 + 统计）
     *
     * 线程安全：开始时在 optimization_mutex 保护下做毫秒级位姿/点云
     * 快照，之后无锁处理（点云内容加载后不可变）。
     */
    static PointCloudBuildResult build(
        const std::shared_ptr<InteractiveGraph>& graph, int maxRenderPoints) {
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

        // ---- 阶段 3：点预算自适应体素降采样 ----
        const bool decimate = (maxRenderPoints > 0 &&
                               r.allWorldPoints.size() > (size_t)maxRenderPoints);
        if (decimate) {
            float leaf = estimateLeaf(r, maxRenderPoints);
            for (int iter = 0; iter < 6; ++iter) {
                size_t c = countVoxels(r.allWorldPoints, leaf);
                if (c == 0 || c <= (size_t)maxRenderPoints) break;  // 已满足预算
                double ratio = (double)c / (double)maxRenderPoints;
                leaf *= (float)(std::pow(ratio, 0.5) * 1.03);
            }
            buildDecimated(r, leaf);
            r.renderFullRes = false;
        } else {
            r.renderRanges = r.cloudRanges;
            r.renderFullRes = true;
        }

        // ---- 阶段 4：构建渲染顶点数组（普通容器填充，无 GL 调用） ----
        const size_t renderCount =
            r.renderFullRes ? r.allWorldPoints.size() : r.renderIndices.size();
        r.vertices = new osg::Vec3Array;
        r.vertices->reserve(renderCount);
        for (size_t i = 0; i < renderCount; ++i) {
            const Eigen::Vector3d& wp =
                r.allWorldPoints[r.renderFullRes ? i : r.renderIndices[i]];
            r.vertices->push_back(osg::Vec3(
                static_cast<float>(wp.x()),
                static_cast<float>(wp.y()),
                static_cast<float>(wp.z())));
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
    static void buildDecimated(PointCloudBuildResult& r, float leaf) {
        const float inv = 1.0f / leaf;
        // 体素键 -> 该体素首个点的全量索引
        std::unordered_map<uint64_t, size_t> kept;
        kept.reserve(std::min<size_t>(r.allWorldPoints.size(),
                                      std::max<size_t>(r.allWorldPoints.size() / 2, 4096)));

        std::vector<size_t> sel;
        sel.reserve(std::min<size_t>(r.allWorldPoints.size(),
                                     r.allWorldPoints.size() / 2 + 1));

        r.renderRanges.clear();
        r.renderRanges.reserve(r.cloudRanges.size());

        for (const auto& range : r.cloudRanges) {
            size_t rstart = sel.size();
            for (size_t j = range.startVertex;
                 j < range.startVertex + range.vertexCount; ++j) {
                const Eigen::Vector3d& p = r.allWorldPoints[j];
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
            r.renderRanges.push_back({rstart, sel.size() - rstart, range.vertexId});
        }

        r.renderIndices.swap(sel);
    }

    /**
     * @brief 估计初始体素边长（从包围盒体积与预算反推）
     *
     * 初始值刻意偏大（×2），使首轮体素数偏小，避免极端分布下
     * 临时哈希表占用过大的内存峰值。
     */
    static float estimateLeaf(const PointCloudBuildResult& r, int maxRenderPoints) {
        double extX = r.bMax.x() - r.bMin.x();
        double extY = r.bMax.y() - r.bMin.y();
        double extZ = r.bMax.z() - r.bMin.z();
        double vol = extX * extY * extZ;
        if (!(vol > 0.0)) vol = 1.0;
        double budget = (double)std::max(1, maxRenderPoints);
        float leaf = static_cast<float>(std::cbrt(vol / budget));
        if (!(leaf > 1e-4f)) leaf = 0.1f;
        return leaf * 2.0f;
    }
};

}  // namespace hdl_graph_slam
