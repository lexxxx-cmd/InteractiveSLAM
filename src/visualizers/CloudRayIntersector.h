// ============================================================================
// CloudRayIntersector.h
// pose-aware 拾取相交器 —— OSG 官方为自定义几何体预留的扩展点
//
// 为什么必须自定义：新架构把位姿放进了顶点着色器（这是性能的唯一来源），
// CPU 侧的相交器因此读不到世界坐标。原生 LineSegmentIntersector /
// PolytopeIntersector 会通过 Drawable::accept(PrimitiveFunctor&) 拿到**帧局部系**
// 顶点，与**世界系**射线一比就给出世界系错误的交点 —— 而且"看起来合理"，
// 比没有命中更危险。CloudGeometry 已把这两个 accept 覆写为空实现，
// 使原生相交器对点云"不产生交点"而不是"产生错交点"。
//
// OSG 3.6 的分发链（已实测头文件）：
//   IntersectionVisitor::apply(osg::Drawable&)      // osgUtil/IntersectionVisitor:268
//     → Intersector::intersect(iv, drawable)         // osgUtil/LineSegmentIntersector:96
//   osg::Drawable **没有** accept(osgUtil::IntersectionVisitor&) —— 自定义
//   osgUtil::Intersector 就是官方唯一的扩展点。
//
// 本类派生 LineSegmentIntersector（而不是直接派生 Intersector）的理由：
// 可以直接用它的公有 insertIntersection()（:76）与现成的 Intersection 结构
// （含 matrix / indexList / ratio），并复用它的 IntersectionVisitor 分发。
//
// 算法（设计文档 §4.7）：
//   世界射线
//     → 逐帧：帧局部 AABB 经 T_i 变换成世界 AABB（旋转绝对值矩阵技巧）
//     → 射线 vs 世界 AABB 剔除候选帧                      O(帧数)，微秒级
//     → 候选帧按 AABB 入口参数排序，逐个把射线变换到帧局部系
//       （T 是刚体变换 → 长度与垂距都不变，可在局部系里完成全部判定）
//     → 在帧局部点上求「垂距 ≤ 半径 且 沿线最近」的点
//     → 一旦下一个候选帧的入口参数已经大于当前最优 t，立即结束
//     → 只把胜出的点变换回世界，insertIntersection(...)
//
// 关键性质：**完全不看 Drawable**。不读它的顶点、不判断它是否可见、
// 在哪个 LOD 级别、是否已驻留 —— 因此拾取与渲染状态解耦（Phase 4 验收第 1/2/3 条）。
// ============================================================================

#pragma once

#include "visualizers/CloudPickSource.h"

#include <osg/BoundingBox>
#include <osg/CopyOp>
#include <osg/Drawable>
#include <osg/Object>
#include <osg/Vec3d>
#include <osg/Geometry>   // osg::RenderInfo（drawImplementation 形参）
#include <osgUtil/IntersectionVisitor>
#include <osgUtil/LineSegmentIntersector>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <utility>
#include <vector>

namespace hdl_graph_slam {

/**
 * @brief 点云专用拾取相交器：世界射线 + 帧级代理 + 姿态感知
 *
 * 用法（与原生相交器完全一致，这正是"OSG 拾取框架原样使用"的含义）：
 * @code
 *   auto src = std::make_shared<ChunkPickSource>(&chunks, chunkFrames, &poses,
 *                                                CloudPoseTable::kSlotOptimized);
 *   osg::ref_ptr<CloudRayIntersector> picker = new CloudRayIntersector(rayStart, rayEnd, src);
 *   picker->setPickRadius(radiusWorld);
 *   osgUtil::IntersectionVisitor iv(picker.get());
 *   sceneRoot->accept(iv);
 *   if (picker->containsIntersections()) {
 *       const auto& hit = *picker->getIntersections().begin();   // ratio 升序 = 近→远
 *       osg::Vec3d world = hit.getWorldIntersectPoint();
 *       size_t frame = hit.indexList[0];      // 全局帧序
 *       size_t point = hit.indexList[1];      // 帧内点下标
 *   }
 * @endcode
 */
class CloudRayIntersector : public osgUtil::LineSegmentIntersector {
public:
    /**
     * @param start,end 世界坐标射线段（反投影得到，见 SpherePickingHandler.h:333-344）
     * @param source    帧级数据源（须在整个拾取期间存活）
     */
    CloudRayIntersector(const osg::Vec3d& start, const osg::Vec3d& end,
                        std::shared_ptr<const CloudPickSource> source)
        : osgUtil::LineSegmentIntersector(start, end),   // 基类 2 参构造 = MODEL 系
          m_source(std::move(source)) {}

    // ------------------------------------------------------------------
    // 参数
    // ------------------------------------------------------------------

    /**
     * @brief 拾取半径（**世界单位**，米）
     *
     * 对应原生路径里 PolytopeIntersector 的窗口矩形半径，但这里是世界空间的
     * 垂距阈值。因为帧位姿是刚体变换（保距），"世界垂距 ≤ r"与"帧局部垂距 ≤ r"
     * 完全等价，所以在局部系里判定即可，不需要逐点做世界变换。
     *
     * 调用方若想保持"屏幕空间手感一致"，可按相机到命中区域的距离换算：
     *   radiusWorld ≈ radiusNormalized × 2 × tan(fovY/2) × distance
     */
    void setPickRadius(double r) { m_pickRadius = r > 0.0 ? r : 0.0; }
    double pickRadius() const { return m_pickRadius; }

    // ------------------------------------------------------------------
    // 诊断统计（自检与性能日志用）
    // ------------------------------------------------------------------

    /** @brief 一帧都没被 AABB 剔除的候选帧数（上一次扫描） */
    size_t lastCandidateFrames() const { return m_candFrames; }
    /** @brief 上一次扫描实际逐点测过的帧数 */
    size_t lastScannedFrames() const { return m_scannedFrames; }
    /** @brief 上一次扫描实际逐点测过的点数 */
    size_t lastTestedPoints() const { return m_testedPoints; }

    // ------------------------------------------------------------------
    // osgUtil::Intersector 覆写
    // ------------------------------------------------------------------

    /**
     * @brief 克隆（IntersectionVisitor 在遇到 Transform/Projection/Camera 时
     *        会 push_clone()，见 osgUtil/IntersectionVisitor:283）
     *
     * 两处必须小心，都由 OSG 的实现细节决定（已实测头文件）：
     *   1. `getIntersections()` 是 `_parent ? _parent->_intersections : _intersections`
     *      —— 只向上看**一层**。因此多级克隆必须把 `_parent` 直接指向**根**，
     *      否则结果会落进中间克隆，调用方读自己持有的实例时什么都拿不到。
     *   2. 整图扫描只能做一次，所以"是否已扫描"要跨克隆共享。
     */
    osgUtil::Intersector* clone(osgUtil::IntersectionVisitor& iv) override {
        auto* c = new CloudRayIntersector(getStart(), getEnd(), m_source);
        c->m_pickRadius = m_pickRadius;
        c->m_state      = m_state;          // 共享"是否已扫描"
        c->_parent      = rootAncestor();   // 结果集与"已扫描"标记都挂在根上
        (void)iv;
        return c;
    }

    bool enter(const osg::Node&) override { return true; }
    void leave() override {}

    void reset() override {
        osgUtil::LineSegmentIntersector::reset();
        if (m_state) m_state->scanned = false;   // 下一次遍历可以重新扫描
        m_candFrames = m_scannedFrames = m_testedPoints = 0;
    }

    bool containsIntersections() override { return !getIntersections().empty(); }

    /**
     * @brief 唯一的扩展点：被遍历到任意 Drawable 时触发**整图**扫描
     *
     * 刻意不读 drawable（见文件头注释）。因此这个 Drawable 是谁无关紧要 ——
     * 只要场景里还有**任何一个** Drawable 被遍历到，拾取就会发生。
     * 这一点在分页下很重要：若点云 Drawable 全被淘汰，仍由常驻的
     * CloudPickProxy 触发（见下）。
     */
    void intersect(osgUtil::IntersectionVisitor& iv, osg::Drawable* drawable) override {
        auto* root = rootCloud();
        if (!root->m_state) root->m_state = std::make_shared<ScanState>();
        if (root->m_state->scanned) return;
        root->m_state->scanned = true;
        root->scan(drawable);
        (void)iv;
    }

private:
    /** @brief 跨克隆共享的"是否已扫描"状态 */
    struct ScanState {
        bool scanned = false;
    };

    /** @brief 链上最顶层的 LineSegmentIntersector（结果集挂在它身上） */
    osgUtil::LineSegmentIntersector* rootAncestor() {
        osgUtil::LineSegmentIntersector* top = this;
        for (osgUtil::LineSegmentIntersector* p = _parent; p != nullptr;) {
            top = p;
            auto* cp = dynamic_cast<CloudRayIntersector*>(p);
            p = cp ? cp->_parent : nullptr;   // 非本类则无法继续上溯
        }
        return top;
    }

    /** @brief 链上最顶层的本类实例（"已扫描"标记挂在它身上） */
    CloudRayIntersector* rootCloud() {
        CloudRayIntersector* top = this;
        for (osgUtil::LineSegmentIntersector* p = _parent; p != nullptr;) {
            auto* cp = dynamic_cast<CloudRayIntersector*>(p);
            if (!cp) break;
            top = cp;
            p = cp->_parent;
        }
        return top;
    }

    /**
     * @brief 帧局部 AABB 经位姿变换成世界 AABB（旋转绝对值矩阵技巧），并按拾取半径膨胀
     *
     * **半径必须参与粗筛**：判定是"点到射线的垂距 ≤ 半径"，而满足该条件的点可以
     * 落在帧 AABB 之外（射线从盒子外侧擦过、最近点仍在半径内）。只按原始 AABB 剔除
     * 会把这些合法命中丢掉。膨胀量 = 拾取半径 + 一点 float 舍入余量。
     */
    bool frameWorldAABB(const Eigen::Isometry3d& T, const float lo[3], const float hi[3],
                        float wlo[3], float whi[3]) const {
        const Eigen::Vector3d c(0.5 * (static_cast<double>(lo[0]) + hi[0]),
                                0.5 * (static_cast<double>(lo[1]) + hi[1]),
                                0.5 * (static_cast<double>(lo[2]) + hi[2]));
        const Eigen::Vector3d e(0.5 * (static_cast<double>(hi[0]) - lo[0]),
                                0.5 * (static_cast<double>(hi[1]) - lo[1]),
                                0.5 * (static_cast<double>(hi[2]) - lo[2]));
        const Eigen::Vector3d cw = T.linear() * c + T.translation();
        const Eigen::Vector3d ew = T.linear().cwiseAbs() * e;
        const double pad = kAabbPad + m_pickRadius;
        for (int k = 0; k < 3; ++k) {
            wlo[k] = static_cast<float>(cw[k] - ew[k] - pad);
            whi[k] = static_cast<float>(cw[k] + ew[k] + pad);
        }
        return true;
    }

    /** @brief 射线 vs AABB 的 slab 检验；命中时输出入口参数（clamp 到 >= 0） */
    static bool rayHitsAABB(const osg::Vec3d& s, const osg::Vec3d& d, double len,
                            const float lo[3], const float hi[3], double& tEnter) {
        double t0 = 0.0, t1 = len;
        for (int k = 0; k < 3; ++k) {
            const double sk = s[k];
            const double dk = d[k];
            const double mn = static_cast<double>(lo[k]);
            const double mx = static_cast<double>(hi[k]);
            if (std::fabs(dk) < 1e-15) {
                if (sk < mn || sk > mx) return false;    // 与该轴平行且在板外
            } else {
                double ta = (mn - sk) / dk;
                double tb = (mx - sk) / dk;
                if (ta > tb) std::swap(ta, tb);
                if (ta > t0) t0 = ta;
                if (tb < t1) t1 = tb;
                if (t0 > t1) return false;
            }
        }
        tEnter = t0;
        return true;
    }

    /** @brief 整图扫描：帧级 AABB 剔除 → 候选帧排序 → 局部系精确求点 */
    void scan(osg::Drawable* drawable) {
        if (!m_source) return;

        const osg::Vec3d s0 = getStart();
        const osg::Vec3d e0 = getEnd();
        const osg::Vec3d seg = e0 - s0;
        const double segLen = seg.length();
        if (!(segLen > 1e-12)) return;
        const osg::Vec3d dir = seg / segLen;

        struct Cand {
            size_t frame;
            double tEnter;
        };
        std::vector<Cand> cands;
        const size_t nf = m_source->frameCount();
        cands.reserve(nf < 1024 ? nf : 1024);

        for (size_t i = 0; i < nf; ++i) {
            size_t count = 0;
            float lo[3], hi[3];
            if (!m_source->framePoints(i, count, lo, hi) || count == 0) continue;
            float wlo[3], whi[3];
            frameWorldAABB(m_source->framePose(i), lo, hi, wlo, whi);
            double tEnter = 0.0;
            if (!rayHitsAABB(s0, dir, segLen, wlo, whi, tEnter)) continue;
            cands.push_back({i, tEnter});
        }
        m_candFrames = cands.size();
        if (cands.empty()) return;

        // 按 AABB 入口参数升序：一旦确定最优命中，后面帧的入口更远就不可能更近
        std::sort(cands.begin(), cands.end(),
                  [](const Cand& a, const Cand& b) { return a.tEnter < b.tEnter; });

        bool        found = false;
        double      bestT = 0.0;        // 世界射线上的参数，单位米（自 s0 起）
        double      bestPerp2 = 0.0;
        size_t      bestFrame = 0;
        size_t      bestPoint = 0;
        osg::Vec3d  bestWorld;

        for (const Cand& cd : cands) {
            if (found && cd.tEnter > bestT) break;   // 后续候选不可能更近

            size_t count = 0;
            float lo[3], hi[3];
            const float* pts = m_source->framePoints(cd.frame, count, lo, hi);
            if (!pts || count == 0) continue;

            const Eigen::Isometry3d T = m_source->framePose(cd.frame);
            // 射线变换到帧局部系：T 刚体 → 长度与垂距不变，于是判定全在局部系做，
            // 只有胜出的那一个点需要变换回世界（每帧一次 4×4 求逆 + 两次乘）
            const Eigen::Isometry3d Tinv = T.inverse();
            const Eigen::Vector3d sl = Tinv * Eigen::Vector3d(s0.x(), s0.y(), s0.z());
            const Eigen::Vector3d el = Tinv * Eigen::Vector3d(e0.x(), e0.y(), e0.z());
            const Eigen::Vector3d dl = el - sl;
            const double dl2 = dl.squaredNorm();
            if (!(dl2 > 1e-24)) continue;

            ++m_scannedFrames;
            m_testedPoints += count;

            const double r2 = m_pickRadius * m_pickRadius;
            for (size_t k = 0; k < count; ++k) {
                const double px = pts[3 * k];
                const double py = pts[3 * k + 1];
                const double pz = pts[3 * k + 2];
                const double vx = px - sl.x();
                const double vy = py - sl.y();
                const double vz = pz - sl.z();

                // Clamp the projection into [0,1] instead of rejecting out-of-range
                // points. That turns the test below into "distance to the SEGMENT",
                // which is the physically meaningful quantity for picking.
                //
                // Rejecting on t < 0 looks equivalent but is not: the pose table
                // stores float, so this class reconstructs T from rounded values and
                // T^-1 * (T * p) differs from p by ~1e-4 m at large map scales. A ray
                // that starts exactly on a surface point therefore projects to
                // t = 0 +/- 1e-5 and is silently dropped whenever the rounding lands
                // negative. Measured on a real 624-frame map: 4 misses out of 200
                // such samples (cloud_pick_selftest --map). Clamping removes the
                // sensitivity entirely and needs no magic slack constant.
                double t = (vx * dl.x() + vy * dl.y() + vz * dl.z()) / dl2;
                if (t < 0.0) t = 0.0;
                else if (t > 1.0) t = 1.0;

                const double cx = vx - t * dl.x();
                const double cy = vy - t * dl.y();
                const double cz = vz - t * dl.z();
                const double perp2 = cx * cx + cy * cy + cz * cz;
                if (perp2 > r2) continue;                // beyond the pick radius

                const double tWorld = t * segLen;        // local param x segment length = metres
                const bool better =
                    !found ||
                    (tWorld < bestT - kTieEps) ||
                    (std::fabs(tWorld - bestT) <= kTieEps && perp2 < bestPerp2);
                if (!better) continue;

                found      = true;
                bestT      = tWorld;
                bestPerp2  = perp2;
                bestFrame  = cd.frame;
                bestPoint  = k;
                const Eigen::Vector3d w = T * Eigen::Vector3d(px, py, pz);
                bestWorld  = osg::Vec3d(w.x(), w.y(), w.z());
            }
        }

        if (!found) return;

        osgUtil::LineSegmentIntersector::Intersection isect;
        // ratio 用于 Intersections（std::multiset，按 ratio 升序 = 近→远）
        isect.ratio = bestT / segLen;
        isect.localIntersectionPoint = bestWorld;
        isect.localIntersectionNormal = osg::Vec3(0.0f, 0.0f, 0.0f);
        // 这里给的已经是世界坐标，故不挂参考矩阵（getWorldIntersectPoint 将原样返回）
        isect.matrix = nullptr;
        isect.drawable = drawable;
        isect.primitiveIndex = static_cast<unsigned int>(bestPoint);
        isect.indexList.clear();
        isect.indexList.push_back(static_cast<unsigned int>(bestFrame));  // 全局帧序
        isect.indexList.push_back(static_cast<unsigned int>(bestPoint));  // 帧内点下标
        insertIntersection(isect);
    }

    /** @brief 命中点与射线求垂距时判定"同一参数"的容差（米） */
    static constexpr double kTieEps = 1e-9;
    /** @brief 世界 AABB 的额外膨胀量（米），抵消 float 舍入 */
    static constexpr double kAabbPad = 1e-3;

    std::shared_ptr<const CloudPickSource> m_source;
    std::shared_ptr<ScanState> m_state = std::make_shared<ScanState>();
    double m_pickRadius = 0.05;

    size_t m_candFrames    = 0;
    size_t m_scannedFrames = 0;
    size_t m_testedPoints  = 0;
};

/**
 * @brief 常驻拾取代理：不渲染任何东西，但**永远在场景图里**的 Drawable
 *
 * 存在的理由：CloudRayIntersector 的整图扫描由"遍历到某个 Drawable"触发，而它
 * 刻意不读 Drawable 的数据。若场景里的点云 Drawable 因分页淘汰/渐进上传隐藏而
 * 一个都不在（或都被移出），拾取就会静默失效 —— 正是设计文档 §5.3 要避免的
 * "未加载节点让拾取静默失败"。挂上这个代理后，"拾取与驻留无关"就从
 * "恰好因为云还在"变成**结构性保证**（Phase 4 验收第 3 条）。
 *
 * 它的包围盒 = 整图 AABB，因此也能被 OSG 正常纳入遍历/剔除计算。
 */
class CloudPickProxy : public osg::Drawable {
public:
    CloudPickProxy() { setUseDisplayList(false); setUseVertexBufferObjects(false); }

    CloudPickProxy(const CloudPickProxy& rhs,
                   const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY)
        : osg::Drawable(rhs, copyop), m_bounds(rhs.m_bounds) {}

    META_Object(hdl_graph_slam, CloudPickProxy)

    /** @brief 设置代理的包围盒（= 整图世界 AABB；位姿变化后由外层刷新） */
    void setWorldBounds(const osg::BoundingBox& bb) {
        m_bounds = bb;
        dirtyBound();
    }
    const osg::BoundingBox& worldBounds() const { return m_bounds; }

    osg::BoundingBox computeBoundingBox() const override { return m_bounds; }

    /** @brief 不产生任何绘制（拾取代理不参与渲染） */
    void drawImplementation(osg::RenderInfo&) const override {}

    // 与 CloudGeometry 一致：不把图元交给原生相交器（这里本来也没有图元）
    void accept(osg::PrimitiveFunctor&) const override {}
    void accept(osg::PrimitiveIndexFunctor&) const override {}
    bool supports(const osg::PrimitiveFunctor&) const override { return false; }
    bool supports(const osg::PrimitiveIndexFunctor&) const override { return false; }

private:
    osg::BoundingBox m_bounds;
};

}  // namespace hdl_graph_slam
