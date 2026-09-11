// ============================================================================
// cloud_pick_selftest.cpp
// Phase 4 拾取的无头自检（不需要 GPU / 窗口 / 交互）
//
// 目的：拾取数学是整套架构里最容易"看起来对、实际错"的一环（射线是世界系、
// 顶点是帧局部系，一旦把 T 用成 T⁻¹ 或漏了逆变换，结果依然是个合理的点，
// 只是位置错了）。所以在接进 GUI 之前，先用可判定的数值把这条链路钉死。
//
// 两段：
//   A. 合成数据 —— 精确数值验证（默认执行，不依赖任何地图文件）
//      逐条断言：命中的帧/点下标、世界坐标、ratio、以及**"命中点到射线的垂距
//      ≤ 半径"这条核心不变量**（用错变换时正是它会被破坏）。
//   B. 真实地图往返（--map 时执行）
//      随机取 (帧, 点)，用它的世界坐标构造射线，断言往返一致。
//
// 用法：
//   cloud_pick_selftest [--map <地图目录>] [--sample N]
//
// 退出码: 0 = 全部通过；1 = 有失败项；2 = 参数/加载错误
// ============================================================================

#include "visualizers/CloudPickSource.h"
#include "visualizers/CloudRayIntersector.h"
#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "data/hdl_graph_slam/keyframe.hpp"
#include "data/hdl_graph_slam/progress_interface.hpp"

#include <osg/Geode>
#include <osg/Group>
#include <osg/Vec3d>
#include <osgUtil/IntersectionVisitor>

#include <Eigen/Geometry>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <random>
#include <string>
#include <vector>

using namespace hdl_graph_slam;

namespace {

void usage() {
    std::printf(
        "cloud_pick_selftest —— Phase 4 拾取无头自检\n"
        "\n"
        "用法:\n"
        "  cloud_pick_selftest [--map <地图目录>] [--sample N]\n"
        "      --map DIR   额外跑真实地图往返验证（默认只跑合成数据段）\n"
        "      --sample N  真实地图段的抽样帧数（默认 200）\n"
        "\n"
        "退出码: 0 = 全部通过；1 = 有失败项；2 = 参数/加载错误\n");
}

struct Checker {
    int checks = 0, failures = 0;
    void check(bool cond, const char* what) {
        ++checks;
        if (!cond) {
            ++failures;
            std::printf("  [FAIL] %s\n", what);
            std::fflush(stdout);
        }
    }
    void section(const char* name) {
        std::printf("-- %s\n", name);
        std::fflush(stdout);
    }
    int summary() const {
        std::printf("\n自检项 %d，失败 %d\n", checks, failures);
        return failures == 0 ? 0 : 1;
    }
};

struct SilentProgress : ProgressInterface {
    int max = 0, cur = 0;
    void set_maximum(int m) override { max = m; }
    void increment() override {
        ++cur;
        if (max > 0 && (cur % 512 == 0 || cur == max)) {
            std::printf("\r[load] %3d%%", cur * 100 / max);
            std::fflush(stdout);
        }
    }
};

std::string argValue(int argc, char** argv, const char* name, const std::string& def) {
    for (int i = 0; i < argc; ++i) {
        if (std::strcmp(argv[i], name) == 0 && i + 1 < argc) return argv[i + 1];
    }
    return def;
}

// ---------------------------------------------------------------------------
// 射线工具
// ---------------------------------------------------------------------------

/** @brief 点到线段的垂距 */
double perpToSegment(const osg::Vec3d& p, const osg::Vec3d& s, const osg::Vec3d& e) {
    const osg::Vec3d d = e - s;
    const double L2 = d.length2();
    if (L2 < 1e-24) return (p - s).length();
    double t = ((p - s) * d) / L2;   // osg::Vec3d 的 operator* 是点积
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    return (p - (s + d * t)).length();
}

/** @brief 命中结果（从相交器的第一条结果提取；multiset 按 ratio 升序 = 近→远） */
struct Hit {
    bool   ok = false;
    osg::Vec3d world{0, 0, 0};
    long long  frame = -1;
    long long  point = -1;
    double     ratio = -1.0;
};

/**
 * @brief 跑一次拾取：场景里**只放常驻拾取代理**，不放任何点云 Drawable
 *
 * 这本身就是 Phase 4 验收第 3 条的结构性验证：拾取不依赖任何点云 Drawable
 * 是否存在。若哪天有人把拾取改成读 Drawable 顶点，这一整段会立刻全挂。
 */
Hit runPick(CloudRayIntersector* picker) {
    osg::ref_ptr<osg::Geode> geode = new osg::Geode;
    geode->addDrawable(new CloudPickProxy);
    osgUtil::IntersectionVisitor iv(picker);
    geode->accept(iv);

    Hit h;
    const auto& xs = picker->getIntersections();
    if (xs.empty()) return h;
    const auto& it = *xs.begin();
    h.ok    = true;
    h.world = it.getWorldIntersectPoint();
    h.ratio = it.ratio;
    if (it.indexList.size() >= 1) h.frame = static_cast<long long>(it.indexList[0]);
    if (it.indexList.size() >= 2) h.point = static_cast<long long>(it.indexList[1]);
    return h;
}

/** @brief 便捷封装：构造相交器 + 扫描 + 取首条命中 */
Hit pickWith(const std::shared_ptr<const CloudPickSource>& src,
             const osg::Vec3d& s, const osg::Vec3d& e, double radius,
             size_t* candFrames = nullptr, size_t* scannedFrames = nullptr,
             size_t* testedPoints = nullptr) {
    osg::ref_ptr<CloudRayIntersector> picker = new CloudRayIntersector(s, e, src);
    picker->setPickRadius(radius);
    const Hit h = runPick(picker.get());
    if (candFrames)    *candFrames    = picker->lastCandidateFrames();
    if (scannedFrames) *scannedFrames = picker->lastScannedFrames();
    if (testedPoints)  *testedPoints  = picker->lastTestedPoints();
    return h;
}

// ---------------------------------------------------------------------------
// 合成帧数据源（段 A 用）
// ---------------------------------------------------------------------------

class SyntheticSource : public CloudPickSource {
public:
    void addFrame(const Eigen::Isometry3d& T,
                  const std::vector<std::array<float, 3>>& pts) {
        Frame f;
        f.pose = T;
        f.lo[0] = f.lo[1] = f.lo[2] = 1e30f;
        f.hi[0] = f.hi[1] = f.hi[2] = -1e30f;
        for (const auto& p : pts) {
            f.pts.insert(f.pts.end(), {p[0], p[1], p[2]});
            for (int k = 0; k < 3; ++k) {
                f.lo[k] = std::min(f.lo[k], p[k]);
                f.hi[k] = std::max(f.hi[k], p[k]);
            }
        }
        frames.push_back(std::move(f));
    }

    size_t frameCount() const override { return frames.size(); }
    uint32_t poseSlot() const override { return 0; }

    Eigen::Isometry3d framePose(size_t i) const override { return frames[i].pose; }

    const float* framePoints(size_t i, size_t& count, float lo[3], float hi[3]) const override {
        const Frame& f = frames[i];
        for (int k = 0; k < 3; ++k) { lo[k] = f.lo[k]; hi[k] = f.hi[k]; }
        count = f.pts.size() / 3;
        return f.pts.empty() ? nullptr : f.pts.data();
    }

    /** @brief 第 i 帧第 k 个点的世界坐标（期望值） */
    Eigen::Vector3d worldPoint(size_t i, size_t k) const {
        const Eigen::Vector3d p(frames[i].pts[3 * k],
                                frames[i].pts[3 * k + 1],
                                frames[i].pts[3 * k + 2]);
        return frames[i].pose * p;
    }

private:
    struct Frame {
        Eigen::Isometry3d pose;
        std::vector<float> pts;   // float3
        float lo[3], hi[3];
    };
    std::vector<Frame> frames;
};

bool near3(const osg::Vec3d& a, const Eigen::Vector3d& b, double eps) {
    return std::fabs(a.x() - b.x()) < eps &&
           std::fabs(a.y() - b.y()) < eps &&
           std::fabs(a.z() - b.z()) < eps;
}

// ---------------------------------------------------------------------------
// 段 A：合成数据的精确数值验证
// ---------------------------------------------------------------------------

void sectionSynthetic(Checker& ck) {
    ck.section("A. 合成数据（精确数值验证：帧/点下标、世界坐标、ratio、垂距不变量）");

    SyntheticSource src;
    // 帧 0：单位位姿；帧 1：绕 Z 转 +90° 后平移 (100,0,0)
    // 两帧的点集相同，但世界坐标完全不同 —— 用错位姿或漏掉逆变换都会立刻露馅
    const std::vector<std::array<float, 3>> pts = {
        {0.0f, 0.0f, 0.0f}, {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 1.0f}};
    src.addFrame(Eigen::Isometry3d::Identity(), pts);

    Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
    // 用数字字面量而不是 M_PI：MSVC 的 <cmath> 默认不定义 M_PI（需要 _USE_MATH_DEFINES），
    // 本项目其余代码（如 FirstPersonManipulator）也一律用字面量，这里保持一致。
    T1.linear() = Eigen::AngleAxisd(1.5707963267948966, Eigen::Vector3d::UnitZ())
                      .toRotationMatrix();
    T1.translation() = Eigen::Vector3d(100.0, 0.0, 0.0);
    src.addFrame(T1, pts);

    // 两帧的世界点位（用于断言）：
    //   帧0: k0=(0,0,0) k1=(1,0,0) k2=(0,1,0) k3=(0,0,1)
    //   帧1: k0=(100,0,0) k1=(100,1,0) k2=(99,0,0) k3=(100,0,1)
    const auto S = std::make_shared<SyntheticSource>(src);

    // —— 用例 1：射线 y=5, z=0 沿 +X ——
    // 帧0 的 k0/k2/k3 都在 t=10 处；k2 的垂距最小（4，其余 5 / 5.099）
    // 帧1 的点在 t≈110，更远 → 应当被"沿线最近"淘汰
    {
        size_t cand = 0, scanned = 0, tested = 0;
        const Hit h = pickWith(S, osg::Vec3d(-10, 5, 0), osg::Vec3d(110, 5, 0),
                               /*radius=*/6.0, &cand, &scanned, &tested);
        std::printf("  用例1 命中=%d 帧=%lld 点=%lld ratio=%.5f 候选帧=%zu 实扫帧=%zu 实测点=%zu\n",
                    h.ok ? 1 : 0, h.frame, h.point, h.ratio, cand, scanned, tested);
        ck.check(h.ok, "用例1：应当命中");
        ck.check(h.frame == 0 && h.point == 2,
                 "用例1：命中帧0的第2个点（t 相同取垂距最小者）");
        ck.check(near3(h.world, src.worldPoint(0, 2), 1e-6),
                 "用例1：世界坐标 == T0·p2 == (0,1,0)");
        ck.check(std::fabs(h.ratio - 10.0 / 120.0) < 1e-9,
                 "用例1：ratio == 10/120（世界射线参数）");
        ck.check(perpToSegment(h.world, osg::Vec3d(-10, 5, 0), osg::Vec3d(110, 5, 0)) <= 6.0 + 1e-9,
                 "用例1：命中点到射线垂距 <= 半径（核心不变量）");
        ck.check(cand == 2, "用例1：两帧的世界 AABB 都被判为候选");
        ck.check(scanned == 1, "用例1：按 AABB 入口排序 + 提前结束，只实扫了最近的 1 帧");
        ck.check(tested == 4, "用例1：只逐点测了 4 个点（一帧）");
    }

    // —— 用例 2：收紧半径到 4.5 —— 仍应命中帧0 的 k2（垂距 4）——
    {
        const Hit h = pickWith(S, osg::Vec3d(-10, 5, 0), osg::Vec3d(110, 5, 0), 4.5);
        ck.check(h.ok && h.frame == 0 && h.point == 2,
                 "用例2：半径 4.5 时仍命中帧0第2点（垂距 4 <= 4.5）");
    }

    // —— 用例 3：半径 3.5 —— 所有点的垂距都 > 3.5，应当无命中 ——
    {
        const Hit h = pickWith(S, osg::Vec3d(-10, 5, 0), osg::Vec3d(110, 5, 0), 3.5);
        ck.check(!h.ok, "用例3：半径 3.5 时应当无命中（最小垂距为 4）");
    }

    // —— 用例 4：射线反向 —— 最近命中应变成帧1（验证方向与 T⁻¹ 的正确性）——
    {
        const Hit h = pickWith(S, osg::Vec3d(110, 5, 0), osg::Vec3d(-10, 5, 0), 6.0);
        std::printf("  用例4 命中=%d 帧=%lld 点=%lld ratio=%.5f\n",
                    h.ok ? 1 : 0, h.frame, h.point, h.ratio);
        ck.check(h.ok, "用例4：反向射线应当命中");
        ck.check(h.frame == 1 && h.point == 1,
                 "用例4：反向时最近的是帧1第1点（t 相同取垂距最小者）");
        ck.check(near3(h.world, src.worldPoint(1, 1), 1e-6),
                 "用例4：世界坐标 == T1·p1 == (100,1,0)");
        ck.check(perpToSegment(h.world, osg::Vec3d(110, 5, 0), osg::Vec3d(-10, 5, 0)) <= 6.0 + 1e-9,
                 "用例4：命中点到反向射线垂距 <= 半径");
    }

    // —— 用例 5：垂距刚好在半径内外 —— （射线贴近 k2 的世界点 (0,1,0)，偏移 0.03）——
    {
        const Hit in = pickWith(S, osg::Vec3d(-10, 1.03, 0), osg::Vec3d(10, 1.03, 0), 0.05);
        ck.check(in.ok && in.frame == 0 && in.point == 2,
                 "用例5a：垂距 0.03 <= 半径 0.05 → 命中 k2");

        const Hit out = pickWith(S, osg::Vec3d(-10, 1.03, 0), osg::Vec3d(10, 1.03, 0), 0.02);
        ck.check(!out.ok, "用例5b：垂距 0.03 > 半径 0.02 → 无命中");
    }

    // —— 用例 5c：穿透帧 AABB 但离所有点都超出半径 ——
    // 这条**单独**验证"逐点半径判定"（前两个用例其实是 AABB 粗筛就挡住了）：
    // 射线穿过单位立方体内部（y=z=0.5），离四个角点最近也有 0.707
    {
        const Hit no = pickWith(S, osg::Vec3d(-10, 0.5, 0.5), osg::Vec3d(10, 0.5, 0.5), 0.5);
        ck.check(!no.ok, "用例5c-1：射线穿过 AABB 但离所有点 0.707 > 半径 0.5 → 逐点判定拒绝");

        const Hit yes = pickWith(S, osg::Vec3d(-10, 0.5, 0.5), osg::Vec3d(10, 0.5, 0.5), 0.8);
        ck.check(yes.ok && yes.frame == 0,
                 "用例5c-2：半径放到 0.8 后命中（0.707 <= 0.8）");
        ck.check(perpToSegment(yes.world, osg::Vec3d(-10, 0.5, 0.5), osg::Vec3d(10, 0.5, 0.5))
                     <= 0.8 + 1e-9,
                 "用例5c-2：命中点垂距 <= 半径");
    }

    // —— 用例 6：射线完全在场景之外 ——
    {
        const Hit h = pickWith(S, osg::Vec3d(-10, 500, 500), osg::Vec3d(110, 500, 500), 1.0);
        ck.check(!h.ok, "用例6：远离所有帧的射线无命中");
    }

    // —— 用例 7：重置后可重复使用同一实例（reset 会清掉"已扫描"标记）——
    {
        auto picker = osg::ref_ptr<CloudRayIntersector>(
            new CloudRayIntersector(osg::Vec3d(-10, 5, 0), osg::Vec3d(110, 5, 0), S));
        picker->setPickRadius(6.0);
        const Hit h1 = runPick(picker.get());
        picker->reset();
        const Hit h2 = runPick(picker.get());
        ck.check(h1.ok && h2.ok && h1.frame == h2.frame && h1.point == h2.point,
                 "用例7：reset() 后同一实例可重新扫描并给出相同结果");
    }

    // —— 用例 8：跨克隆共享结果集（IntersectionVisitor 遇到 Transform 会 push_clone）——
    // 手工构造"克隆链"，断言结果仍然落回调用方持有的那个实例
    {
        auto picker = osg::ref_ptr<CloudRayIntersector>(
            new CloudRayIntersector(osg::Vec3d(-10, 5, 0), osg::Vec3d(110, 5, 0), S));
        picker->setPickRadius(6.0);
        osgUtil::IntersectionVisitor iv(picker.get());
        osg::ref_ptr<osgUtil::Intersector> c1 = picker->clone(iv);
        osg::ref_ptr<osgUtil::Intersector> c2 = c1->clone(iv);   // 二级克隆
        // 用二级克隆去扫描，结果必须出现在 picker 自己的结果集里
        auto* cc2 = dynamic_cast<CloudRayIntersector*>(c2.get());
        ck.check(cc2 != nullptr, "用例8a：clone() 返回的仍是 CloudRayIntersector");
        if (cc2) {
            const Hit h = runPick(cc2);
            ck.check(h.ok, "用例8b：二级克隆也能完成扫描");
            ck.check(picker->containsIntersections(),
                     "用例8c：结果落在调用方持有的实例上（_parent 直指根，避免多级克隆丢结果）");
        }
    }
}

// ---------------------------------------------------------------------------
// 段 B：真实地图往返
// ---------------------------------------------------------------------------

void sectionRealMap(Checker& ck, const std::string& mapDir, int sampleFrames) {
    ck.section("B. 真实地图往返（随机取帧/点构造射线，断言往返一致）");

    SilentProgress progress;
    auto graph = std::make_shared<InteractiveGraph>();
    const auto tLoad = std::chrono::steady_clock::now();
    if (!graph->load_map_data(mapDir, progress)) {
        std::printf("\n错误：地图加载失败\n");
        ck.check(false, "地图加载成功");
        return;
    }
    const double loadMs = std::chrono::duration<double, std::milli>(
                              std::chrono::steady_clock::now() - tLoad).count();
    std::printf("\n地图加载完成：%zu 关键帧，%.1f s\n", graph->keyframes.size(), loadMs / 1000.0);

    CloudPoseTable poses;
    ChunkBuildOptions opt;
    opt.lodLevels = 2;   // 拾取与 LOD 无关，这里少建几级以缩短自检时间
    ChunkBuildResult res = buildChunks(graph, poses, opt);
    std::printf("分块：%zu chunk / %llu 点\n", res.chunks.size(),
                static_cast<unsigned long long>(res.totalPoints));

    auto src = std::make_shared<ChunkPickSource>(
        &res.chunks, opt.chunkFrames, &poses, CloudPoseTable::kSlotOptimized);

    const size_t nf = src->frameCount();
    ck.check(nf > 0, "帧级数据源可用");
    if (nf == 0) return;

    std::mt19937 rng(20260911u);
    std::uniform_int_distribution<size_t> fd(0, nf - 1);
    std::uniform_real_distribution<double> ud(-1.0, 1.0);

    const int k = sampleFrames;
    const double kRadius = 0.05;
    int noHit = 0, worldBad = 0, perpBad = 0, selfBad = 0, tested = 0;

    for (int s = 0; s < k; ++s) {
        const size_t f = fd(rng);
        size_t count = 0;
        float lo[3], hi[3];
        const float* pts = src->framePoints(f, count, lo, hi);
        if (!pts || count == 0) continue;
        ++tested;

        const size_t pi = static_cast<size_t>(rng() % count);
        const Eigen::Vector3d pl(pts[3 * pi], pts[3 * pi + 1], pts[3 * pi + 2]);
        const Eigen::Vector3d w = src->framePose(f) * pl;      // 期望的世界坐标

        // 射线从该点出发、沿随机方向延伸 1 m：该点自身必在 t=0、垂距 0 处
        Eigen::Vector3d dir(ud(rng), ud(rng), ud(rng));
        if (dir.norm() < 1e-6) dir = Eigen::Vector3d::UnitX();
        dir.normalize();
        const osg::Vec3d a(w.x(), w.y(), w.z());
        const osg::Vec3d b(w.x() + dir.x(), w.y() + dir.y(), w.z() + dir.z());

        const Hit h = pickWith(src, a, b, kRadius);
        if (!h.ok) { ++noHit; continue; }

        // ① 命中的 (帧,点) 复算世界坐标，应与返回的世界点一致
        size_t hc = 0;
        float hlo[3], hhi[3];
        const float* hpts = (h.frame >= 0) ? src->framePoints(static_cast<size_t>(h.frame), hc, hlo, hhi)
                                           : nullptr;
        if (!hpts || h.point < 0 || static_cast<size_t>(h.point) >= hc) {
            ++selfBad;
        } else {
            const Eigen::Vector3d hp(hpts[3 * h.point], hpts[3 * h.point + 1], hpts[3 * h.point + 2]);
            const Eigen::Vector3d hw = src->framePose(static_cast<size_t>(h.frame)) * hp;
            if (!near3(h.world, hw, 1e-4)) ++selfBad;
        }

        // ② 返回的世界点必须真的在半径内（顺序颠倒/漏逆变换会破坏这一条）
        if (perpToSegment(h.world, a, b) > kRadius + 1e-6) ++perpBad;

        // ③ 起始点自身应当在半径内命中，返回点不应离它太远
        if ((h.world - a).length() > 1.0 + 1e-6) ++worldBad;
    }

    std::printf("  抽样 %d：实际测点 %d，无命中 %d，世界点越界 %d，垂距超半径 %d，自洽失败 %d\n",
                k, tested, noHit, worldBad, perpBad, selfBad);
    ck.check(tested > 0, "至少取到一个有效帧×点样本");
    ck.check(noHit == 0, "射线上必有点：所有样本都应命中（无自遮挡假设）");
    ck.check(selfBad == 0, "返回的 (帧,点) 复算世界坐标 == 返回的世界点");
    ck.check(perpBad == 0, "返回世界点到射线的垂距 <= 半径（核心不变量）");
    ck.check(worldBad == 0, "返回世界点落在射线 1 m 段内");

    // 远在场景之外的射线必须无命中
    {
        const size_t mid = nf / 2;
        size_t c0 = 0;
        float l0[3], h0[3];
        src->framePoints(mid, c0, l0, h0);
        const Eigen::Vector3d far(static_cast<double>(h0[0]) + 1.0e6,
                                  static_cast<double>(h0[1]) + 1.0e6,
                                  static_cast<double>(h0[2]) + 1.0e6);
        const Hit h = pickWith(src, osg::Vec3d(far.x(), far.y(), far.z()),
                               osg::Vec3d(far.x() + 10.0, far.y(), far.z()), 0.05);
        ck.check(!h.ok, "远离地图的射线无命中");
    }

    // 候选帧粗筛必须把"半径内但 AABB 外"的点也算进来：
    // 构造一条从帧 AABB 外侧擦过的射线，半径取大到足以覆盖，仍应命中
    {
        size_t c0 = 0;
        float l0[3], h0[3];
        const float* p0 = nullptr;
        size_t fFound = static_cast<size_t>(-1);
        for (size_t f = 0; f < nf; ++f) {
            p0 = src->framePoints(f, c0, l0, h0);
            if (p0 && c0 > 0) { fFound = f; break; }
        }
        if (p0 && c0 > 0) {
            const size_t pi = static_cast<size_t>(rng() % c0);
            const Eigen::Vector3d w =
                src->framePose(fFound) *
                Eigen::Vector3d(p0[3 * pi], p0[3 * pi + 1], p0[3 * pi + 2]);
            // 射线平行于 +Y 且从点旁边 1 m 处经过（远在该帧 AABB 之外侧），半径 1.5 m
            const osg::Vec3d a(w.x() + 1.0, w.y() - 5.0, w.z());
            const osg::Vec3d b(w.x() + 1.0, w.y() + 5.0, w.z());
            const Hit h = pickWith(src, a, b, 1.5);
            ck.check(h.ok && perpToSegment(h.world, a, b) <= 1.5 + 1e-6,
                     "半径内但帧 AABB 外侧的点也能被粗筛放行（半径已参与 AABB 膨胀）");
        } else {
            ck.check(false, "找到一个非空帧用于 AABB 膨胀验证");
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    if (argc >= 2 && (std::strcmp(argv[1], "-h") == 0 || std::strcmp(argv[1], "--help") == 0)) {
        usage();
        return 0;
    }
    const std::string mapDir = argValue(argc, argv, "--map", "");
    const int sampleFrames = std::atoi(argValue(argc, argv, "--sample", "200").c_str());

    std::printf("=== cloud_pick_selftest ===\n");
    if (!mapDir.empty()) std::printf("地图: %s\n", mapDir.c_str());
    std::printf("\n");

    Checker ck;
    sectionSynthetic(ck);
    if (!mapDir.empty()) sectionRealMap(ck, mapDir, sampleFrames);

    const int rc = ck.summary();
    std::printf(rc == 0 ? "\n结果：全部通过 ✓\n" : "\n结果：存在失败项 ✗\n");
    return rc;
}
