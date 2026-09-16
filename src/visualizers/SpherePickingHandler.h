// ============================================================================
// SpherePickingHandler.h
// 球体/边线拾取处理器
//
// 功能：OSG 事件处理器，用于在 3D 场景中拾取顶点球体和边线段，
//       并提供右键上下文菜单支持。
//
// 交互方式：
//   - Ctrl + 左键单击：拾取最近的顶点球体（触发选择回调）
//   - 左键双击：射线柱拾取点云最近点作为视角焦点（触发居中回调）；
//     未命中点云时回退为拾取最近顶点球体（旧行为，含空白取消聚焦）
//   - Ctrl + 左键双击：拾取最近的顶点球体（触发帧视角回调）
//   - 右键点击：按交点归属仲裁——射线实际打到视锥体标记几何 → 顶点
//     菜单；否则用 PolytopeIntersector 窗口小矩形拾取边线（GL_LINES
//     线图元）命中 → 边菜单；主路径漏检时回退全交点距离匹配兜底；
//     皆未命中 → 空命中（回调方据此弹出场景菜单）
//
// 注：OSG 3.6.5 的 LineSegmentIntersector 对 GL_LINES / GL_POINTS 图元
// 是空实现（src/osgUtil/LineSegmentIntersector.cpp 中线/点分支为空壳），
// 因此边线拾取不能依赖射线投射，必须走 PolytopeIntersector 的线图元
// 相交测试（真实实现）。
//
// 使用延迟提供者函数（lazy provider），确保处理器始终获取最新的
// 球心/边线段数据（在每次图加载后更新），而非构造时的快照。
// ============================================================================

#pragma once

#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osg/Geometry>
#include <osgUtil/LineSegmentIntersector>
#include <osgUtil/PolytopeIntersector>

#include <algorithm>
#include <cstring>
#include <functional>
#include <vector>
#include <limits>

#include "visualizers/EdgeLineVisualizer.h"   // 引入 EdgeSegment

/**
 * @brief 拾取命中数据结构 —— 分两阶段填充
 *
 * 阶段 1：SpherePickingHandler 填充 screenX/Y + vertexId 或 edgeId/edge* 字段。
 * 阶段 2：ViewportWidget 通过查询 KeyFrame 填充 vertex* 字段。
 */
struct PickingHit {
    long vertexId = -1;  ///< 拾取的顶点 ID（-1 表示未拾取到顶点）
    // --- 顶点字段（由 ViewportWidget 填充） ---
    long   vtxCloudSize = 0;  ///< 顶点的点云点数
    double vtxPosX = 0, vtxPosY = 0, vtxPosZ = 0;  ///< 顶点世界坐标
    double vtxAccumDist = 0;  ///< 顶点累计里程距离
    int    vtxDegree  = 0;    ///< 顶点度数（关联的边数）
    // --- 边字段（由 SpherePickingHandler 填充） ---
    long edgeId   = -1;       ///< 拾取的边 ID（-1 表示未拾取到边）
    long edgeV1   = -1, edgeV2 = -1;  ///< 边连接的两个顶点 ID
    double edgeDist = 0;      ///< 边的长度
    std::string edgeKernel;   ///< 边的鲁棒核函数类型
    float screenX = 0, screenY = 0;  ///< 点击时的屏幕坐标
};

/**
 * @brief 射线交点及其归属类别
 *
 * raycast 在收集世界坐标交点的同时，按交点 nodePath 中包含的
 * 场景子组名判定归属（GraphSceneVisualizer 的子组已 setName）：
 *   - Vertex：交点位于 "Spheres" 子树（顶点视锥体标记几何本身）
 *   - Edge：  交点位于 "Edges"  子树（边线 GL_LINES 几何）
 *   - Other： 其余（地面网格、点云、坐标轴等）
 *
 * 注意：LineSegmentIntersector 在 OSG 3.6.5 下对线/点图元不做相交
 * 测试（空实现），因此 Edge 归属交点在 raycast() 返回结果中实际
 * 不会出现；该分类保留供 PolytopeIntersector 线拾取路径
 * （raycastEdges）复用。
 *
 * 归属判据是"nodePath 上是否存在该名字的节点"而非尾节点名——
 * nodePath 尾部通常是 Geode/Geometry，子组在其上层。
 */
struct RayHit {
    osg::Vec3d point;   ///< 交点世界坐标
    enum class Kind { Vertex, Edge, Other };
    Kind kind = Kind::Other;
};

/**
 * @brief 可拾取的顶点标记数据（标记中心 + 顶点 ID + 拾取半径）
 *
 * pickRadius 为该标记的世界空间拾取阈值：标记渲染为相机视锥体，
 * 远平面四角到锥顶最远约 2.3 倍特征尺寸，故 pickRadius 取
 * 2.5 倍特征尺寸，保证标记可见部分全部可点中。高亮放大的标记
 * （2 倍尺寸）自动获得成比例的更大拾取半径。
 */
struct PickableCenter {
    osg::Vec3d center;      ///< 标记中心（世界坐标）
    long      vertexId;     ///< 对应顶点 ID
    float     pickRadius;   ///< 拾取阈值（世界空间距离）
};

/**
 * @brief 球体和边线的拾取事件处理器
 *
 * 使用延迟提供者函数（std::function）来获取最新的数据，
 * 确保在每次图加载或重建后拾取数据是最新的，无需重新构造处理器。
 */
class SpherePickingHandler : public osgGA::GUIEventHandler {
public:
    using SelectionCallback   = std::function<void(long)>;           ///< 选择回调（参数为顶点 ID）
    using ContextMenuCallback = std::function<void(const PickingHit&)>; ///< 右键菜单回调
    using DoubleClickCallback = std::function<void(long)>;           ///< 双击兜底回调（球体聚焦；-1 = 未命中）
    using FocusPointCallback  = std::function<void(const osg::Vec3d&)>; ///< 双击点云命中回调（居中焦点，世界坐标）
    using FrameViewCallback   = std::function<void(long)>;           ///< Ctrl+双击回调（帧视角；-1 = 未命中）

    /// 球心数据提供者：返回当前可拾取标记数据向量指针（可能为空）
    using SphereProvider = std::function<const std::vector<PickableCenter>*()>;
    /// 边线段数据提供者：返回当前边线段数据向量指针（可能为空）
    using EdgeProvider   = std::function<const std::vector<EdgeSegment>*()>;
    /// 球体半径提供者：返回当前渲染特征尺寸（运行期可变，如渲染面板滑块调整）
    using RadiusProvider = std::function<float()>;

    /**
     * @brief 构造函数
     *
     * @param sphereProvider  球心数据延迟提供函数
     * @param edgeProvider    边线段数据延迟提供函数
     * @param sphereRadius    球体半径（用于拾取距离阈值判断的兜底初值）
     * @param onSelect        选中顶点时的回调函数
     * @param onContextMenu   右键上下文菜单回调函数
     * @param onDoubleClick   左键双击兜底回调（球体聚焦；-1 = 未命中/空白）
     * @param onFocusPoint    双击点云命中回调（参数为焦点世界坐标）
     * @param onFrameView     Ctrl+双击回调（切换到帧视角；-1 = 未命中）
     * @param radiusProvider  实时球体半径提供函数（可空；空时退回
     *                        sphereRadius 快照）。右键边命中阈值依赖它
     *                        随运行期尺寸调整保持正确（快照会失真）
     */
    SpherePickingHandler(SphereProvider sphereProvider,
                         EdgeProvider   edgeProvider,
                         float sphereRadius,
                         SelectionCallback onSelect,
                         ContextMenuCallback onContextMenu,
                         DoubleClickCallback onDoubleClick = DoubleClickCallback(),
                         FocusPointCallback onFocusPoint = FocusPointCallback(),
                         FrameViewCallback onFrameView = FrameViewCallback(),
                         RadiusProvider radiusProvider = RadiusProvider())
        : m_sphereProvider(std::move(sphereProvider))
        , m_edgeProvider(std::move(edgeProvider))
        , m_sphereRadius(sphereRadius)
        , m_onSelect(std::move(onSelect))
        , m_onContextMenu(std::move(onContextMenu))
        , m_onDoubleClick(std::move(onDoubleClick))
        , m_onFocusPoint(std::move(onFocusPoint))
        , m_onFrameView(std::move(onFrameView))
        , m_radiusProvider(std::move(radiusProvider))
    {}

    /**
     * @brief OSG 事件处理入口
     *
     * 处理两种交互事件：
     *   1. Ctrl + 左键：通过射线检测选择最近顶点
     *   2. 右键：通过射线检测优先选择顶点，未命中则选择边
     *
     * @param ea 事件适配器
     * @param aa 动作适配器（可转为 osgViewer::Viewer 进行射线检测）
     * @return 事件是否已处理
     */
    bool handle(const osgGA::GUIEventAdapter& ea,
                osgGA::GUIActionAdapter& aa) override
    {
        // ---- Ctrl + 左键：顶点选择 ----
        if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH &&
            ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON &&
            (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)) {

            auto* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
            auto* centers = m_sphereProvider();
            // 无数据时取消选择
            if (!viewer || !centers || centers->empty()) {
                m_onSelect(-1);
                return true;
            }
            // 射线检测（全部交点）并查找最近的标记
            auto hits = raycast(ea.getX(), ea.getY(), viewer);
            if (hits.empty()) { m_onSelect(-1); return true; }
            m_onSelect(nearestCenter(hits, *centers));
            return true;
        }

        // ---- 左键双击：点云居中（Ctrl+双击：切换到帧视角） ----
        if (ea.getEventType() == osgGA::GUIEventAdapter::DOUBLECLICK &&
            ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON) {

            auto* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);

            // Ctrl + 双击：拾取最近顶点 → 切换到该帧位姿视角
            if (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL) {
                if (m_onFrameView) {
                    long id = -1;
                    auto* centers = m_sphereProvider();
                    if (viewer && centers && !centers->empty()) {
                        auto hits = raycast(ea.getX(), ea.getY(), viewer);
                        if (!hits.empty()) id = nearestCenter(hits, *centers);
                    }
                    m_onFrameView(id);
                }
                return true;
            }

            // 普通双击：射线柱拾取点云 → 命中点作为居中焦点
            if (m_onFocusPoint) {
                osg::Vec3d pt;
                if (viewer &&
                    raycastCloudPoint(ea.getX(), ea.getY(), viewer, pt)) {
                    m_onFocusPoint(pt);
                    return true;
                }
            }

            // 未命中点云 → 回退旧行为（双击球体聚焦 / 空白取消聚焦）
            if (m_onDoubleClick) {
                long id = -1;
                auto* centers = m_sphereProvider();
                if (viewer && centers && !centers->empty()) {
                    auto hits = raycast(ea.getX(), ea.getY(), viewer);
                    if (!hits.empty()) id = nearestCenter(hits, *centers);
                }
                m_onDoubleClick(id);
            }
            return true;
        }

        // ---- 右键：上下文菜单（按交点归属仲裁：视锥体几何 → 顶点，
        //      边线附近 → 边，空白 → 空命中） ----
        if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH &&
            ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) {

            auto* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
            if (!viewer) return false;

            PickingHit hitInfo;
            hitInfo.screenX = ea.getX();
            hitInfo.screenY = ea.getY();

            // 射线检测场景交点（全部，近 → 远，含归属类别）
            auto hits = raycast(ea.getX(), ea.getY(), viewer);
            if (!hits.empty()) {
                // —— 顶点命中（收紧版） ——
                // 旧逻辑是"任何交点落入标记 2.5R 拾取球即命中"，而边线
                // 端点恰为标记中心（拾取球球心），点边几乎必然先落进顶点
                // 拾取球，边命中被完全遮蔽。现收紧为"射线实际打到视锥体
                // 几何本身（Vertex 归属交点）且该交点落在标记 pickRadius
                // 内"才命中顶点——点锥体本体仍出顶点菜单（AC-1.3），
                // 点边线/空白不再被大球吞掉（AC-1.1/1.2）。
                long vertexId = -1;
                std::vector<RayHit> vertexHits;
                for (const auto& hit : hits) {
                    if (hit.kind == RayHit::Kind::Vertex) vertexHits.push_back(hit);
                }
                auto* centers = m_sphereProvider();
                if (!vertexHits.empty() && centers && !centers->empty())
                    vertexId = nearestCenter(vertexHits, *centers);

                // —— 边命中（窗口矩形线图元拾取 + 距离匹配兜底） ——
                // 主路径：OSG 3.6.5 的 LineSegmentIntersector 对线图元
                // 不做相交测试（空实现），Edge 归属交点永远不会出现，
                // 必须用 PolytopeIntersector 的窗口小矩形 + LINE_PRIMITIVES
                // 掩码做线拾取（见 raycastEdges 注释）。
                // 兜底：主路径只认「线段穿过点击点约 5 像素邻域」的命中，
                // 点击偏移略大而射线恰在贴地边附近打到地面网格时，全交点
                // nearestSegment 距离匹配仍可命中。阈值 = 实时球体
                // 半径 × 2.5，与 pickRadius 系数对齐（m_sphereRadius 是
                // 构造快照，运行期调尺寸会失真）。
                // 仲裁：顶点未命中时先走线图元拾取主路径，主路径无候选
                // 才回退全交点距离匹配兜底。
                if (vertexId < 0) {
                    auto* edges = m_edgeProvider();
                    if (edges && !edges->empty()) {
                        bool edgeMatched = false;
                        // 主路径：窗口矩形拾取 GL_LINES 线图元
                        std::vector<size_t> candidates;
                        raycastEdges(ea, viewer, candidates);
                        if (!candidates.empty()) {
                            // 过滤越界下标（防御性，正常不会发生）
                            candidates.erase(
                                std::remove_if(candidates.begin(), candidates.end(),
                                               [edges](size_t i) { return i >= edges->size(); }),
                                candidates.end());
                            if (!candidates.empty()) {
                                // 多候选（多条边共用端点/交叉重叠）时按
                                // "视线射线到线段的最短距离"取最近一条
                                osg::Vec3d rayStart, rayDir;
                                if (!windowRay(ea, viewer, rayStart, rayDir)) {
                                    rayStart = osg::Vec3d();
                                    rayDir   = osg::Vec3d(0, 0, -1);
                                }
                                size_t bestIdx = candidates.front();
                                double bestD2 = std::numeric_limits<double>::max();
                                for (size_t idx : candidates) {
                                    const double d2 =
                                        raySegmentDist2(rayStart, rayDir, (*edges)[idx]);
                                    if (d2 < bestD2) { bestD2 = d2; bestIdx = idx; }
                                }
                                // 命中：从 m_edgeSegments[bestIdx] 填充边字段
                                const auto& seg = (*edges)[bestIdx];
                                hitInfo.edgeId    = seg.id;
                                hitInfo.edgeV1    = seg.v1_id;
                                hitInfo.edgeV2    = seg.v2_id;
                                hitInfo.edgeDist  = seg.distance;
                                hitInfo.edgeKernel = seg.kernel;
                                edgeMatched = true;
                            }
                        }
                        // 兜底：原全交点距离匹配（处理贴地轨迹等主路径
                        // 漏检场景），逻辑保持不变
                        if (!edgeMatched) {
                            const float threshold = currentSphereRadius() * 2.5f;
                            matchEdge(hits, *edges, threshold, hitInfo);
                        }
                    }
                }

                hitInfo.vertexId = vertexId;
            }
            m_onContextMenu(hitInfo);
            return true;
        }
        return false;
    }

private:
    /**
     * @brief 从给定屏幕坐标发射射线，收集场景沿线的所有交点
     *
     * 使用 osgUtil::LineSegmentIntersector 进行窗口坐标到场景坐标的
     * 射线投射。返回按离相机距离排序的全部交点（近 → 远），每个
     * 交点按其 nodePath 归属分类为 Vertex / Edge / Other。
     *
     * 不再只取第一个交点：地面网格（z=0 平面三角面片）等前景几何
     * 会"截胡"第一个交点，导致点击 z=0 以下的标记失效；把所有交点
     * 交给调用方逐一匹配即可绕过遮挡问题。
     *
     * @param x      屏幕 X 坐标
     * @param y      屏幕 Y 坐标
     * @param viewer OSG 查看器指针
     * @return 交点列表（含归属类别，可能为空）
     */
    static std::vector<RayHit> raycast(float x, float y,
                                       osgViewer::Viewer* viewer) {
        std::vector<RayHit> hits;
        osg::ref_ptr<osgUtil::LineSegmentIntersector> picker =
            new osgUtil::LineSegmentIntersector(
                osgUtil::Intersector::WINDOW, x, y);
        osgUtil::IntersectionVisitor iv(picker.get());
        viewer->getCamera()->accept(iv);
        if (!picker->containsIntersections()) return hits;
        const auto& intersections = picker->getIntersections();
        for (const auto& isect : intersections) {
            RayHit hit;
            hit.point = isect.getWorldIntersectPoint();
            hit.kind  = classifyHit(isect.nodePath);
            hits.push_back(hit);
        }
        return hits;
    }

    /**
     * @brief 按交点 nodePath 判定归属子树
     *
     * 遍历 nodePath 匹配子组名（"Spheres"/"Edges"，见
     * GraphSceneVisualizer 构造函数中的 setName）。名字匹配用
     * strcmp 精确比对：nodePath 上可能出现同名自定义节点，子组
     * 名是本模块约定的保留名，精确匹配即可。两者皆不匹配时归为
     * Other（地面网格、点云、坐标轴等）。
     *
     * @param nodePath 交点记录的节点路径（相机 → Drawable 全链）
     * @return 归属类别
     */
    static RayHit::Kind classifyHit(const osg::NodePath& nodePath) {
        bool inSpheres = false, inEdges = false;
        for (const auto* node : nodePath) {
            if (!node) continue;
            const char* name = node->getName().c_str();
            if      (std::strcmp(name, "Spheres") == 0) inSpheres = true;
            else if (std::strcmp(name, "Edges")   == 0) inEdges   = true;
        }
        if (inSpheres) return RayHit::Kind::Vertex;
        if (inEdges)   return RayHit::Kind::Edge;
        return RayHit::Kind::Other;
    }

    /**
     * @brief 在射线交点中查找可拾取的标记
     *
     * 遍历（交点 × 标记）组合，找到"交点落在标记 pickRadius 内"
     * 的最近匹配。只要参与匹配的任一交点（含被遮挡的远端交点）进入
     * 某标记的拾取半径即视为命中。
     *
     * 距离匹配使用交点世界坐标，与交点归属类别无关：地面网格/点云
     * 交点（Other）与视锥体交点（Vertex）同样参与——标记低于 z=0
     * 时网格交点先到，若只认 Vertex 归属交点会重新引入前景遮挡
     * 回归（8589cb9 修复的场景）。
     *
     * @param hits    射线与世界几何的全部交点（按距离排序）
     * @param centers 可拾取标记数据（含各自拾取半径）
     * @return 最近的顶点 ID（未找到返回 -1）
     */
    static long nearestCenter(
            const std::vector<RayHit>& hits,
            const std::vector<PickableCenter>& centers) {
        double bestD2 = std::numeric_limits<double>::max();
        long   bestId = -1;
        for (const auto& [c, id, r] : centers) {
            double thresh2 = static_cast<double>(r) * r;
            for (const auto& hit : hits) {
                double d2 = (c - hit.point).length2();
                if (d2 <= thresh2 && d2 < bestD2) { bestD2 = d2; bestId = id; }
            }
        }
        return bestId;
    }

    /**
     * @brief 射线柱拾取点云：双击位置发射带半径的射线柱（窗口坐标系
     *        下以拾取点为中心的细长柱体），在与柱体相交的点云点中
     *        返回距视线（射线）最近的一个
     *
     * 仅检查 POINTS 图元（点云），线/面图元不计。OSG 3.6 的
     * LineSegmentIntersector 无带半径的构造，故用 PolytopeIntersector
     * 的窗口矩形柱实现，并按"到视线直线的垂距"排序。
     * 已知限制：柱体受投影近平面（0.1m）限定，相机贴得过近时
     * 目标点可能落在近平面内导致拾取失效。
     *
     * @param x        屏幕 X 坐标（归一化）
     * @param y        屏幕 Y 坐标（归一化）
     * @param viewer   OSG 查看器
     * @param outPoint 输出：命中的点云点（世界坐标）
     * @return 是否命中点云
     */
    static bool raycastCloudPoint(float x, float y,
                                  osgViewer::Viewer* viewer,
                                  osg::Vec3d& outPoint) {
        // 射线柱半径：归一化窗口坐标
        const double kCloudPickRadius = 0.1;
        osg::ref_ptr<osgUtil::PolytopeIntersector> picker =
            new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW,
                                             x - kCloudPickRadius,
                                             y - kCloudPickRadius,
                                             x + kCloudPickRadius,
                                             y + kCloudPickRadius);
        // 只检查点（0 维图元）：线/三角面片（网格、视锥体标记）不计
        picker->setPrimitiveMask(osgUtil::PolytopeIntersector::POINT_PRIMITIVES);
        osgUtil::IntersectionVisitor iv(picker.get());
        viewer->getCamera()->accept(iv);
        if (!picker->containsIntersections()) return false;

        // 相机视线（世界坐标）：双击点窗口坐标 → NDC → 逆(view*proj)
        // 反投影近/远两点得到射线，用于计算各命中点的垂距
        const osg::Camera* cam = viewer->getCamera();
        osg::Matrix invVP = osg::Matrix::inverse(
            cam->getViewMatrix() * cam->getProjectionMatrix());
        const double ndcX = x * 2.0 - 1.0;
        const double ndcY = y * 2.0 - 1.0;
        const osg::Vec3d rayStart = osg::Vec3d(ndcX, ndcY, -1.0) * invVP;
        const osg::Vec3d rayEnd   = osg::Vec3d(ndcX, ndcY,  1.0) * invVP;
        osg::Vec3d dir = rayEnd - rayStart;
        if (dir.length2() < 1e-18) return false;
        dir.normalize();

        bool found = false;
        double bestPerp = 0.0;
        for (const auto& isect : picker->getIntersections()) {
            // 世界坐标 = 局部命中点 × 绘制时参考矩阵（无矩阵即恒等）
            osg::Vec3d world = isect.localIntersectionPoint;
            if (isect.matrix.valid()) world = isect.localIntersectionPoint * (*isect.matrix);
            const double perp = ((world - rayStart) ^ dir).length();
            if (!found || perp < bestPerp) {
                bestPerp = perp;
                outPoint = world;
                found = true;
            }
        }
        return found;
    }

    /**
     * @brief 右键边线拾取主路径：窗口小矩形拾取 GL_LINES 线图元
     *
     * 为什么不能用 LineSegmentIntersector：OSG 3.6.5 对 GL_LINES /
     * GL_POINTS 图元的相交测试是空实现（vcpkg 源码
     * src/osgUtil/LineSegmentIntersector.cpp 第 352-361 / 377-383 行
     * 的线/点分支为空壳，只有三角面片走真实测试），因此射线永远打不中
     * 边线几何，必须改用 PolytopeIntersector——其对线图元的相交测试
     * 是真实实现（判断线段是否穿过拾取体积，见
     * src/osgUtil/PolytopeIntersector.cpp 第 299-319 行），即本文件
     * raycastCloudPoint() 用 POINT_PRIMITIVES 拾点云的同一模式。
     *
     * primitiveIndex → EdgeSegment 下标映射依据（跨模块隐式耦合约定）：
     * PolytopeIntersector 的 _primitiveIndex 逐图元递增，而
     * EdgeLineVisualizer 只有一个 DrawArrays(GL_LINES, 0, N) 图元集，
     * 且其 rebuild() 中顶点对与 m_edgeSegments 按同一循环同序填充，
     * 故 Intersection::primitiveIndex 直接就是 m_edgeSegments 的下标。
     * 若 EdgeLineVisualizer 改为多图元集/乱序构建，此映射需要同步修改。
     *
     * 视锥体线框与坐标轴同为 GL_LINES，但 PolytopeIntersector 的交点
     * 记录 nodePath，用 classifyHit() 过滤后只保留 "Edges" 子树的
     * 命中（与 raycast()/classifyHit 同一套归属判定）。
     *
     * @param ea           事件适配器（取归一化窗口坐标与窗口尺寸）
     * @param viewer       OSG 查看器
     * @param outCandidates 输出：命中的边线段下标候选（已去重，
     *                     顺序为交点遍历顺序，未按距离排序）
     */
    static void raycastEdges(const osgGA::GUIEventAdapter& ea,
                             osgViewer::Viewer* viewer,
                             std::vector<size_t>& outCandidates) {
        outCandidates.clear();
        // 拾取半径：像素 → 归一化窗口坐标（WINDOW 坐标系即 [0,1] 归一化）
        const float kEdgePickRadiusPx = 5.0f;
        float radius = kEdgePickRadiusPx;
        if (ea.getWindowWidth() > 0)
            radius /= static_cast<float>(ea.getWindowWidth());
        else
            radius = 0.005f;  // 窗口宽度非法时退回约半百分比
        osg::ref_ptr<osgUtil::PolytopeIntersector> picker =
            new osgUtil::PolytopeIntersector(osgUtil::Intersector::WINDOW,
                                             ea.getX() - radius,
                                             ea.getY() - radius,
                                             ea.getX() + radius,
                                             ea.getY() + radius);
        // 只检查线图元（GL_LINES）：三角面片（网格/视锥体标记）与点不计
        picker->setPrimitiveMask(osgUtil::PolytopeIntersector::LINE_PRIMITIVES);
        osgUtil::IntersectionVisitor iv(picker.get());
        viewer->getCamera()->accept(iv);
        if (!picker->containsIntersections()) return;

        for (const auto& isect : picker->getIntersections()) {
            // 归属过滤：只收 "Edges" 子树的线命中，剔除视锥体线框、
            // 坐标轴等同为 GL_LINES 的几何
            if (classifyHit(isect.nodePath) != RayHit::Kind::Edge) continue;
            const size_t idx = static_cast<size_t>(isect.primitiveIndex);
            if (std::find(outCandidates.begin(), outCandidates.end(), idx)
                    == outCandidates.end())
                outCandidates.push_back(idx);  // 同一线段可能被多个线-体相交记录
        }
    }

    /**
     * @brief 由窗口坐标反投影得到世界坐标视线射线
     *
     * 与 raycastCloudPoint 同款方法：点击点（归一化窗口坐标）→ NDC →
     * 逆(view*proj) 变换近/远两点得到射线起点与方向。
     *
     * @param ea       事件适配器
     * @param viewer   OSG 查看器
     * @param rayStart 输出：射线上的一点（近平面反投影点，世界坐标）
     * @param rayDir   输出：射线方向（单位向量）
     * @return 是否成功（viewer/矩阵退化时返回 false）
     */
    static bool windowRay(const osgGA::GUIEventAdapter& ea,
                          osgViewer::Viewer* viewer,
                          osg::Vec3d& rayStart, osg::Vec3d& rayDir) {
        const osg::Camera* cam = viewer->getCamera();
        if (!cam) return false;
        const osg::Matrix invVP = osg::Matrix::inverse(
            cam->getViewMatrix() * cam->getProjectionMatrix());
        const double x = ea.getX(), y = ea.getY();
        const double ndcX = x * 2.0 - 1.0;
        const double ndcY = y * 2.0 - 1.0;
        const osg::Vec3d nearPt = osg::Vec3d(ndcX, ndcY, -1.0) * invVP;
        const osg::Vec3d farPt  = osg::Vec3d(ndcX, ndcY,  1.0) * invVP;
        osg::Vec3d dir = farPt - nearPt;
        if (dir.length2() < 1e-18) return false;
        rayStart = nearPt;
        rayDir   = dir / dir.length();
        return true;
    }

    /**
     * @brief 计算视线射线到边线段的最短距离平方
     *
     * 用于多候选排序：PolytopeIntersector 的窗口矩形命中不区分远近，
     * 多条边同时穿过拾取体积（共用端点/交叉重叠）时取距视线最近者。
     * 这里采用点到线段的垂距近似（射线自身长度远大于拾取容差，把
     * "射线-线段"距离退化为"线段到射线起点+方向所在直线"的垂距即可
     * 满足排序需要，无需完整的线段-线段距离）。
     *
     * @param rayStart 视线射线上一点（世界坐标）
     * @param rayDir   视线射线方向（单位向量）
     * @param seg      边线段
     * @return 最短距离平方；零长度线段退化为起点垂距平方
     */
    static double raySegmentDist2(const osg::Vec3d& rayStart,
                                  const osg::Vec3d& rayDir,
                                  const EdgeSegment& seg) {
        // 采样线段两端点到视线直线的垂距，取较小者：
        // 拾取体积仅数像素宽，两端点垂距的较小值足以排序，
        // 无需精确的线段-线段最短距离
        const osg::Vec3d v1 = seg.p1 - rayStart;
        const osg::Vec3d v2 = seg.p2 - rayStart;
        const double d1 = (v1 ^ rayDir).length2();
        const double d2 = (v2 ^ rayDir).length2();
        if (seg.p1 == seg.p2) return d1;  // 零长度线段（退化）
        return std::min(d1, d2);
    }

    /**
     * @brief 当前球体半径（实时值优先，构造快照兜底）
     *
     * 右键边命中阈值依赖它：radiusProvider 未注入或返回非法值时
     * 退回构造时的 m_sphereRadius 快照（旧行为）。
     */
    float currentSphereRadius() const {
        if (m_radiusProvider) {
            const float r = m_radiusProvider();
            if (r > 0.0f) return r;
        }
        return m_sphereRadius;
    }

    /**
     * @brief 用给定交点集合对全部边线段做距离匹配，填充边命中信息
     *
     * 遍历（交点 × 边线段）组合取最近距离，仅在最近距离小于阈值时
     * 视为命中并填充 out 的边字段。
     *
     * @param hits      参与匹配的交点（调用方已按归属筛选）
     * @param segments  边线段数据
     * @param threshold 世界空间距离阈值
     * @param out       输出：命中时填充边元数据，未命中时 edgeId 保持 -1
     * @return 是否命中
     */
    static bool matchEdge(const std::vector<RayHit>& hits,
                          const std::vector<EdgeSegment>& segments,
                          float threshold,
                          PickingHit& out) {
        double bestD2 = std::numeric_limits<double>::max();
        PickingHit best;
        for (const auto& hit : hits) {
            PickingHit tmp;
            double d2 = nearestSegment(hit.point, segments, threshold, tmp);
            if (d2 < bestD2) { bestD2 = d2; best = tmp; }
        }
        if (best.edgeId < 0) return false;
        // 只取边字段，保留 out 原有的屏幕坐标等字段
        out.edgeId     = best.edgeId;
        out.edgeV1     = best.edgeV1;
        out.edgeV2     = best.edgeV2;
        out.edgeDist   = best.edgeDist;
        out.edgeKernel = best.edgeKernel;
        return true;
    }

    /**
     * @brief 查找离射线交点最近的边线段
     *
     * 计算点到线段的最短距离。如果点到线段投影超出端点范围，
     * 则计算到最近端点的距离。
     * 仅在最近距离小于指定阈值时认为拾取成功。
     *
     * @param hit       射线与场景的交点
     * @param segments  边线段数据向量
     * @param threshold 距离阈值
     * @param out       输出：填充对应边的元数据
     * @return 最近距离的平方（未命中返回 double 最大值，供调用方跨交点比较）；
     *         out.edgeId 仅在命中时有效
     */
    static double nearestSegment(const osg::Vec3d& hit,
                                 const std::vector<EdgeSegment>& segments,
                                 float threshold,
                                 PickingHit& out) {
        long   bestIdx = -1;
        double bestD2  = static_cast<double>(threshold) * threshold;
        for (size_t i = 0; i < segments.size(); ++i) {
            const auto& seg = segments[i];
            osg::Vec3d ab = seg.p2 - seg.p1;     // 线段向量
            double len2 = ab.length2();           // 线段长度的平方
            double d2;
            if (len2 < 1e-12) {
                // 退化为点的线段（两端点重合）
                d2 = (seg.p1 - hit).length2();
            } else {
                // 计算点到线段的最短距离
                double t = ((hit - seg.p1) * ab) / len2;
                t = std::max(0.0, std::min(1.0, t));  // 限制在 [0,1] 范围内
                d2 = (seg.p1 + ab * t - hit).length2();
            }
            if (d2 < bestD2) { bestD2 = d2; bestIdx = static_cast<long>(i); }
        }
        if (bestIdx < 0) {
            out.edgeId = -1;
            return std::numeric_limits<double>::max();
        }
        // 填充输出结构
        const auto& seg = segments[bestIdx];
        out.edgeV1    = seg.v1_id;
        out.edgeV2    = seg.v2_id;
        out.edgeDist  = seg.distance;
        out.edgeKernel = seg.kernel;
        out.edgeId    = seg.id;
        return bestD2;
    }

    // —— 成员变量 ——
    SphereProvider      m_sphereProvider;   ///< 球心数据提供函数
    EdgeProvider        m_edgeProvider;     ///< 边线段数据提供函数
    float               m_sphereRadius;     ///< 球体半径（构造快照，兜底用）
    SelectionCallback   m_onSelect;         ///< 选择回调
    ContextMenuCallback m_onContextMenu;    ///< 右键菜单回调
    DoubleClickCallback m_onDoubleClick;    ///< 双击兜底回调（球体聚焦）
    FocusPointCallback  m_onFocusPoint;     ///< 双击点云命中回调（居中焦点）
    FrameViewCallback   m_onFrameView;      ///< Ctrl+双击回调（帧视角）
    RadiusProvider      m_radiusProvider;   ///< 实时球体半径提供函数（可空）
};
