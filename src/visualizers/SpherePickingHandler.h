// ============================================================================
// SpherePickingHandler.h
// 球体/边线拾取处理器
//
// 功能：OSG 事件处理器，用于在 3D 场景中拾取顶点球体和边线段，
//       并提供右键上下文菜单支持。
//
// 交互方式：
//   - Ctrl + 左键点击：拾取最近的顶点球体（触发选择回调）
//   - 右键点击：拾取场景对象并触发上下文菜单回调（优先检测球体，后检测边线）
//
// 使用延迟提供者函数（lazy provider），确保处理器始终获取最新的
// 球心/边线段数据（在每次图加载后更新），而非构造时的快照。
// ============================================================================

#pragma once

#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgUtil/LineSegmentIntersector>

#include <functional>
#include <vector>
#include <limits>
#include <optional>

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
 * @brief 球体和边线的拾取事件处理器
 *
 * 使用延迟提供者函数（std::function）来获取最新的数据，
 * 确保在每次图加载或重建后拾取数据是最新的，无需重新构造处理器。
 */
class SpherePickingHandler : public osgGA::GUIEventHandler {
public:
    using SelectionCallback   = std::function<void(long)>;           ///< 选择回调（参数为顶点 ID）
    using ContextMenuCallback = std::function<void(const PickingHit&)>; ///< 右键菜单回调

    /// 球心数据提供者：返回当前球心数据向量指针（可能为空）
    using SphereProvider = std::function<const std::vector<std::pair<osg::Vec3d, long>>*()>;
    /// 边线段数据提供者：返回当前边线段数据向量指针（可能为空）
    using EdgeProvider   = std::function<const std::vector<EdgeSegment>*()>;

    /**
     * @brief 构造函数
     *
     * @param sphereProvider  球心数据延迟提供函数
     * @param edgeProvider    边线段数据延迟提供函数
     * @param sphereRadius    球体半径（用于拾取距离阈值判断）
     * @param onSelect        选中顶点时的回调函数
     * @param onContextMenu   右键上下文菜单回调函数
     */
    SpherePickingHandler(SphereProvider sphereProvider,
                         EdgeProvider   edgeProvider,
                         float sphereRadius,
                         SelectionCallback onSelect,
                         ContextMenuCallback onContextMenu)
        : m_sphereProvider(std::move(sphereProvider))
        , m_edgeProvider(std::move(edgeProvider))
        , m_sphereRadius(sphereRadius)
        , m_onSelect(std::move(onSelect))
        , m_onContextMenu(std::move(onContextMenu))
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
            // 射线检测并查找最近球体
            auto hit = raycast(ea.getX(), ea.getY(), viewer);
            if (!hit) { m_onSelect(-1); return true; }
            m_onSelect(nearestCenter(*hit, *centers, m_sphereRadius));
            return true;
        }

        // ---- 右键：上下文菜单（顶点优先，边作为备选） ----
        if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH &&
            ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) {

            auto* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
            if (!viewer) return false;

            PickingHit hitInfo;
            hitInfo.screenX = ea.getX();
            hitInfo.screenY = ea.getY();

            // 射线检测场景交点
            auto hitPt = raycast(ea.getX(), ea.getY(), viewer);
            if (hitPt) {
                // 优先检测球体（顶点）
                auto* centers = m_sphereProvider();
                if (centers && !centers->empty())
                    hitInfo.vertexId = nearestCenter(*hitPt, *centers, m_sphereRadius);

                // 如果未命中球体，检测边线段
                if (hitInfo.vertexId < 0) {
                    auto* edges = m_edgeProvider();
                    if (edges && !edges->empty())
                        hitInfo.edgeId = nearestSegment(*hitPt, *edges,
                                                        m_sphereRadius * 2.0f,
                                                        hitInfo);
                }
            }
            m_onContextMenu(hitInfo);
            return true;
        }
        return false;
    }

private:
    /**
     * @brief 从给定屏幕坐标发射射线，计算场景交点
     *
     * 使用 osgUtil::LineSegmentIntersector 进行窗口坐标到场景坐标的
     * 射线投射。
     *
     * @param x      屏幕 X 坐标
     * @param y      屏幕 Y 坐标
     * @param viewer OSG 查看器指针
     * @return 世界坐标系下的交点（如果存在），否则 std::nullopt
     */
    static std::optional<osg::Vec3d> raycast(float x, float y,
                                             osgViewer::Viewer* viewer) {
        osg::ref_ptr<osgUtil::LineSegmentIntersector> picker =
            new osgUtil::LineSegmentIntersector(
                osgUtil::Intersector::WINDOW, x, y);
        osgUtil::IntersectionVisitor iv(picker.get());
        viewer->getCamera()->accept(iv);
        if (!picker->containsIntersections()) return std::nullopt;
        return picker->getFirstIntersection().getWorldIntersectPoint();
    }

    /**
     * @brief 查找离射线交点最近的球体中心
     *
     * 遍历所有球心，找到距离射线交点最近的球体。
     * 仅当最近距离小于球体半径阈值时才认为拾取成功。
     *
     * @param hit     射线与场景的交点
     * @param centers 球心数据向量（包含球心位置和顶点 ID）
     * @param radius  球体半径（距离阈值）
     * @return 最近的顶点 ID（未找到返回 -1）
     */
    static long nearestCenter(
            const osg::Vec3d& hit,
            const std::vector<std::pair<osg::Vec3d, long>>& centers,
            float radius) {
        double bestD2  = std::numeric_limits<double>::max();
        long   bestId  = -1;
        double thresh2 = static_cast<double>(radius) * radius;  // 距离平方阈值
        for (const auto& [c, id] : centers) {
            double d2 = (c - hit).length2();
            if (d2 < bestD2) { bestD2 = d2; bestId = id; }
        }
        return (bestId >= 0 && bestD2 <= thresh2) ? bestId : -1;
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
     * @return 最近的边 ID（未找到返回 -1）
     */
    static long nearestSegment(const osg::Vec3d& hit,
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
        if (bestIdx < 0) return -1;
        // 填充输出结构
        const auto& seg = segments[bestIdx];
        out.edgeV1    = seg.v1_id;
        out.edgeV2    = seg.v2_id;
        out.edgeDist  = seg.distance;
        out.edgeKernel = seg.kernel;
        return seg.id;
    }

    // —— 成员变量 ——
    SphereProvider      m_sphereProvider;   ///< 球心数据提供函数
    EdgeProvider        m_edgeProvider;     ///< 边线段数据提供函数
    float               m_sphereRadius;     ///< 球体半径（拾取距离阈值）
    SelectionCallback   m_onSelect;         ///< 选择回调
    ContextMenuCallback m_onContextMenu;    ///< 右键菜单回调
};
