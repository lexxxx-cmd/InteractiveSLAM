#pragma once

#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgUtil/LineSegmentIntersector>

#include <functional>
#include <vector>
#include <limits>
#include <optional>

#include "visualizers/EdgeLineVisualizer.h"   // for EdgeSegment

/// Context menu payload.
struct PickingHit {
    long vertexId = -1;
    long edgeId   = -1;
    long edgeV1   = -1, edgeV2 = -1;
    double edgeDist = 0;
    std::string edgeKernel;
    float screenX = 0, screenY = 0;
};

/// @brief OSG event handler for sphere/edge picking and context menu.
///
///        Uses lazy provider functions so the handler always sees the latest
///        sphere-centers / edge-segments data (populated after graph load),
///        not a snapshot captured at construction time.
class SpherePickingHandler : public osgGA::GUIEventHandler {
public:
    using SelectionCallback   = std::function<void(long)>;
    using ContextMenuCallback = std::function<void(const PickingHit&)>;

    /// Provider: returns a pointer to the current data vector (may be null).
    using SphereProvider = std::function<const std::vector<std::pair<osg::Vec3d, long>>*()>;
    using EdgeProvider   = std::function<const std::vector<EdgeSegment>*()>;

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

    bool handle(const osgGA::GUIEventAdapter& ea,
                osgGA::GUIActionAdapter& aa) override
    {
        // ---- Ctrl + LeftClick: selection ----
        if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH &&
            ea.getButton() == osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON &&
            (ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL)) {

            auto* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
            auto* centers = m_sphereProvider();
            if (!viewer || !centers || centers->empty()) {
                m_onSelect(-1);
                return true;
            }
            auto hit = raycast(ea.getX(), ea.getY(), viewer);
            if (!hit) { m_onSelect(-1); return true; }
            m_onSelect(nearestCenter(*hit, *centers, m_sphereRadius));
            return true;
        }

        // ---- RightClick: context menu ----
        if (ea.getEventType() == osgGA::GUIEventAdapter::PUSH &&
            ea.getButton() == osgGA::GUIEventAdapter::RIGHT_MOUSE_BUTTON) {

            auto* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
            if (!viewer) return false;

            PickingHit hitInfo;
            hitInfo.screenX = ea.getX();
            hitInfo.screenY = ea.getY();

            auto hitPt = raycast(ea.getX(), ea.getY(), viewer);
            if (hitPt) {
                auto* centers = m_sphereProvider();
                if (centers && !centers->empty())
                    hitInfo.vertexId = nearestCenter(*hitPt, *centers, m_sphereRadius);

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

    static long nearestCenter(
            const osg::Vec3d& hit,
            const std::vector<std::pair<osg::Vec3d, long>>& centers,
            float radius) {
        double bestD2  = std::numeric_limits<double>::max();
        long   bestId  = -1;
        double thresh2 = static_cast<double>(radius) * radius;
        for (const auto& [c, id] : centers) {
            double d2 = (c - hit).length2();
            if (d2 < bestD2) { bestD2 = d2; bestId = id; }
        }
        return (bestId >= 0 && bestD2 <= thresh2) ? bestId : -1;
    }

    static long nearestSegment(const osg::Vec3d& hit,
                               const std::vector<EdgeSegment>& segments,
                               float threshold,
                               PickingHit& out) {
        long   bestIdx = -1;
        double bestD2  = static_cast<double>(threshold) * threshold;
        for (size_t i = 0; i < segments.size(); ++i) {
            const auto& seg = segments[i];
            osg::Vec3d ab = seg.p2 - seg.p1;
            double len2 = ab.length2();
            double d2;
            if (len2 < 1e-12) {
                d2 = (seg.p1 - hit).length2();
            } else {
                double t = ((hit - seg.p1) * ab) / len2;
                t = std::max(0.0, std::min(1.0, t));
                d2 = (seg.p1 + ab * t - hit).length2();
            }
            if (d2 < bestD2) { bestD2 = d2; bestIdx = static_cast<long>(i); }
        }
        if (bestIdx < 0) return -1;
        const auto& seg = segments[bestIdx];
        out.edgeV1    = seg.v1_id;
        out.edgeV2    = seg.v2_id;
        out.edgeDist  = seg.distance;
        out.edgeKernel = seg.kernel;
        return seg.id;
    }

    SphereProvider      m_sphereProvider;
    EdgeProvider        m_edgeProvider;
    float               m_sphereRadius;
    SelectionCallback   m_onSelect;
    ContextMenuCallback m_onContextMenu;
};
