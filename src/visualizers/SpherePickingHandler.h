#pragma once

#include <osgGA/GUIEventHandler>
#include <osgViewer/Viewer>
#include <osgUtil/LineSegmentIntersector>

#include <functional>
#include <vector>
#include <limits>

/// @brief OSG event handler that intercepts Ctrl+LeftClick for sphere picking.
///
///        Uses LineSegmentIntersector to cast a ray from the mouse position,
///        then searches the sphere-centers cache (owned by GraphSceneVisualizer)
///        for the nearest sphere within the configured radius threshold.
///
///        Non-Ctrl events pass through (return false), so TrackballManipulator
///        continues to function for camera rotation/pan/zoom.
class SpherePickingHandler : public osgGA::GUIEventHandler {
public:
    /// Callback signature: void(long vertexId).
    /// vertexId == -1 means deselection (clicked empty space).
    using PickingCallback = std::function<void(long)>;

    /// @param centers       Pointer to the sphere-centers cache.
    ///                      Must remain valid for the handler's lifetime.
    /// @param sphereRadius  Radius used as hit-distance threshold.
    /// @param callback      Invoked synchronously during event traversal.
    SpherePickingHandler(
            const std::vector<std::pair<osg::Vec3d, long>>* centers,
            float sphereRadius,
            PickingCallback callback)
        : m_centers(centers)
        , m_sphereRadius(sphereRadius)
        , m_callback(std::move(callback))
    {}

    bool handle(const osgGA::GUIEventAdapter& ea,
                osgGA::GUIActionAdapter& aa) override
    {
        // Only intercept Ctrl+LeftButton PUSH
        if (ea.getEventType() != osgGA::GUIEventAdapter::PUSH)
            return false;
        if (ea.getButton() != osgGA::GUIEventAdapter::LEFT_MOUSE_BUTTON)
            return false;
        if ((ea.getModKeyMask() & osgGA::GUIEventAdapter::MODKEY_CTRL) == 0)
            return false;

        auto* viewer = dynamic_cast<osgViewer::Viewer*>(&aa);
        if (!viewer || !m_centers || m_centers->empty()) {
            m_callback(-1);
            return true;
        }

        // Ray-cast against the scene from the mouse coordinate
        osg::ref_ptr<osgUtil::LineSegmentIntersector> intersector =
            new osgUtil::LineSegmentIntersector(
                osgUtil::Intersector::WINDOW,
                ea.getX(), ea.getY());
        osgUtil::IntersectionVisitor iv(intersector.get());
        viewer->getCamera()->accept(iv);

        if (!intersector->containsIntersections()) {
            m_callback(-1);  // clicked empty space
            return true;
        }

        // Search nearest sphere center within radius threshold
        const auto& hit = intersector->getFirstIntersection();
        osg::Vec3d hitPoint = hit.getWorldIntersectPoint();

        long   nearestId    = -1;
        double minDistSq    = std::numeric_limits<double>::max();
        double thresholdSq  = static_cast<double>(m_sphereRadius) * m_sphereRadius;

        for (const auto& [center, id] : *m_centers) {
            double d2 = (center - hitPoint).length2();
            if (d2 < minDistSq) {
                minDistSq = d2;
                nearestId = id;
            }
        }

        if (nearestId >= 0 && minDistSq <= thresholdSq) {
            m_callback(nearestId);
        } else {
            m_callback(-1);
        }

        return true;  // consume event — TrackballManipulator won't see it
    }

private:
    const std::vector<std::pair<osg::Vec3d, long>>* m_centers;
    float m_sphereRadius;
    PickingCallback m_callback;
};
