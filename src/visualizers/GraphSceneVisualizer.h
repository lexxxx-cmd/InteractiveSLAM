#pragma once

#include <osg/Group>
#include <osg/MatrixTransform>

#include <memory>
#include <unordered_map>
#include <Eigen/Geometry>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>

#include "data/hdl_graph_slam/interactive_graph.hpp"
#include "ui/DrawFlags.h"

#include "visualizers/CoordinateAxesVisualizer.h"
#include "visualizers/GroundGridVisualizer.h"
#include "visualizers/VertexSphereVisualizer.h"
#include "visualizers/KeyframePointCloudVisualizer.h"
#include "visualizers/EdgeLineVisualizer.h"

/// @brief Central orchestrator that builds and manages the OSG scene graph
///        from an InteractiveGraph. Owns all sub-visualizers.
///
///        Spheres and edges are built in WORLD SPACE (no MatrixTransform) —
///        same proven pattern as EdgeLineVisualizer.
class GraphSceneVisualizer {
public:
    GraphSceneVisualizer() {
        m_root = new osg::Group;
        m_root->setName("GraphScene");

        // Sub-groups for visibility toggling
        m_sphereGroup = new osg::Group;
        m_sphereGroup->setName("Spheres");
        m_edgeGroup = new osg::Group;
        m_edgeGroup->setName("Edges");
        m_cloudGroup = new osg::Group;
        m_cloudGroup->setName("PointClouds");

        // Static scene elements
        m_axes = std::make_unique<CoordinateAxesVisualizer>();
        m_grid  = std::make_unique<GroundGridVisualizer>(100.0f);

        m_root->addChild(m_grid->getNode());
        m_root->addChild(m_axes->getNode());
        m_root->addChild(m_sphereGroup);
        m_root->addChild(m_edgeGroup);
        m_root->addChild(m_cloudGroup);
    }

    osg::ref_ptr<osg::Group> getRootNode() const { return m_root; }

    // ---- Visibility toggles ----

    void setDrawVertices(bool v) {
        m_sphereGroup->setNodeMask(v ? ~0u : 0u);
    }

    void setDrawEdges(bool v) {
        m_edgeGroup->setNodeMask(v ? ~0u : 0u);
    }

    void setDrawKeyframeClouds(bool v) {
        m_drawClouds = v;
        m_cloudGroup->setNodeMask(v ? ~0u : 0u);
    }

    void setPointSize(float size) {
        m_pointSize = size;
        if (m_cloudViz) m_cloudViz->setPointSize(size);
    }

    void setPointOpacity(float opacity) {
        m_pointOpacity = opacity;
        if (m_cloudViz) m_cloudViz->setOpacity(opacity);
    }

    // ---- Z-clip controls ----

    void setZClipping(bool enabled) {
        if (m_cloudViz) m_cloudViz->setZClipping(enabled);
    }

    void setZClipRange(float minZ, float maxZ) {
        if (m_cloudViz) m_cloudViz->setZClipRange(minZ, maxZ);
    }

    void setColorZRange(float minZ, float maxZ) {
        if (m_cloudViz) m_cloudViz->setColorZRange(minZ, maxZ);
    }

    void setAutoColorRange(bool autoRange) {
        if (m_cloudViz) m_cloudViz->setAutoColorRange(autoRange);
    }

    float getDataZMin() const { return m_cloudViz ? m_cloudViz->getDataZMin() : 0.0f; }
    float getDataZMax() const { return m_cloudViz ? m_cloudViz->getDataZMax() : 0.0f; }
    float getColorZMin() const { return m_cloudViz ? m_cloudViz->getColorZMin() : 0.0f; }
    float getColorZMax() const { return m_cloudViz ? m_cloudViz->getColorZMax() : 0.0f; }
    float getZClipMin() const { return m_cloudViz ? m_cloudViz->getZClipMin() : 0.0f; }
    float getZClipMax() const { return m_cloudViz ? m_cloudViz->getZClipMax() : 0.0f; }
    bool  isZClipping()  const { return m_cloudViz ? m_cloudViz->isZClipping()  : false; }
    bool  isAutoColorRange() const { return m_cloudViz ? m_cloudViz->isAutoColorRange() : true; }

    void setEdgeWidth(float width) {
        m_edgeWidth = width;
        if (m_edgeLineViz) m_edgeLineViz->setLineWidth(width);
    }

    void setSphereRadius(float radius) {
        m_sphereRadius = radius;
        if (m_sphereViz && m_lastGraph) {
            rebuildSpheres(m_lastGraph);
        }
    }

    /// Sphere-centers cache for picking.
    const std::vector<std::pair<osg::Vec3d, long>>& sphereCenters() const {
        return m_sphereCenters;
    }

    /// Edge segments for right-click picking (point-to-segment distance).
    const std::vector<EdgeSegment>& edgeSegments() const {
        return m_edgeLineViz ? m_edgeLineViz->edgeSegments() : m_emptySegments;
    }

    float sphereRadius() const { return m_sphereRadius; }

    void setSelectedVertex(long id) {
        m_selectedVertexId = id;
        if (m_cloudViz) m_cloudViz->recolorHighlight(id);
    }
    long selectedVertex() const { return m_selectedVertexId; }

    // ---- Scene construction ----

    /// Build the entire scene graph from an InteractiveGraph.
    void buildFromGraph(std::shared_ptr<hdl_graph_slam::InteractiveGraph> graph,
                        const hdl_graph_slam::DrawFlags& /*flags*/) {
        clearGraph();

        if (!graph || graph->keyframes.empty()) return;

        m_lastGraph = graph;

        // 1. Spheres — world-space, same vertex positions as edges
        rebuildSpheres(graph);
        m_sphereGroup->addChild(m_sphereViz->getNode());

        // 2. Point cloud — world-space, merged across all keyframes
        rebuildPointClouds(graph);

        // 3. Edges — world-space lines, colored by EdgeSource
        m_edgeLineViz = std::make_unique<EdgeLineVisualizer>();
        m_edgeLineViz->rebuild(graph.get(), m_hiddenEdgeIds);
        m_edgeGroup->addChild(m_edgeLineViz->getNode());

        m_hasGraph = true;
    }

    /// Update spheres and edges every frame.  Point clouds are NOT updated
    /// here (too expensive) — call rebuildPointClouds() after optimization.
    void updatePoses(std::shared_ptr<hdl_graph_slam::InteractiveGraph> graph) {
        if (!graph) return;
        m_lastGraph = graph;

        rebuildSpheres(graph);

        if (m_edgeLineViz) {
            m_edgeLineViz->rebuild(graph.get(), m_hiddenEdgeIds);
        }
    }

    /// Replace the set of hidden edge IDs (called from MainWindow).
    void setHiddenEdges(const std::set<long>& ids) {
        m_hiddenEdgeIds = ids;
    }

    /// Highlight spheres for auto loop detection visualization.
    /// @param sourceId      Vertex ID of the current search source (colored blue).
    /// @param candidateIds  Vertex IDs of loop candidates (colored green).
    void setLoopHighlight(long sourceId, const std::vector<long>& candidateIds) {
        m_loopSourceId = sourceId;
        m_loopCandidateIds.clear();
        m_loopCandidateIds.insert(candidateIds.begin(), candidateIds.end());
    }

    /// Rebuild the merged world-space point cloud from current g2o poses.
    /// Expensive (CPU transform + GPU upload of all points) — call only
    /// after optimization, NOT every frame.
    void rebuildPointClouds(std::shared_ptr<hdl_graph_slam::InteractiveGraph> graph) {
        if (!graph) return;

        if (!m_cloudViz) {
            m_cloudViz = std::make_unique<KeyframePointCloudVisualizer>();
            m_cloudViz->setPointSize(m_pointSize);
            m_cloudViz->setOpacity(m_pointOpacity);
            m_cloudGroup->addChild(m_cloudViz->getNode());
        }

        // Save user's z-clip and color range settings before clear
        bool  savedZClip   = m_cloudViz->isZClipping();
        float savedClipMin = m_cloudViz->getZClipMin();
        float savedClipMax = m_cloudViz->getZClipMax();
        bool  savedAutoColor = m_cloudViz->isAutoColorRange();
        float savedColorMin  = m_cloudViz->getColorZMin();
        float savedColorMax  = m_cloudViz->getColorZMax();

        m_cloudViz->clear();
        for (auto& [id, kf] : graph->keyframes) {
            auto* v = dynamic_cast<g2o::VertexSE3*>(kf->node);
            if (!v || !kf->cloud || kf->cloud->empty()) continue;
            m_cloudViz->appendCloud(kf->cloud, v->estimate(), id);
        }
        m_cloudViz->finish();

        // Restore user's z-clip and color range settings
        m_cloudViz->setZClipping(savedZClip);
        m_cloudViz->setZClipRange(savedClipMin, savedClipMax);
        if (!savedAutoColor) {
            m_cloudViz->setColorZRange(savedColorMin, savedColorMax);
        }

        // Re-apply highlight if one was selected
        if (m_selectedVertexId >= 0) {
            m_cloudViz->recolorHighlight(m_selectedVertexId);
        }
    }

    void clear() {
        clearGraph();
    }

private:
    void rebuildSpheres(std::shared_ptr<hdl_graph_slam::InteractiveGraph> graph) {
        if (!m_sphereViz) {
            m_sphereViz = std::make_unique<VertexSphereVisualizer>(m_sphereRadius);
        }
        m_sphereViz->clear();
        m_sphereViz->setRadius(m_sphereRadius);

        m_sphereCenters.clear();
        m_sphereCenters.reserve(graph->keyframes.size());

        const osg::Vec4 defaultColor(1.0f, 0.0f, 0.0f, 1.0f);    // red
        const osg::Vec4 selectedColor(1.0f, 0.8f, 0.0f, 1.0f);   // orange
        const osg::Vec4 loopSourceColor(0.0f, 0.0f, 1.0f, 1.0f); // blue
        const osg::Vec4 loopCandColor(0.0f, 1.0f, 0.0f, 1.0f);   // green

        for (auto& [id, kf] : graph->keyframes) {
            auto* v = dynamic_cast<g2o::VertexSE3*>(kf->node);
            if (!v) continue;
            Eigen::Vector3d pos = v->estimate().translation();
            osg::Vec3d center(pos.x(), pos.y(), pos.z());

            m_sphereCenters.emplace_back(center, id);

            osg::Vec4 color = defaultColor;
            if (id == m_selectedVertexId) {
                color = selectedColor;
            } else if (id == m_loopSourceId) {
                color = loopSourceColor;
            } else if (m_loopCandidateIds.count(id)) {
                color = loopCandColor;
            }
            m_sphereViz->appendSphere(center, color);
        }
        m_sphereViz->finish();
    }

    void clearGraph() {
        m_sphereGroup->removeChildren(0, m_sphereGroup->getNumChildren());
        m_edgeGroup->removeChildren(0, m_edgeGroup->getNumChildren());
        m_cloudGroup->removeChildren(0, m_cloudGroup->getNumChildren());
        m_sphereViz.reset();
        m_cloudViz.reset();
        m_edgeLineViz.reset();
        m_hasGraph = false;
    }

    // Root
    osg::ref_ptr<osg::Group> m_root;

    // Static elements
    std::unique_ptr<CoordinateAxesVisualizer> m_axes;
    std::unique_ptr<GroundGridVisualizer>  m_grid;

    // Dynamic sub-groups (for visibility toggle)
    osg::ref_ptr<osg::Group> m_sphereGroup;
    osg::ref_ptr<osg::Group> m_edgeGroup;
    osg::ref_ptr<osg::Group> m_cloudGroup;

    // Dynamic visualizers (world-space geometry)
    std::unique_ptr<VertexSphereVisualizer>       m_sphereViz;
    std::unique_ptr<KeyframePointCloudVisualizer> m_cloudViz;
    std::unique_ptr<EdgeLineVisualizer>           m_edgeLineViz;

    // Cached graph reference for live parameter changes
    std::shared_ptr<hdl_graph_slam::InteractiveGraph> m_lastGraph;

    // Sphere centers cache for picking (parallel to VBO, refreshed on rebuild)
    std::vector<std::pair<osg::Vec3d, long>> m_sphereCenters;
    // Fallback for edgeSegments() when no edges loaded
    mutable std::vector<EdgeSegment> m_emptySegments;
    // Edge IDs hidden by user via EdgeListPanel
    std::set<long> m_hiddenEdgeIds;
    // Auto loop detection highlights
    long m_loopSourceId = -1;
    std::set<long> m_loopCandidateIds;
    long m_selectedVertexId = -1;

    // State
    bool m_hasGraph    = false;
    bool m_drawClouds  = true;
    float m_sphereRadius  = 1.0f;
    float m_edgeWidth     = 2.0f;
    float m_pointSize     = 3.0f;
    float m_pointOpacity  = 1.0f;
};
