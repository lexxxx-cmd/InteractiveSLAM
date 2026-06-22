#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/LineWidth>
#include <vector>
#include <string>

#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/core/hyper_graph.h>

#include "data/g2o/robust_kernel_io.hpp"

#include "visualizers/CoreShaders.h"

/// Edge segment data for picking and context menu info.
struct EdgeSegment {
    osg::Vec3d p1, p2;      // world-space endpoints
    long id;                 // g2o edge ID
    long v1_id, v2_id;      // connected vertex IDs
    double distance;         // edge length (meters)
    double info_diag;        // trace of information matrix (confidence indicator)
    std::string kernel;      // robust kernel type ("NONE", "Huber", …)
};

/// @brief Renders all SE3 edges as a single merged line geometry.
///        Uses GLSL 330 shader for Core Profile compatibility.
class EdgeLineVisualizer {
public:
    EdgeLineVisualizer() {
        m_geom = new osg::Geometry;
        m_geom->setUseDisplayList(false);
        m_geom->setUseVertexBufferObjects(true);
        m_geom->setUseVertexArrayObject(true);  // mandatory in Core Profile 3.3
        m_geom->setDataVariance(osg::Object::DYNAMIC);

        m_vertices = new osg::Vec3Array;
        m_colors = new osg::Vec4Array;

        m_geom->setVertexArray(m_vertices);
        m_geom->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);
        m_geom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, 0));

        // Custom GLSL 330 shader
        auto* ss = m_geom->getOrCreateStateSet();
        applySimpleColorShader(ss);

        // Line width state attribute
        m_lineWidth = new osg::LineWidth(m_width);
        ss->setAttributeAndModes(m_lineWidth, osg::StateAttribute::ON);

        m_geode = new osg::Geode;
        m_geode->addDrawable(m_geom);
    }

    /// Rebuild all edge lines from the g2o graph.
    /// @param graph  g2o HyperGraph (stored as HyperGraph* in GraphSLAM,
    ///               but always a SparseOptimizer at runtime).
    void rebuild(g2o::HyperGraph* graph) {
        m_vertices->clear();
        m_colors->clear();
        m_edgeSegments.clear();

        if (!graph) return;

        const osg::Vec4 edgeColor(1.0f, 0.0f, 0.0f, 0.7f);

        for (auto* edge : graph->edges()) {
            auto* se3 = dynamic_cast<g2o::EdgeSE3*>(edge);
            if (!se3) continue;

            auto* v1 = dynamic_cast<g2o::VertexSE3*>(se3->vertices()[0]);
            auto* v2 = dynamic_cast<g2o::VertexSE3*>(se3->vertices()[1]);
            if (!v1 || !v2) continue;

            Eigen::Vector3d p1 = v1->estimate().translation();
            Eigen::Vector3d p2 = v2->estimate().translation();

            m_vertices->push_back(osg::Vec3(p1.x(), p1.y(), p1.z()));
            m_vertices->push_back(osg::Vec3(p2.x(), p2.y(), p2.z()));
            m_colors->push_back(edgeColor);
            m_colors->push_back(edgeColor);

            // Store rich edge metadata for picking + context menu
            double dist = (p1 - p2).norm();
            double trace = se3->information().trace();
            std::string kernelName = g2o::kernel_type(se3->robustKernel());
            if (kernelName.empty()) kernelName = "NONE";
            m_edgeSegments.push_back({
                osg::Vec3d(p1.x(), p1.y(), p1.z()),
                osg::Vec3d(p2.x(), p2.y(), p2.z()),
                static_cast<long>(se3->id()),
                v1->id(), v2->id(),
                dist, trace, kernelName
            });
        }

        m_vertices->dirty();
        m_colors->dirty();

        auto* prim = static_cast<osg::DrawArrays*>(m_geom->getPrimitiveSet(0));
        if (prim) prim->setCount(m_vertices->size());
        m_geom->dirtyBound();
    }

    /// Edge segments for right-click picking (point-to-segment distance).
    const std::vector<EdgeSegment>& edgeSegments() const {
        return m_edgeSegments;
    }

    void setLineWidth(float w) {
        m_width = w;
        if (m_lineWidth) m_lineWidth->setWidth(w);
    }

    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

private:
    osg::ref_ptr<osg::Geode> m_geode;
    osg::ref_ptr<osg::Geometry> m_geom;
    osg::ref_ptr<osg::Vec3Array> m_vertices;
    osg::ref_ptr<osg::Vec4Array> m_colors;
    osg::ref_ptr<osg::LineWidth> m_lineWidth;
    float m_width = 2.0f;
    std::vector<EdgeSegment> m_edgeSegments;
};
