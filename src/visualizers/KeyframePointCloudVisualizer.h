#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/Uniform>
#include <osg/BlendFunc>
#include <osg/BlendColor>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "visualizers/TurboColormap.h"
#include "visualizers/CoreShaders.h"

/// @brief Builds a merged point-cloud geometry in WORLD SPACE.
///        Each keyframe's local LiDAR points are transformed by the g2o pose
///        on the CPU, then appended to a single VBO — same pattern as
///        EdgeLineVisualizer and VertexSphereVisualizer.
class KeyframePointCloudVisualizer {
public:
    KeyframePointCloudVisualizer() {
        m_geom = new osg::Geometry;
        m_geom->setUseDisplayList(false);
        m_geom->setUseVertexBufferObjects(true);
        m_geom->setUseVertexArrayObject(true);
        m_geom->setDataVariance(osg::Object::DYNAMIC);  // allow uniform updates (point size)

        m_vertices = new osg::Vec3Array;
        m_colors   = new osg::Vec4Array;

        m_geom->setVertexArray(m_vertices);
        m_geom->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);
        m_geom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, 0));

        auto* ss = m_geom->getOrCreateStateSet();

        // Point-cloud shader with uPointSize uniform
        m_pointSizeUniform = applyPointCloudShader(ss, m_pointSize);

        // Opacity via blend constant alpha
        m_blendColor = new osg::BlendColor(osg::Vec4(1, 1, 1, m_opacity));
        ss->setAttributeAndModes(m_blendColor, osg::StateAttribute::ON);
        ss->setAttributeAndModes(
            new osg::BlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA),
            osg::StateAttribute::ON);

        m_geode = new osg::Geode;
        m_geode->addDrawable(m_geom);
    }

    /// Append one keyframe's point cloud, transformed to world space.
    /// @param cloud   LiDAR points in local (sensor) frame
    /// @param pose    g2o-estimated world pose of this keyframe
    void appendCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                     const Eigen::Isometry3d& pose) {
        if (!cloud || cloud->empty()) return;

        // Track global Z range for turbo colour mapping
        for (const auto& pt : cloud->points) {
            Eigen::Vector3d wp = pose * Eigen::Vector3d(pt.x, pt.y, pt.z);
            float wz = static_cast<float>(wp.z());
            if (wz < m_zMin) m_zMin = wz;
            if (wz > m_zMax) m_zMax = wz;
            m_allWorldPoints.push_back(wp);
        }
    }

    /// Call after all appendCloud() calls.  Colours points by world-space Z
    /// using the Turbo colormap and uploads to GPU.
    void finish() {
        if (m_allWorldPoints.empty()) return;

        if (m_zMax - m_zMin < 0.001f) {
            m_zMin -= 0.5f;
            m_zMax += 0.5f;
        }

        m_vertices->reserve(m_allWorldPoints.size());
        m_colors->reserve(m_allWorldPoints.size());

        for (const auto& wp : m_allWorldPoints) {
            m_vertices->push_back(osg::Vec3(
                static_cast<float>(wp.x()),
                static_cast<float>(wp.y()),
                static_cast<float>(wp.z())));
            m_colors->push_back(
                turboColor(static_cast<float>(wp.z()), m_zMin, m_zMax));
        }

        m_vertices->dirty();
        m_colors->dirty();

        auto* prim = static_cast<osg::DrawArrays*>(m_geom->getPrimitiveSet(0));
        if (prim) prim->setCount(m_vertices->size());
        m_geom->dirtyBound();

        // Free the intermediate buffer
        m_allWorldPoints.clear();
        m_allWorldPoints.shrink_to_fit();
    }

    void setPointSize(float size) {
        m_pointSize = size;
        if (m_pointSizeUniform)
            m_pointSizeUniform->set(size);
    }

    void setOpacity(float opacity) {
        m_opacity = opacity;
        if (m_blendColor)
            m_blendColor->setConstantColor(osg::Vec4(1, 1, 1, opacity));
    }

    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

    int pointCount() const {
        auto* prim = static_cast<osg::DrawArrays*>(m_geom->getPrimitiveSet(0));
        return prim ? prim->getCount() : 0;
    }

private:
    osg::ref_ptr<osg::Geode>     m_geode;
    osg::ref_ptr<osg::Geometry>  m_geom;
    osg::ref_ptr<osg::Vec3Array> m_vertices;
    osg::ref_ptr<osg::Vec4Array> m_colors;
    osg::ref_ptr<osg::BlendColor> m_blendColor;
    osg::ref_ptr<osg::Uniform>    m_pointSizeUniform;

    // Intermediate buffer: collect world-space points before colour mapping
    std::vector<Eigen::Vector3d,
                Eigen::aligned_allocator<Eigen::Vector3d>> m_allWorldPoints;

    float m_zMin   =  std::numeric_limits<float>::max();
    float m_zMax   = -std::numeric_limits<float>::max();
    float m_pointSize = 3.0f;
    float m_opacity   = 1.0f;
};
