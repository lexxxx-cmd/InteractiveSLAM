#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/Uniform>
#include <osg/BlendFunc>
#include <osg/BlendColor>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

#include <vector>

#include "visualizers/TurboColormap.h"
#include "visualizers/CoreShaders.h"

/// @brief Builds a merged point-cloud geometry in WORLD SPACE.
///        Each keyframe's local LiDAR points are transformed by the g2o pose
///        on the CPU, then appended to a single VBO — same pattern as
///        EdgeLineVisualizer and VertexSphereVisualizer.
///
///        Tracks per-keyframe vertex ranges so the selected keyframe's cloud
///        can be highlighted without rebuilding the entire VBO.
class KeyframePointCloudVisualizer {
public:
    KeyframePointCloudVisualizer() {
        m_geom = new osg::Geometry;
        m_geom->setUseDisplayList(false);
        m_geom->setUseVertexBufferObjects(true);
        m_geom->setUseVertexArrayObject(true);
        m_geom->setDataVariance(osg::Object::DYNAMIC);

        m_vertices = new osg::Vec3Array;
        m_colors   = new osg::Vec4Array;

        m_geom->setVertexArray(m_vertices);
        m_geom->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);
        m_geom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, 0));

        auto* ss = m_geom->getOrCreateStateSet();
        m_pointSizeUniform = applyPointCloudShader(ss, m_pointSize);

        // Look up z-clip uniforms (added by applyPointCloudShader)
        m_zClipUniform  = ss->getUniform("z_clipping");
        m_zRangeUniform = ss->getUniform("z_range");

        m_blendColor = new osg::BlendColor(osg::Vec4(1, 1, 1, m_opacity));
        ss->setAttributeAndModes(m_blendColor, osg::StateAttribute::ON);
        ss->setAttributeAndModes(
            new osg::BlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA),
            osg::StateAttribute::ON);

        m_geode = new osg::Geode;
        m_geode->addDrawable(m_geom);
    }

    /// Append one keyframe's point cloud, transformed to world space.
    void appendCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                     const Eigen::Isometry3d& pose,
                     long vertexId) {
        if (!cloud || cloud->empty()) return;

        size_t start = m_allWorldPoints.size();

        for (const auto& pt : cloud->points) {
            Eigen::Vector3d wp = pose * Eigen::Vector3d(pt.x, pt.y, pt.z);
            float wz = static_cast<float>(wp.z());
            if (wz < m_zMin) m_zMin = wz;
            if (wz > m_zMax) m_zMax = wz;
            m_allWorldPoints.push_back(wp);
        }

        size_t count = m_allWorldPoints.size() - start;
        m_cloudRanges.push_back({start, count, vertexId});
    }

    /// Upload vertex data and apply initial turbo colouring.
    void finish() {
        if (m_allWorldPoints.empty()) return;

        if (m_zMax - m_zMin < 0.001f) {
            m_zMin -= 0.5f;
            m_zMax += 0.5f;
        }

        // Initialize color range from data (only if auto mode)
        if (m_useAutoColorRange) {
            m_colorZMin = m_zMin;
            m_colorZMax = m_zMax;
        }

        // Initialize clip range from data on first load
        if (!m_clipRangeInitialized) {
            m_zClipMin = m_zMin;
            m_zClipMax = m_zMax;
            m_clipRangeInitialized = true;
        }

        // Push z-clip range to GPU (always, so shader has valid values)
        if (m_zRangeUniform)
            m_zRangeUniform->set(osg::Vec2(m_zClipMin, m_zClipMax));

        m_vertices->reserve(m_allWorldPoints.size());
        m_colors->reserve(m_allWorldPoints.size());

        for (const auto& wp : m_allWorldPoints) {
            m_vertices->push_back(osg::Vec3(
                static_cast<float>(wp.x()),
                static_cast<float>(wp.y()),
                static_cast<float>(wp.z())));
            m_colors->push_back(
                turboColor(static_cast<float>(wp.z()), m_colorZMin, m_colorZMax));
        }

        m_vertices->dirty();
        m_colors->dirty();

        auto* prim = static_cast<osg::DrawArrays*>(m_geom->getPrimitiveSet(0));
        if (prim) prim->setCount(m_vertices->size());
        m_geom->dirtyBound();
    }

    /// Re-colour one keyframe's cloud for selection highlight without
    /// rebuilding vertices.  Call after finish().
    void recolorHighlight(long selectedVertexId) {
        if (!m_colors || m_cloudRanges.empty()) return;

        const osg::Vec4 orange(1.0f, 0.55f, 0.0f, 1.0f);

        for (const auto& range : m_cloudRanges) {
            if (range.vertexId == selectedVertexId) {
                for (size_t i = range.startVertex; i < range.startVertex + range.vertexCount; ++i) {
                    (*m_colors)[i] = orange;
                }
            } else {
                // Restore turbo colour from world-space Z
                for (size_t i = range.startVertex; i < range.startVertex + range.vertexCount; ++i) {
                    float wz = static_cast<float>(m_allWorldPoints[i].z());
                    (*m_colors)[i] = turboColor(wz, m_colorZMin, m_colorZMax);
                }
            }
        }
        m_colors->dirty();
    }

    /// Clear all data for a full rebuild.
    void clear() {
        m_allWorldPoints.clear();
        m_cloudRanges.clear();
        m_vertices->clear();
        m_colors->clear();
        m_zMin =  std::numeric_limits<float>::max();
        m_zMax = -std::numeric_limits<float>::max();
        m_useAutoColorRange = true;
        m_clipRangeInitialized = false;
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

    // ---- Z-clip controls (shader-based, no VBO rebuild) ----

    void setZClipping(bool enabled) {
        m_zClipping = enabled;
        if (m_zClipUniform)
            m_zClipUniform->set(enabled ? 1 : 0);
    }

    void setZClipRange(float minZ, float maxZ) {
        m_zClipMin = minZ;
        m_zClipMax = maxZ;
        if (m_zRangeUniform)
            m_zRangeUniform->set(osg::Vec2(minZ, maxZ));
    }

    bool isZClipping() const { return m_zClipping; }
    float getZClipMin() const { return m_zClipMin; }
    float getZClipMax() const { return m_zClipMax; }

    // ---- Elevation color range controls (CPU recolor, no vertex rebuild) ----

    void setColorZRange(float minZ, float maxZ) {
        m_colorZMin = minZ;
        m_colorZMax = maxZ;
        m_useAutoColorRange = false;
        recolorAll();
    }

    void setAutoColorRange(bool autoRange) {
        m_useAutoColorRange = autoRange;
        if (autoRange) {
            m_colorZMin = m_zMin;
            m_colorZMax = m_zMax;
        }
        recolorAll();
    }

    bool isAutoColorRange() const { return m_useAutoColorRange; }

    // ---- Data range accessors (for UI initialization) ----

    float getDataZMin() const { return m_zMin; }
    float getDataZMax() const { return m_zMax; }
    float getColorZMin() const { return m_colorZMin; }
    float getColorZMax() const { return m_colorZMax; }

    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

    int pointCount() const {
        auto* prim = static_cast<osg::DrawArrays*>(m_geom->getPrimitiveSet(0));
        return prim ? prim->getCount() : 0;
    }

private:
    struct CloudRange {
        size_t startVertex;
        size_t vertexCount;
        long   vertexId;
    };

    /// Recompute all vertex colors from current color Z range.
    /// Does NOT rebuild vertices — only updates the color array.
    void recolorAll() {
        if (!m_colors || m_allWorldPoints.empty()) return;
        if (m_colorZMax - m_colorZMin < 0.001f) return;

        for (size_t i = 0; i < m_allWorldPoints.size(); ++i) {
            float wz = static_cast<float>(m_allWorldPoints[i].z());
            (*m_colors)[i] = turboColor(wz, m_colorZMin, m_colorZMax);
        }
        m_colors->dirty();
    }

    osg::ref_ptr<osg::Geode>     m_geode;
    osg::ref_ptr<osg::Geometry>  m_geom;
    osg::ref_ptr<osg::Vec3Array> m_vertices;
    osg::ref_ptr<osg::Vec4Array> m_colors;
    osg::ref_ptr<osg::BlendColor> m_blendColor;
    osg::ref_ptr<osg::Uniform>    m_pointSizeUniform;
    osg::ref_ptr<osg::Uniform>    m_zClipUniform;
    osg::ref_ptr<osg::Uniform>    m_zRangeUniform;

    // World-space points (kept for recolouring on selection change)
    std::vector<Eigen::Vector3d,
                Eigen::aligned_allocator<Eigen::Vector3d>> m_allWorldPoints;

    // Per-keyframe cloud metadata for selective recolouring
    std::vector<CloudRange> m_cloudRanges;

    float m_zMin   =  std::numeric_limits<float>::max();
    float m_zMax   = -std::numeric_limits<float>::max();
    float m_pointSize = 3.0f;
    float m_opacity   = 1.0f;

    // Z-clip state (shader-based, no rebuild needed on change)
    bool  m_zClipping = false;
    float m_zClipMin  = -10.0f;
    float m_zClipMax  = 10.0f;
    bool  m_clipRangeInitialized = false;

    // Elevation color range (CPU recolor on change)
    float m_colorZMin = 0.0f;
    float m_colorZMax = 1.0f;
    bool  m_useAutoColorRange = true;
};
