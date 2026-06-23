#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/MatrixTransform>
#include <osg/StateSet>
#include <osg/Uniform>
#include <osg/BlendFunc>
#include <osg/BlendColor>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Geometry>

#include <unordered_map>

#include "visualizers/TurboColormap.h"
#include "visualizers/CoreShaders.h"

/// @brief Builds per-keyframe point-cloud geometry in LOCAL SPACE under
///        osg::MatrixTransform nodes.  After g2o optimization only the
///        transform matrices are updated — vertex data stays on the GPU.
///
///        Tracks per-keyframe geometry so the selected keyframe's cloud
///        can be highlighted without rebuilding anything.
class KeyframePointCloudVisualizer {
public:
    KeyframePointCloudVisualizer() {
        m_cloudGroup = new osg::Group;
        m_cloudGroup->setName("PointClouds");

        // Shared StateSet on the parent group — children inherit shader,
        // blend mode, and point-size uniform.
        auto* ss = m_cloudGroup->getOrCreateStateSet();
        m_pointSizeUniform = applyPointCloudShader(ss, m_pointSize);

        m_blendColor = new osg::BlendColor(osg::Vec4(1, 1, 1, m_opacity));
        ss->setAttributeAndModes(m_blendColor, osg::StateAttribute::ON);
        ss->setAttributeAndModes(
            new osg::BlendFunc(GL_CONSTANT_ALPHA, GL_ONE_MINUS_CONSTANT_ALPHA),
            osg::StateAttribute::ON);
    }

    /// Add one keyframe's point cloud in LOCAL space, wrapped in a
    /// MatrixTransform initialised with the supplied pose.
    void addKeyframeCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                          const Eigen::Isometry3d& pose,
                          long vertexId) {
        if (!cloud || cloud->empty()) return;

        auto* xform = new osg::MatrixTransform;
        xform->setMatrix(eigenToOsg(pose));

        // Local-space geometry
        osg::ref_ptr<osg::Geometry> geom = new osg::Geometry;
        geom->setUseDisplayList(false);
        geom->setUseVertexBufferObjects(true);
        geom->setUseVertexArrayObject(true);
        geom->setDataVariance(osg::Object::DYNAMIC);

        auto* verts = new osg::Vec3Array;
        auto* colors = new osg::Vec4Array;
        verts->reserve(cloud->size());
        colors->reserve(cloud->size());

        float zMin =  std::numeric_limits<float>::max();
        float zMax = -std::numeric_limits<float>::max();

        for (const auto& pt : cloud->points) {
            float z = pt.z;
            if (z < zMin) zMin = z;
            if (z > zMax) zMax = z;
            verts->push_back(osg::Vec3(pt.x, pt.y, pt.z));
        }

        if (zMax - zMin < 0.001f) {
            zMin -= 0.5f;
            zMax += 0.5f;
        }

        // Colour by local Z (turbo), same as original world-space scheme
        for (const auto& pt : cloud->points) {
            colors->push_back(turboColor(pt.z, zMin, zMax));
        }

        // Store the raw local Z range for recolouring on selection
        m_keyframeZRanges[vertexId] = { zMin, zMax };

        geom->setVertexArray(verts);
        geom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
        geom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, verts->size()));

        auto* geode = new osg::Geode;
        geode->addDrawable(geom);
        xform->addChild(geode);
        m_cloudGroup->addChild(xform);

        m_transforms[vertexId] = xform;
        m_geometries[vertexId] = geom;
        m_colorArrays[vertexId] = colors;
    }

    /// Call after all addKeyframeCloud() calls (currently a no-op, kept for
    /// API compatibility).
    void finish() {}

    /// Update the MatrixTransform for a single keyframe from its new pose.
    /// Cheap — no vertex uploads, just a matrix update.
    void updateTransform(long vertexId, const Eigen::Isometry3d& pose) {
        auto it = m_transforms.find(vertexId);
        if (it != m_transforms.end()) {
            it->second->setMatrix(eigenToOsg(pose));
        }
    }

    /// Re-colour one keyframe's cloud for selection highlight without
    /// rebuilding vertices.
    void recolorHighlight(long selectedVertexId) {
        const osg::Vec4 orange(1.0f, 0.55f, 0.0f, 1.0f);

        for (auto& [id, colors] : m_colorArrays) {
            if (id == selectedVertexId) {
                for (size_t i = 0; i < colors->size(); ++i) {
                    (*colors)[i] = orange;
                }
            } else {
                // Restore turbo colour from local Z
                auto zRangeIt = m_keyframeZRanges.find(id);
                if (zRangeIt == m_keyframeZRanges.end()) continue;
                float zMin = zRangeIt->second.first;
                float zMax = zRangeIt->second.second;
                auto* verts = dynamic_cast<osg::Vec3Array*>(
                    m_geometries[id]->getVertexArray());
                if (!verts) continue;
                for (size_t i = 0; i < colors->size() && i < verts->size(); ++i) {
                    (*colors)[i] = turboColor((*verts)[i].z(), zMin, zMax);
                }
            }
            colors->dirty();
        }
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

    osg::ref_ptr<osg::Group> getNode() const { return m_cloudGroup; }

    int pointCount() const {
        int total = 0;
        for (auto& [id, geom] : m_geometries) {
            auto* prim = static_cast<osg::DrawArrays*>(geom->getPrimitiveSet(0));
            if (prim) total += prim->getCount();
        }
        return total;
    }

private:
    static osg::Matrixd eigenToOsg(const Eigen::Isometry3d& pose) {
        Eigen::Matrix4d m = pose.matrix();
        osg::Matrixd mat;
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c)
                mat(r, c) = m(r, c);
        return mat;
    }

    osg::ref_ptr<osg::Group>          m_cloudGroup;
    osg::ref_ptr<osg::BlendColor>     m_blendColor;
    osg::ref_ptr<osg::Uniform>        m_pointSizeUniform;

    /// Per-keyframe data for fast pose updates and recolouring
    std::unordered_map<long, osg::MatrixTransform*>     m_transforms;
    std::unordered_map<long, osg::ref_ptr<osg::Geometry>> m_geometries;
    std::unordered_map<long, osg::ref_ptr<osg::Vec4Array>> m_colorArrays;

    /// Local-space Z range per keyframe (for turbo recolouring)
    std::unordered_map<long, std::pair<float, float>>   m_keyframeZRanges;

    float m_pointSize = 3.0f;
    float m_opacity   = 1.0f;
};
