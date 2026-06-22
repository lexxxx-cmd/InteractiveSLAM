#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/Vec3>
#include <osg/Vec4>
#include <cmath>

#include "visualizers/CoreShaders.h"

/// @brief Builds sphere geometry vertices in WORLD SPACE (no MatrixTransform
///        needed).  Call appendSphere() for each position, then finish() to
///        seal the geometry.  Same pattern as EdgeLineVisualizer — raw
///        world-space vertices, no scene-graph transform tricks.
class VertexSphereVisualizer {
public:
    VertexSphereVisualizer(float radius = 1.0f,
                           int rings = 8,
                           int sectors = 8)
        : m_radius(radius), m_rings(rings), m_sectors(sectors) {

        m_geom = new osg::Geometry;
        m_geom->setUseDisplayList(false);
        m_geom->setUseVertexBufferObjects(true);
        m_geom->setUseVertexArrayObject(true);
        m_geom->setDataVariance(osg::Object::DYNAMIC);

        m_verts  = new osg::Vec3Array;
        m_colors = new osg::Vec4Array;
        m_indices = new osg::DrawElementsUInt(GL_TRIANGLES);

        m_geom->setVertexArray(m_verts);
        m_geom->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);
        m_geom->addPrimitiveSet(m_indices);

        applySimpleColorShader(m_geom->getOrCreateStateSet());

        m_geode = new osg::Geode;
        m_geode->addDrawable(m_geom);
    }

    /// Append one sphere centred at @p center (world-space).
    void appendSphere(const osg::Vec3d& center,
                      const osg::Vec4& color = osg::Vec4(1.0f, 0.0f, 0.0f, 1.0f)) {
        const float pi = 3.14159265f;
        unsigned int base = m_verts->size();

        // Generate Y-up sphere vertices (same tessellation as glk::buildSphere)
        for (int r = 0; r <= m_rings; ++r) {
            float phi    = float(r) * pi / float(m_rings);
            float sinPhi = std::sin(phi);
            float cosPhi = std::cos(phi);

            for (int s = 0; s <= m_sectors; ++s) {
                float theta = float(s) * 2.0f * pi / float(m_sectors);
                float sinT  = std::sin(theta);
                float cosT  = std::cos(theta);

                m_verts->push_back(osg::Vec3(
                    center.x() + m_radius * sinPhi * cosT,
                    center.y() + m_radius * cosPhi,
                    center.z() + m_radius * sinPhi * sinT));
                m_colors->push_back(color);
            }
        }

        // Index buffer — two triangles per quad
        for (int r = 0; r < m_rings; ++r) {
            for (int s = 0; s < m_sectors; ++s) {
                unsigned int a = base + r * (m_sectors + 1) + s;
                unsigned int b = a + m_sectors + 1;
                m_indices->push_back(a);
                m_indices->push_back(b);
                m_indices->push_back(a + 1);
                m_indices->push_back(b);
                m_indices->push_back(b + 1);
                m_indices->push_back(a + 1);
            }
        }
    }

    void setRadius(float r) { m_radius = r; }
    float radius() const { return m_radius; }

    /// Call after all spheres have been appended.
    void finish() {
        m_verts->dirty();
        m_colors->dirty();
        m_indices->dirty();
        m_geom->dirtyBound();
    }

    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

    /// Clear all data (for rebuild).
    void clear() {
        m_verts->clear();
        m_colors->clear();
        m_indices->clear();
    }

private:
    float m_radius;
    int m_rings;
    int m_sectors;

    osg::ref_ptr<osg::Geode> m_geode;
    osg::ref_ptr<osg::Geometry> m_geom;
    osg::ref_ptr<osg::Vec3Array> m_verts;
    osg::ref_ptr<osg::Vec4Array> m_colors;
    osg::ref_ptr<osg::DrawElementsUInt> m_indices;
};
