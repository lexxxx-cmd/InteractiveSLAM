#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include "visualizers/CoreShaders.h"

/// @brief Simple RGB coordinate axes at world origin.
///        Uses GLSL 330 shader (not OSG fixed-function) for Core Profile compat.
class CoordinateAxesVisualizer {
public:
    CoordinateAxesVisualizer() {
        auto* geom = new osg::Geometry;
        geom->setUseDisplayList(false);
        geom->setUseVertexBufferObjects(true);
        geom->setUseVertexArrayObject(true);   // mandatory in Core Profile 3.3

        auto* verts = new osg::Vec3Array;
        auto* colors = new osg::Vec4Array;

        verts->push_back(osg::Vec3(0, 0, 0));
        verts->push_back(osg::Vec3(1, 0, 0));
        colors->push_back(osg::Vec4(1, 0, 0, 1));
        colors->push_back(osg::Vec4(1, 0, 0, 1));

        verts->push_back(osg::Vec3(0, 0, 0));
        verts->push_back(osg::Vec3(0, 1, 0));
        colors->push_back(osg::Vec4(0, 1, 0, 1));
        colors->push_back(osg::Vec4(0, 1, 0, 1));

        verts->push_back(osg::Vec3(0, 0, 0));
        verts->push_back(osg::Vec3(0, 0, 1));
        colors->push_back(osg::Vec4(0, 0, 1, 1));
        colors->push_back(osg::Vec4(0, 0, 1, 1));

        geom->setVertexArray(verts);
        geom->setColorArray(colors, osg::Array::BIND_PER_VERTEX);
        geom->addPrimitiveSet(new osg::DrawArrays(GL_LINES, 0, verts->size()));

        // Custom GLSL 330 shader — prevents OSG from applying deprecated
        // Material/LightModel/AlphaFunc uniforms.
        applySimpleColorShader(geom->getOrCreateStateSet());

        m_geode = new osg::Geode;
        m_geode->addDrawable(geom);
    }

    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

private:
    osg::ref_ptr<osg::Geode> m_geode;
};
