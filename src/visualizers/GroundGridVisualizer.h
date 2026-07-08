#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/Program>
#include <osg/Uniform>
#include <osg/BlendFunc>
#include <osg/Depth>

/// @brief Infinite ground grid rendered via GLSL shader.
///        Major/minor grid lines, X/Y axis highlighting, distance fade.
///        Adapted from 3DPCViewer's GridVisualizer.
class GroundGridVisualizer {
public:
    explicit GroundGridVisualizer(float size = 1000.0f) {
        auto* gridGeode = new osg::Geode;

        auto* quad = new osg::Geometry;
        auto* vertices = new osg::Vec3Array;

        float s = size;
        vertices->push_back(osg::Vec3(-s, -s, 0.0f));
        vertices->push_back(osg::Vec3( s, -s, 0.0f));
        vertices->push_back(osg::Vec3(-s,  s, 0.0f));
        vertices->push_back(osg::Vec3( s, -s, 0.0f));
        vertices->push_back(osg::Vec3( s,  s, 0.0f));
        vertices->push_back(osg::Vec3(-s,  s, 0.0f));

        quad->setVertexArray(vertices);
        quad->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, 6));

        auto* program = new osg::Program;
        program->addShader(new osg::Shader(osg::Shader::VERTEX, kVertSource));
        program->addShader(new osg::Shader(osg::Shader::FRAGMENT, kFragSource));

        auto* ss = quad->getOrCreateStateSet();
        ss->setAttributeAndModes(program, osg::StateAttribute::ON);
        ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));
        ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);

        ss->setRenderBinDetails(10, "DepthSortedBin");
        auto* depth = new osg::Depth;
        depth->setWriteMask(false);
        ss->setAttributeAndModes(depth, osg::StateAttribute::ON);

        ss->addUniform(new osg::Uniform("gridSpacing", 1.0f));
        ss->addUniform(new osg::Uniform("lineWidth", 1.0f));
        ss->addUniform(new osg::Uniform("majorGridStep", 10.0f));
        ss->addUniform(new osg::Uniform("gridColor", osg::Vec4(0.5f, 0.5f, 0.5f, 0.6f)));
        ss->addUniform(new osg::Uniform("axisXColor", osg::Vec4(0.8f, 0.1f, 0.1f, 1.0f)));
        ss->addUniform(new osg::Uniform("axisYColor", osg::Vec4(0.1f, 0.8f, 0.1f, 1.0f)));
        ss->addUniform(new osg::Uniform("fadeRadius", size * 0.8f));

        gridGeode->addDrawable(quad);
        m_geode = gridGeode;
    }

    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

private:
    osg::ref_ptr<osg::Geode> m_geode;

    static constexpr const char* kVertSource = R"(
        #version 330 core
        in vec4 osg_Vertex;
        uniform mat4 osg_ModelViewProjectionMatrix;
        out vec3 vWorldPos;
        void main() {
            vWorldPos = osg_Vertex.xyz;
            gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;
        }
    )";

    static constexpr const char* kFragSource = R"(
        #version 330 core
        in vec3 vWorldPos;
        uniform float gridSpacing;
        uniform float lineWidth;
        uniform float majorGridStep;
        uniform vec4 gridColor;
        uniform vec4 axisXColor;
        uniform vec4 axisYColor;
        uniform float fadeRadius;
        out vec4 fragColor;

        float getGrid(float pos, float spacing) {
            float coord = pos / spacing;
            float grid = abs(fract(coord - 0.5) - 0.5) / fwidth(coord);
            return 1.0 - smoothstep(0.0, lineWidth, grid);
        }

        void main() {
            float x = vWorldPos.x;
            float y = vWorldPos.y;

            float lineX = getGrid(x, gridSpacing);
            float lineY = getGrid(y, gridSpacing);
            float majorX = getGrid(x, gridSpacing * majorGridStep);
            float majorY = getGrid(y, gridSpacing * majorGridStep);

            float gridAlpha = max(lineX, lineY);
            float majorAlpha = max(majorX, majorY);

            vec4 finalColor = gridColor;
            if (majorAlpha > 0.1) {
                finalColor.rgb *= 1.5;
                gridAlpha = max(gridAlpha, majorAlpha);
            }

            float axisX = 1.0 - smoothstep(0.0, 0.05, abs(y));
            float axisY = 1.0 - smoothstep(0.0, 0.05, abs(x));

            if (axisX > 0.1) finalColor = mix(finalColor, axisXColor, axisX);
            if (axisY > 0.1) finalColor = mix(finalColor, axisYColor, axisY);

            float dist = length(vWorldPos.xy);
            float fade = 1.0 - smoothstep(fadeRadius * 0.5, fadeRadius, dist);

            float alpha = gridAlpha * finalColor.a * fade;
            if (alpha < 0.01) discard;

            fragColor = vec4(finalColor.rgb, alpha);
        }
    )";
};
