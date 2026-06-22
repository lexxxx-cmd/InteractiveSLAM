#pragma once

#include <osg/Program>
#include <osg/Shader>
#include <osg/Uniform>
#include <osg/StateSet>

/// @file
/// GLSL 330 Core Profile shader programs that replace OSG's built-in
/// fixed-function pipeline. Key principle: by setting an explicit Program
/// on every drawable, OSG won't try to set its deprecated internal uniforms
/// (Material, LightModel, AlphaFunc), eliminating GL errors in Core Profile.

// ---------------------------------------------------------------------------
// Simple color-only program — for lines, axes, spheres
// ---------------------------------------------------------------------------
inline osg::Program* createSimpleColorProgram() {
    static osg::ref_ptr<osg::Program> s_prog;
    if (s_prog.valid()) return s_prog.get();

    const char* vert = R"(
        #version 330 core
        in vec4 osg_Vertex;
        in vec4 osg_Color;
        uniform mat4 osg_ModelViewProjectionMatrix;
        out vec4 vColor;
        void main() {
            gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;
            vColor = osg_Color;
        }
    )";
    const char* frag = R"(
        #version 330 core
        in vec4 vColor;
        out vec4 fragColor;
        void main() {
            fragColor = vColor;
        }
    )";
    s_prog = new osg::Program;
    s_prog->addShader(new osg::Shader(osg::Shader::VERTEX, vert));
    s_prog->addShader(new osg::Shader(osg::Shader::FRAGMENT, frag));
    return s_prog.get();
}

// ---------------------------------------------------------------------------
// Point-cloud program — per-vertex color + controllable point size via uniform
// ---------------------------------------------------------------------------
inline osg::Program* createPointCloudProgram() {
    static osg::ref_ptr<osg::Program> s_prog;
    if (s_prog.valid()) return s_prog.get();

    const char* vert = R"(
        #version 330 core
        in vec4 osg_Vertex;
        in vec4 osg_Color;
        uniform mat4 osg_ModelViewProjectionMatrix;
        uniform float uPointSize;
        out vec4 vColor;
        void main() {
            gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;
            gl_PointSize = uPointSize;
            vColor = osg_Color;
        }
    )";
    const char* frag = R"(
        #version 330 core
        in vec4 vColor;
        out vec4 fragColor;
        void main() {
            fragColor = vColor;
        }
    )";
    s_prog = new osg::Program;
    s_prog->addShader(new osg::Shader(osg::Shader::VERTEX, vert));
    s_prog->addShader(new osg::Shader(osg::Shader::FRAGMENT, frag));
    return s_prog.get();
}

/// Attach the simple color shader to a StateSet.
inline void applySimpleColorShader(osg::StateSet* ss) {
    ss->setAttributeAndModes(createSimpleColorProgram(),
                             osg::StateAttribute::ON);
}

/// Attach the point-cloud shader to a StateSet and create the point-size uniform.
/// Returns the uniform so the caller can update it at runtime.
inline osg::Uniform* applyPointCloudShader(osg::StateSet* ss, float pointSize = 3.0f) {
    ss->setAttributeAndModes(createPointCloudProgram(),
                             osg::StateAttribute::ON);
    auto* u = new osg::Uniform("uPointSize", pointSize);
    ss->addUniform(u);
    return u;
}
