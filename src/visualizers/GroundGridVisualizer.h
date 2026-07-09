// ============================================================================
// GroundGridVisualizer.h
// 地面网格可视化器
//
// 功能：通过 GLSL 着色器渲染无限延伸的地面网格。
//       包括次要网格线、主要网格线（更粗更亮）、X/Y 轴高亮和距离淡出效果。
//       改编自 3DPCViewer 的 GridVisualizer。
// ============================================================================

#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/Program>
#include <osg/Uniform>
#include <osg/BlendFunc>
#include <osg/Depth>

/**
 * @brief 地面网格可视化器
 *
 * 在场景中渲染一个 Z=0 平面的无限网格，提供空间参考。
 * 使用 GLSL 着色器在片段着色器中动态绘制网格线，而非使用大量线段几何体。
 *
 * 网格特性：
 *   - 次要网格线：默认间距 1 单位，灰色半透明
 *   - 主要网格线：每 10 个次要网格线出现一次（更亮更粗）
 *   - X 轴高亮：沿 Y=0 线显示红色
 *   - Y 轴高亮：沿 X=0 线显示绿色
 *   - 距离淡出：远离原点时网格逐渐透明
 *
 * 渲染优化：仅使用一个包含 2 个三角形的四边形几何体，
 * 所有网格效果均在片段着色器中计算，无额外几何体开销。
 */
class GroundGridVisualizer {
public:
    /**
     * @brief 构造函数
     * @param size 网格半宽（总宽 2*size），默认 1000.0 单位
     */
    explicit GroundGridVisualizer(float size = 1000.0f) {
        auto* gridGeode = new osg::Geode;

        // 创建一个 Z=0 平面上的大四边形，由 2 个三角形组成
        auto* quad = new osg::Geometry;
        auto* vertices = new osg::Vec3Array;

        float s = size;
        // 第一个三角形：(-s,-s,0), (s,-s,0), (-s,s,0)
        vertices->push_back(osg::Vec3(-s, -s, 0.0f));
        vertices->push_back(osg::Vec3( s, -s, 0.0f));
        vertices->push_back(osg::Vec3(-s,  s, 0.0f));
        // 第二个三角形：(s,-s,0), (s,s,0), (-s,s,0)
        vertices->push_back(osg::Vec3( s, -s, 0.0f));
        vertices->push_back(osg::Vec3( s,  s, 0.0f));
        vertices->push_back(osg::Vec3(-s,  s, 0.0f));

        quad->setVertexArray(vertices);
        quad->addPrimitiveSet(new osg::DrawArrays(osg::PrimitiveSet::TRIANGLES, 0, 6));

        // 设置 GLSL 330 Core Profile 着色器
        auto* program = new osg::Program;
        program->addShader(new osg::Shader(osg::Shader::VERTEX, kVertSource));
        program->addShader(new osg::Shader(osg::Shader::FRAGMENT, kFragSource));

        // 配置状态集
        auto* ss = quad->getOrCreateStateSet();
        ss->setAttributeAndModes(program, osg::StateAttribute::ON);
        ss->setAttributeAndModes(new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA));  // 透明混合
        ss->setMode(GL_LIGHTING, osg::StateAttribute::OFF);  // 关闭光照

        // 深度排序渲染，用于透明效果正确排序
        ss->setRenderBinDetails(10, "DepthSortedBin");
        auto* depth = new osg::Depth;
        depth->setWriteMask(false);  // 不写入深度缓冲，允许透明叠加
        ss->setAttributeAndModes(depth, osg::StateAttribute::ON);

        // 添加着色器 uniform 参数
        ss->addUniform(new osg::Uniform("gridSpacing", 1.0f));         // 网格间距
        ss->addUniform(new osg::Uniform("lineWidth", 1.0f));           // 网格线宽
        ss->addUniform(new osg::Uniform("majorGridStep", 10.0f));      // 主要网格线的步长倍数
        ss->addUniform(new osg::Uniform("gridColor", osg::Vec4(0.5f, 0.5f, 0.5f, 0.6f)));  // 网格颜色（灰色半透明）
        ss->addUniform(new osg::Uniform("axisXColor", osg::Vec4(0.8f, 0.1f, 0.1f, 1.0f))); // X 轴颜色（红色）
        ss->addUniform(new osg::Uniform("axisYColor", osg::Vec4(0.1f, 0.8f, 0.1f, 1.0f))); // Y 轴颜色（绿色）
        ss->addUniform(new osg::Uniform("fadeRadius", size * 0.8f));   // 淡出半径

        gridGeode->addDrawable(quad);
        m_geode = gridGeode;
    }

    /** @brief 获取地面网格的 OSG 节点 */
    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

private:
    osg::ref_ptr<osg::Geode> m_geode;  ///< 网格所在的叶节点

    // —— 顶点着色器 ——
    // 将世界坐标传递给片段着色器（用于网格线计算和淡出）
    static constexpr const char* kVertSource = R"(
        #version 330 core
        in vec4 osg_Vertex;
        uniform mat4 osg_ModelViewProjectionMatrix;
        out vec3 vWorldPos;          // 传递世界坐标到片段着色器
        void main() {
            vWorldPos = osg_Vertex.xyz;
            gl_Position = osg_ModelViewProjectionMatrix * osg_Vertex;
        }
    )";

    // —— 片段着色器 ——
    // 动态计算网格线、轴高亮和距离淡出效果
    static constexpr const char* kFragSource = R"(
        #version 330 core
        in vec3 vWorldPos;
        uniform float gridSpacing;       // 网格间距
        uniform float lineWidth;         // 网格线宽
        uniform float majorGridStep;     // 主要网格线的步长倍数
        uniform vec4 gridColor;          // 网格颜色
        uniform vec4 axisXColor;         // X 轴颜色（Y=0 线）
        uniform vec4 axisYColor;         // Y 轴颜色（X=0 线）
        uniform float fadeRadius;        // 淡出半径
        out vec4 fragColor;

        // 计算给定位置上网格线的强度
        float getGrid(float pos, float spacing) {
            float coord = pos / spacing;
            // 使用 fract 计算到最近网格线的距离，fwidth 进行抗锯齿
            float grid = abs(fract(coord - 0.5) - 0.5) / fwidth(coord);
            return 1.0 - smoothstep(0.0, lineWidth, grid);
        }

        void main() {
            float x = vWorldPos.x;
            float y = vWorldPos.y;

            // 分别计算 X 方向和 Y 方向的次要网格线
            float lineX = getGrid(x, gridSpacing);
            float lineY = getGrid(y, gridSpacing);
            // 计算主要网格线
            float majorX = getGrid(x, gridSpacing * majorGridStep);
            float majorY = getGrid(y, gridSpacing * majorGridStep);

            float gridAlpha = max(lineX, lineY);
            float majorAlpha = max(majorX, majorY);

            vec4 finalColor = gridColor;
            // 主要网格线更亮
            if (majorAlpha > 0.1) {
                finalColor.rgb *= 1.5;
                gridAlpha = max(gridAlpha, majorAlpha);
            }

            // X 轴（Y=0）和 Y 轴（X=0）高亮
            float axisX = 1.0 - smoothstep(0.0, 0.05, abs(y));
            float axisY = 1.0 - smoothstep(0.0, 0.05, abs(x));

            if (axisX > 0.1) finalColor = mix(finalColor, axisXColor, axisX);
            if (axisY > 0.1) finalColor = mix(finalColor, axisYColor, axisY);

            // 距离淡出（远离原点时逐渐透明）
            float dist = length(vWorldPos.xy);
            float fade = 1.0 - smoothstep(fadeRadius * 0.5, fadeRadius, dist);

            float alpha = gridAlpha * finalColor.a * fade;
            if (alpha < 0.01) discard;

            fragColor = vec4(finalColor.rgb, alpha);
        }
    )";
};
