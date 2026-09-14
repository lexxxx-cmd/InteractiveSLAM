// ============================================================================
// VertexSphereVisualizer.h
// 顶点位姿标记可视化器（保留历史类名）
//
// 功能：在世界坐标系中构建顶点位姿标记（无需 MatrixTransform）。
//       调用 appendFrustum() 为每个顶点添加相机视锥体标记，
//       然后调用 finish() 完成构建。
//       与 EdgeLineVisualizer 使用相同的模式——原始世界坐标系顶点，
//       不使用场景图变换技巧。
//
// 标记形状（相机视锥体，frustum）：
//   - 所有标记统一为相机视锥体造型：锥顶位于关键帧位姿（相机光心），
//     沿局部 +Z（前方）张开，末端为 16:9 矩形"投影屏"。
//   - 仅投影屏一个蓝色填充面（正反两面均可见），锥体轮廓由
//     锥顶四条棱 + 投影屏边框的同色系提亮线框勾出，轻盈不遮挡。
//   - 纵向渐变标明方向：按标记自身局部右-下-前坐标系的"下"方向（+Y），
//     上面（图像上方）颜色较深、下面颜色较浅，随关键帧姿态一起旋转。
//   - 配色由调用方决定：普通顶点蓝色系，高亮顶点红色系（并放大尺寸）。
//   - 调试：可在锥顶绘制局部 RGB 坐标轴（+X 右红 / +Y 下绿 / +Z 前蓝），
//     X-ray 显示（关闭深度测试），任何视角下方向指示始终可见，可开关。
//
// 方向由关键帧局部位姿（右-下-前坐标系，X右/Y下/Z前）的旋转矩阵决定，
// 生成时把局部坐标系中的偏移量旋转到世界坐标系并平移到顶点位置。
// ============================================================================

#pragma once

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/StateSet>
#include <osg/BlendFunc>
#include <osg/Vec3>
#include <osg/Vec4>
#include <algorithm>
#include <unordered_map>
#include <cstdint>

#include <Eigen/Geometry>

#include "visualizers/CoreShaders.h"

/**
 * @brief 顶点位姿标记可视化器
 *
 * 在场景中为每个 SLAM 图顶点绘制一个定向标记（相机视锥体），
 * 位置与方向均来自关键帧的局部位姿。所有标记合并到单个几何体中
 * 以提高渲染效率。
 *
 * 使用模式：
 *   1. clear() —— 清除旧数据
 *   2. appendFrustum() —— 逐个添加视锥体标记（指定位置、方向和颜色）
 *   3. finish() —— 完成构建并更新 GPU 缓冲区
 */
class VertexSphereVisualizer {
public:
    /**
     * @brief 单个标记在颜色数组中的分区范围
     *
     * 顶点按"投影屏 → 线框"连续排布，记录各段长度即可在
     * updateSphereColor()/setOpacity() 中按分区应用不同的
     * alpha 与线框提亮，避免遍历所有标记。
     * 定义于类前部：ranges() 的返回类型在声明处解析，
     * 需先于其出现。
     */
    struct SphereRange {
        unsigned int startIndex;  ///< 在 m_colors 中的起始索引
        int planeCount = 0;       ///< 投影屏顶点数（4）
        int lineCount  = 0;       ///< 线框顶点数（锥顶 + 四角 = 5）
        unsigned int vertexStartIndex = 0; ///< 在 m_verts 中的起始索引（标记顶点连续排布）
        osg::Vec3d apex;          ///< 锥顶世界坐标（updateSphereScale 的缩放原点）
        float scale = 1.0f;       ///< 当前缩放系数（updateSphereScale 增量缩放用）
        unsigned int axisStartIndex = 0; ///< 在 m_axesColors 中的起始索引
        int axisCount = 0;        ///< 局部坐标轴顶点数（6；未启用为 0）
    };

public:
    /**
     * @brief 构造函数
     *
     * @param radius 标记特征尺寸（默认 1.0；视锥体深度 = 2 × radius，
     *               投影屏 16:9：半宽 = radius、半高 = 0.5625 × radius，
     *               约 53° 水平视场角）
     */
    explicit VertexSphereVisualizer(float radius = 0.1f)
        : m_radius(radius) {

        m_geom = new osg::Geometry;
        m_geom->setUseDisplayList(false);
        m_geom->setUseVertexBufferObjects(true);
        m_geom->setUseVertexArrayObject(true);
        m_geom->setDataVariance(osg::Object::DYNAMIC);

        // 顶点、颜色和索引数组（三角形 = 投影屏，线段 = 线框）
        m_verts  = new osg::Vec3Array;
        m_colors = new osg::Vec4Array;
        m_indices     = new osg::DrawElementsUInt(GL_TRIANGLES);
        m_lineIndices = new osg::DrawElementsUInt(GL_LINES);

        m_geom->setVertexArray(m_verts);
        m_geom->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);
        m_geom->addPrimitiveSet(m_indices);
        m_geom->addPrimitiveSet(m_lineIndices);

        // 应用纯色着色器
        applySimpleColorShader(m_geom->getOrCreateStateSet());

        // 启用 alpha 混合（顶点透明度调整）：OSG 会把开启 GL_BLEND 的
        // StateSet 归入透明渲染队列，在不透明几何体之后绘制
        auto* ss = m_geom->getOrCreateStateSet();
        ss->setMode(GL_BLEND, osg::StateAttribute::ON);
        ss->setAttributeAndModes(
            new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON);
        // 不启用背面剔除：填充面只剩投影屏单一平面（无自重叠，
        // 不会出现正反面同像素双重混合），关闭剔除使其正反两面
        // 均可见——从锥顶方向看投影屏同样显示
        // （要求三角形仍按"从外部看逆时针"的环绕方向生成，保证法线一致）

        m_geode = new osg::Geode;
        m_geode->addDrawable(m_geom);

        // —— 局部坐标轴（调试用，独立几何体）：关闭深度测试做 X-ray
        // 显示，保证任何视角下方向指示始终可见，不与视锥体面片互相遮挡
        m_axesGeom = new osg::Geometry;
        m_axesGeom->setUseDisplayList(false);
        m_axesGeom->setUseVertexBufferObjects(true);
        m_axesGeom->setUseVertexArrayObject(true);
        m_axesGeom->setDataVariance(osg::Object::DYNAMIC);

        m_axesVerts   = new osg::Vec3Array;
        m_axesColors  = new osg::Vec4Array;
        m_axesIndices = new osg::DrawElementsUInt(GL_LINES);
        m_axesGeom->setVertexArray(m_axesVerts);
        m_axesGeom->setColorArray(m_axesColors, osg::Array::BIND_PER_VERTEX);
        m_axesGeom->addPrimitiveSet(m_axesIndices);

        auto* ass = m_axesGeom->getOrCreateStateSet();
        applySimpleColorShader(ass);
        ass->setMode(GL_BLEND, osg::StateAttribute::ON);
        ass->setAttributeAndModes(
            new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON);
        ass->setMode(GL_DEPTH_TEST, osg::StateAttribute::OFF);

        m_geode->addDrawable(m_axesGeom);
    }

    /**
     * @brief 在世界坐标系中添加一个相机视锥体标记
     *
     * 锥顶（相机光心）位于 @p center（即关键帧位姿平移处），沿局部
     * +Z（前方）展开：深度 = 2R，投影屏为 16:9 矩形（半宽 R、
     * 半高 0.5625R）。纵向渐变沿局部"下"方向（右-下-前坐标系的
     * +Y）：图像上方（-Y）颜色较深、下方（+Y）较浅，随关键帧姿态
     * 一起旋转，用于标明视锥体朝向。
     * 着色（均先乘全局不透明度）：
     *   - 投影屏：唯一的填充面，alpha × kPlaneAlpha（较实）
     *   - 线框：同色系提亮（RGB 各 × 0.5 + 0.5），alpha × kLineAlpha
     * 侧面不做填充，锥体轮廓完全由线框表达。
     *
     * 同时记录 vertexId 对应的颜色数组分区范围，供 updateSphereColor()
     * 后续增量更新颜色使用（避免全量几何体重建）。
     *
     * @param center   标记锥顶位置（世界坐标，关键帧位姿处）
     * @param rot      局部坐标系 → 世界坐标系的旋转（关键帧位姿的旋转部分）
     * @param color    标记颜色（RGBA，默认蓝色系）
     * @param vertexId 对应的顶点 ID（用于后续增量颜色更新，默认 -1 不追踪）
     * @param r        特征尺寸（< 0 时使用构造时设置的全局半径 m_radius）
     */
    void appendFrustum(const osg::Vec3d& center,
                       const Eigen::Matrix3f& rot,
                       const osg::Vec4& color = osg::Vec4(0.20f, 0.50f, 1.0f, 1.0f),
                       long vertexId = -1,
                       float r = -1.0f) {
        float R = (r >= 0.0f) ? r : m_radius;

        beginAppend(color, vertexId);

        float d  = kDepth * R;
        float hw = kHalfWidth * R;
        float hh = kHalfHeight * R;

        // ---- 顶点推入顺序固定：投影屏 → 线框 ----
        //（与 SphereRange 分区记录及渐变参数表一一对应，勿打乱）
        // t 为纵向渐变参数：0 = 图像上方（局部 -Y，深），1 = 下方（+Y，浅）

        // 投影屏四角（唯一的填充面，不启用背面剔除，正反两面均可见）
        int ptl = pushVertex(-hw, -hh, d, center, rot, m_planeBase, 0.0f, kPlaneAlpha);
        int ptr = pushVertex( hw, -hh, d, center, rot, m_planeBase, 0.0f, kPlaneAlpha);
        int pbr = pushVertex( hw,  hh, d, center, rot, m_planeBase, 1.0f, kPlaneAlpha);
        int pbl = pushVertex(-hw,  hh, d, center, rot, m_planeBase, 1.0f, kPlaneAlpha);

        // 线框角点（锥顶 + 投影屏四角，独立顶点）
        int lApex = pushVertex(0.0f,  0.0f, 0.0f, center, rot, m_lineBase, 0.5f, kLineAlpha);
        int lTl   = pushVertex(-hw, -hh,    d, center, rot, m_lineBase, 0.0f, kLineAlpha);
        int lTr   = pushVertex( hw, -hh,    d, center, rot, m_lineBase, 0.0f, kLineAlpha);
        int lBr   = pushVertex( hw,  hh,    d, center, rot, m_lineBase, 1.0f, kLineAlpha);
        int lBl   = pushVertex(-hw,  hh,    d, center, rot, m_lineBase, 1.0f, kLineAlpha);

        // 投影屏两个三角形（局部 X 右 / Y 下 / Z 前，外法线朝 +Z）
        m_indices->push_back(ptl); m_indices->push_back(ptr); m_indices->push_back(pbr);
        m_indices->push_back(ptl); m_indices->push_back(pbr); m_indices->push_back(pbl);

        // 线框：锥顶到四角的 4 条棱 + 投影屏矩形边框
        auto line = [this](int a, int b) {
            m_lineIndices->push_back(a);
            m_lineIndices->push_back(b);
        };
        line(lApex, lTl); line(lApex, lTr); line(lApex, lBr); line(lApex, lBl);
        line(lTl, lTr);   line(lTr, lBr);   line(lBr, lBl);   line(lBl, lTl);

        // 调试：锥顶局部 RGB 坐标轴（可选）
        if (m_drawLocalAxes) appendLocalAxes(center, rot, R, vertexId);

        // 实际烘焙进几何体的缩放（选中帧按 2R 构建时为 2.0）
        endAppend(vertexId, center,
                  (m_radius > 1e-6f) ? (R / m_radius) : 1.0f);
    }

    /**
     * @brief 按顶点 ID 更新单个标记的颜色（不重建几何体）
     *
     * 从 m_sphereRanges 中查找该顶点对应的颜色数组分区范围，仅更新该
     * 范围的颜色值，并保留标记的全部视觉结构："投影屏/线框"分区 alpha
     * 与"上深下浅"的纵向方向渐变；调试坐标轴保持固定 RGB 惯例色，
     * 不随高亮变色。与 clear() + appendFrustum() + finish() 的全量
     * 重建相比，此方法只需更新少量颜色值 + 一次 dirty()，O(1) 复杂度，
     * 与总关键帧数无关。
     *
     * @param vertexId 顶点 ID（需已在 appendFrustum 中添加过）
     * @param newColor 新颜色（RGBA）
     */
    void updateSphereColor(long vertexId, const osg::Vec4& newColor) {
        auto it = m_sphereRanges.find(vertexId);
        if (it == m_sphereRanges.end()) return;
        const auto& range = it->second;

        osg::Vec4 base = newColor;
        // 颜色更新不破坏整体不透明度；存在按标记 alpha 覆盖（聚焦淡化）
        // 时优先使用覆盖值
        base.a() = alphaFor(vertexId);

        // 按与 appendFrustum 相同的顶点顺序重放方向渐变
        unsigned int idx = range.startIndex;
        auto assign = [this, &idx](const osg::Vec4& layerBase, const float* ts,
                                   const float* mul, int count) {
            for (int i = 0; i < count; ++i)
                (*m_colors)[idx++] = shaded(layerBase, ts[i], mul[i]);
        };
        assign(base,            kPlaneGradT, kPlaneAlphaMul, range.planeCount);
        assign(lightened(base), kLineGradT,  kLineAlphaMul,  range.lineCount);
        m_colors->dirty();
    }

    /**
     * @brief 按顶点 ID 原地缩放单个标记（不重建几何体）
     *
     * 以锥顶（相机光心）为缩放原点，对该标记的全部顶点（投影屏 +
     * 线框）做等比缩放：newPos = apex + (oldPos - apex) × scale/旧scale。
     * 用于高亮尺寸变化的轻量路径（如播放高亮与选中态切换），
     * O(标记顶点数) 复杂度，与总关键帧数无关。
     *
     * 调试局部坐标轴（若启用）同步缩放。缩放后 dirtyBound()
     * 防止放大后的标记被包围球剔除。
     *
     * @param vertexId 顶点 ID（需已在 appendFrustum 中添加过）
     * @param scale    目标缩放系数（1.0 = 全局默认尺寸）
     */
    void updateSphereScale(long vertexId, float scale) {
        auto it = m_sphereRanges.find(vertexId);
        if (it == m_sphereRanges.end()) return;
        SphereRange& range = it->second;
        if (std::abs(range.scale - scale) < 1e-6f) return;

        float ratio = scale / range.scale;
        range.scale = scale;

        // 标记自身顶点：绕锥顶等比缩放
        for (unsigned int i = 0; i < range.planeCount + range.lineCount; ++i) {
            osg::Vec3f& p = (*m_verts)[range.vertexStartIndex + i];
            osg::Vec3d off(p.x() - range.apex.x(),
                           p.y() - range.apex.y(),
                           p.z() - range.apex.z());
            p.set(range.apex.x() + off.x() * ratio,
                  range.apex.y() + off.y() * ratio,
                  range.apex.z() + off.z() * ratio);
        }
        // 调试坐标轴（独立数组，锥顶即缩放原点）同步缩放
        for (int i = 0; i < range.axisCount; ++i) {
            osg::Vec3f& p = (*m_axesVerts)[range.axisStartIndex + i];
            p.set(range.apex.x() + (p.x() - range.apex.x()) * ratio,
                  range.apex.y() + (p.y() - range.apex.y()) * ratio,
                  range.apex.z() + (p.z() - range.apex.z()) * ratio);
        }
        m_verts->dirty();
        m_axesVerts->dirty();
        m_geom->dirtyBound();
        m_axesGeom->dirtyBound();
    }

    /**
     * @brief 按顶点 ID 更新单个标记的 alpha（不重建几何体）
     *
     * 用于双击聚焦等场景：整体压低透明度后单独抬升目标标记，
     * 使其在淡化环境中仍可辨认。覆盖值被记录，后续 updateSphereColor()
     * 与 setOpacity() 会保留该按标记覆盖，直到再次调用本方法或 clear()。
     *
     * @param vertexId 顶点 ID（需已在 appendFrustum 中添加过）
     * @param alpha    该标记的不透明度（0.0 ~ 1.0）
     */
    void updateSphereOpacity(long vertexId, float alpha) {
        auto it = m_sphereRanges.find(vertexId);
        if (it == m_sphereRanges.end()) return;
        m_alphaOverrides[vertexId] = alpha;
        const auto& range = it->second;
        int total = range.planeCount + range.lineCount;
        for (int i = 0; i < total; ++i)
            (*m_colors)[range.startIndex + i].a() = alpha;
        // 局部坐标轴（独立数组）同步淡化
        for (int i = 0; i < range.axisCount; ++i)
            (*m_axesColors)[range.axisStartIndex + i].a() = alpha;
        m_colors->dirty();
        m_axesColors->dirty();
    }

    /**
     * @brief 按"整体不透明度"语义更新单个标记的 alpha（不重建几何体）
     *
     * 与 updateSphereOpacity() 的平铺 alpha 不同，本方法对投影屏、
     * 线框、坐标轴分别乘以各自的系数（kPlaneAlphaMul/kLineAlphaMul/
     * kAxisAlpha），使目标标记的视觉与全局 setOpacity() 下的正常标记
     * 完全一致。覆盖值同样记录进 m_alphaOverrides，后续
     * updateSphereColor() 会保留。用于播放独显：其余标记全隐藏时，
     * 单独以用户整体不透明度点亮当前帧标记。
     *
     * @param vertexId 顶点 ID（需已在 appendFrustum 中添加过）
     * @param opacity  该标记的整体不透明度（0.0 ~ 1.0）
     */
    void updateSphereOpacityScaled(long vertexId, float opacity) {
        auto it = m_sphereRanges.find(vertexId);
        if (it == m_sphereRanges.end()) return;
        m_alphaOverrides[vertexId] = opacity;
        const auto& range = it->second;
        unsigned int idx = range.startIndex;
        auto apply = [this, &idx, &opacity](const float* mul, int count) {
            for (int i = 0; i < count; ++i)
                (*m_colors)[idx++].a() = opacity * mul[i];
        };
        apply(kPlaneAlphaMul, range.planeCount);
        apply(kLineAlphaMul,  range.lineCount);
        for (int i = 0; i < range.axisCount; ++i)
            (*m_axesColors)[range.axisStartIndex + i].a() = opacity * kAxisAlpha;
        m_colors->dirty();
        m_axesColors->dirty();
    }

    /** @brief 设置标记特征尺寸（触发外部调用方重建几何体） */
    void setRadius(float r) { m_radius = r; }

    /** @brief 获取当前标记特征尺寸 */
    float radius() const { return m_radius; }

    /** @brief 设置是否绘制锥顶局部坐标轴（调试用，下次 append 生效） */
    void setDrawLocalAxes(bool on) { m_drawLocalAxes = on; }

    /** @brief 查询局部坐标轴开关状态 */
    bool drawLocalAxes() const { return m_drawLocalAxes; }

    /**
     * @brief 设置顶点标记的整体不透明度
     *
     * 按顶点 alpha 系数表重写当前颜色数组中的 alpha 值
     * （投影屏 × kPlaneAlpha、线框 × kLineAlpha）；
     * 后续 appendFrustum() 新增的标记也会使用该不透明度。
     * 手动设置整体不透明度视为退出聚焦淡化：清除所有按标记的
     * alpha 覆盖。
     *
     * @param opacity 不透明度（0.0 全透明 ~ 1.0 不透明）
     */
    void setOpacity(float opacity) {
        m_opacity = opacity;
        m_alphaOverrides.clear();
        if (m_sphereRanges.empty()) {
            // 无分区记录时退化为整体统一 alpha
            for (unsigned int i = 0; i < m_colors->size(); ++i)
                (*m_colors)[i].a() = opacity;
            for (unsigned int i = 0; i < m_axesColors->size(); ++i)
                (*m_axesColors)[i].a() = opacity;
        } else {
            for (auto& [id, range] : m_sphereRanges) {
                unsigned int idx = range.startIndex;
                auto apply = [this, &idx, &opacity](const float* mul, int count) {
                    for (int i = 0; i < count; ++i)
                        (*m_colors)[idx++].a() = opacity * mul[i];
                };
                apply(kPlaneAlphaMul, range.planeCount);
                apply(kLineAlphaMul,  range.lineCount);
                for (int i = 0; i < range.axisCount; ++i)
                    (*m_axesColors)[range.axisStartIndex + i].a() =
                        opacity * kAxisAlpha;
            }
        }
        m_colors->dirty();
        m_axesColors->dirty();
    }

    /** @brief 获取当前不透明度 */
    float opacity() const { return m_opacity; }

    /**
     * @brief 在所有标记添加完成后调用，完成构建
     *
     * 标记顶点、颜色、索引数据为脏，使 OSG 将它们上传到 GPU。
     */
    void finish() {
        m_verts->dirty();
        m_colors->dirty();
        m_indices->dirty();
        m_lineIndices->dirty();
        m_axesVerts->dirty();
        m_axesColors->dirty();
        m_axesIndices->dirty();
        m_geom->dirtyBound();
        m_axesGeom->dirtyBound();
    }

    /** @brief 获取包含标记的 OSG 节点 */
    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

    /**
     * @brief 清除所有数据（用于重建）
     */
    void clear() {
        m_verts->clear();
        m_colors->clear();
        m_indices->clear();
        m_lineIndices->clear();
        m_axesVerts->clear();
        m_axesColors->clear();
        m_axesIndices->clear();
        m_sphereRanges.clear();
        m_alphaOverrides.clear();
    }

private:
    // —— 视锥体造型参数（单位 = 特征尺寸 R） ——
    static constexpr float kDepth      = 0.5f;    ///< 视锥体深度（沿局部 +Z）
    static constexpr float kHalfWidth  = 1.0f;    ///< 投影屏半宽（16:9）
    static constexpr float kHalfHeight = 0.5625f; ///< 投影屏半高（9/16 × 半宽）
    // —— 纵向渐变（沿局部"下"方向标明朝向：上面深、下面浅） ——
    static constexpr float kTopShade    = 1.25f;  ///< 图像上方（局部 -Y）亮度系数
    static constexpr float kBottomShade = 0.55f;  ///< 图像下方（局部 +Y）亮度系数
    // —— 分区 alpha（再乘全局不透明度） ——
    static constexpr float kPlaneAlpha = 0.95f;  ///< 投影屏填充面：较实
    static constexpr float kLineAlpha  = 0.55f;  ///< 线框：最亮
    // —— 每标记固定顶点布局（appendFrustum 的推入顺序） ——
    static constexpr int kPlaneVerts = 4;        ///< 投影屏四角
    static constexpr int kLineVerts  = 5;        ///< 线框锥顶 + 四角
    // —— 渐变参数表（与 appendFrustum 顶点推入顺序一一对应） ——
    // t：0 = 图像上方（局部 -Y，最深）、1 = 下方（+Y，最浅）
    static constexpr float kPlaneGradT[kPlaneVerts] = {0.0f, 0.0f, 1.0f, 1.0f};
    static constexpr float kLineGradT[kLineVerts]   = {0.5f, 0.0f, 0.0f, 1.0f, 1.0f};
    // —— 顶点 alpha 系数表（同上顺序） ——
    static constexpr float kPlaneAlphaMul[kPlaneVerts] = {kPlaneAlpha, kPlaneAlpha,
                                                          kPlaneAlpha, kPlaneAlpha};
    static constexpr float kLineAlphaMul[kLineVerts]   = {kLineAlpha, kLineAlpha,
                                                          kLineAlpha, kLineAlpha, kLineAlpha};
    // —— 调试局部坐标轴 ——
    static constexpr float kAxisLen   = 1.0f;    ///< 轴长（× 特征尺寸 R）
    static constexpr float kAxisAlpha = 0.95f;   ///< 坐标轴不透明度系数（× 全局）

    /** @brief appendFrustum 公共前置：准备分区基色并注册追踪范围 */
    void beginAppend(const osg::Vec4& color, long vertexId) {
        osg::Vec4 base = color;
        base.a() *= m_opacity;  // 应用全局不透明度
        m_planeBase = base;             // 投影屏 alpha 按顶点系数表逐点应用
        m_lineBase  = lightened(base);  // 线框整体提亮
        if (vertexId >= 0) {
            m_sphereRanges[vertexId] = {static_cast<unsigned int>(m_colors->size()),
                                        0, 0};
        }
    }

    /** @brief 该标记当前应使用的不透明度：按标记覆盖（聚焦淡化）优先，
     *         否则全局不透明度 */
    float alphaFor(long vertexId) const {
        auto it = m_alphaOverrides.find(vertexId);
        return (it != m_alphaOverrides.end()) ? it->second : m_opacity;
    }

    /** @brief appendFrustum 公共后置：按固定布局回填分区顶点数 */
    void endAppend(long vertexId, const osg::Vec3d& center, float bakedScale) {
        if (vertexId < 0) return;
        auto& range = m_sphereRanges[vertexId];
        int total = static_cast<int>(m_colors->size()) - range.startIndex;
        range.planeCount = kPlaneVerts;
        range.lineCount  = total - kPlaneVerts;
        // 顶点在 m_verts 中也连续排布，记录起始索引与锥顶位置，
        // 供 updateSphereScale() 原地缩放使用
        range.vertexStartIndex =
            static_cast<unsigned int>(m_verts->size()) - total;
        range.apex = center;
        // 记录 append 时烘焙进几何体的实际缩放（如选中帧按 2R 构建
        // 则为 2.0），保证后续 updateSphereScale() 以真实尺寸为基准
        // 增量缩放，不会把已放大的标记再乘一遍
        range.scale = bakedScale;
    }
    /** @brief 同色系提亮：RGB 各 × 0.5 + 0.5（用于线框勾边） */
    static osg::Vec4 lightened(const osg::Vec4& c) {
        return osg::Vec4(c.r() * 0.5f + 0.5f,
                         c.g() * 0.5f + 0.5f,
                         c.b() * 0.5f + 0.5f,
                         c.a());
    }

    /** @brief 计算顶点最终颜色：RGB 按 @p t 做纵向方向渐变
     *         （t = 0 图像上方 × kTopShade 最深 → t = 1 下方 × kBottomShade
     *         最浅，线性插值并钳制），alpha 取 base.a() × @p alphaMul */
    static osg::Vec4 shaded(const osg::Vec4& base, float t, float alphaMul) {
        float f = kTopShade + t * (kBottomShade - kTopShade);
        return osg::Vec4(std::min(1.0f, base.r() * f),
                         std::min(1.0f, base.g() * f),
                         std::min(1.0f, base.b() * f),
                         base.a() * alphaMul);
    }

    /** @brief 追加锥顶局部坐标轴（调试用，X-ray 几何体）
     *
     * 从锥顶沿局部轴向各引一条短线：+X 红（右）/ +Y 绿（下）/
     * +Z 蓝（前），颜色为固定 RGB 惯例，不随标记高亮色变化；
     * 顶点推入顺序固定：X → Y → Z，各 2 个端点。
     */
    void appendLocalAxes(const osg::Vec3d& center, const Eigen::Matrix3f& rot,
                         float R, long vertexId) {
        float L = kAxisLen * R;
        unsigned int start = static_cast<unsigned int>(m_axesVerts->size());

        auto pushAxisVert = [&](float lx, float ly, float lz, const osg::Vec4& c) {
            Eigen::Vector3f local(lx, ly, lz);
            Eigen::Vector3f world = rot * local;
            m_axesVerts->push_back(osg::Vec3(
                center.x() + world.x(),
                center.y() + world.y(),
                center.z() + world.z()));
            m_axesColors->push_back(c);
        };

        float a = m_opacity * kAxisAlpha;
        const osg::Vec4 cx(1.0f, 0.0f, 0.0f, a);       // +X 右
        const osg::Vec4 cy(0.0f, 1.0f, 0.0f, a);       // +Y 下
        const osg::Vec4 cz(0.25f, 0.45f, 1.0f, a);     // +Z 前
        pushAxisVert(0.0f, 0.0f, 0.0f, cx); pushAxisVert(L,  0.0f, 0.0f, cx);
        pushAxisVert(0.0f, 0.0f, 0.0f, cy); pushAxisVert(0.0f, L,     0.0f, cy);
        pushAxisVert(0.0f, 0.0f, 0.0f, cz); pushAxisVert(0.0f, 0.0f,  L,   cz);

        for (int i = 0; i < 3; ++i) {
            m_axesIndices->push_back(start + 2 * i);
            m_axesIndices->push_back(start + 2 * i + 1);
        }

        if (vertexId >= 0) {
            auto& range = m_sphereRanges[vertexId];
            range.axisStartIndex = start;
            range.axisCount = static_cast<int>(m_axesVerts->size()) - start;
        }
    }

    /** @brief 推入一个局部坐标点经 rot 旋转 + center 平移后的顶点，返回其索引。
     *         颜色取 @p base：RGB 按 @p t 纵向渐变（0 顶部深 → 1 底部浅），
     *         alpha 乘 @p alphaMul */
    int pushVertex(float lx, float ly, float lz,
                   const osg::Vec3d& center, const Eigen::Matrix3f& rot,
                   const osg::Vec4& base, float t, float alphaMul) {
        Eigen::Vector3f local(lx, ly, lz);
        Eigen::Vector3f world = rot * local;
        m_verts->push_back(osg::Vec3(
            center.x() + world.x(),
            center.y() + world.y(),
            center.z() + world.z()));
        m_colors->push_back(shaded(base, t, alphaMul));
        return static_cast<int>(m_verts->size()) - 1;
    }

    float m_radius;          ///< 标记特征尺寸（视锥体深度 = 2R）
    float m_opacity = 1.0f;  ///< 标记整体不透明度（1.0 不透明）
    bool  m_drawLocalAxes = true;  ///< 是否绘制锥顶局部坐标轴（调试用）

    osg::ref_ptr<osg::Geode> m_geode;              ///< 叶节点
    osg::ref_ptr<osg::Geometry> m_geom;            ///< 标记几何体
    osg::ref_ptr<osg::Vec3Array> m_verts;          ///< 标记顶点数组
    osg::ref_ptr<osg::Vec4Array> m_colors;         ///< 标记颜色数组
    osg::ref_ptr<osg::DrawElementsUInt> m_indices;     ///< 投影屏索引数组
    osg::ref_ptr<osg::DrawElementsUInt> m_lineIndices; ///< 线框索引数组

    osg::ref_ptr<osg::Geometry> m_axesGeom;            ///< 坐标轴几何体（X-ray）
    osg::ref_ptr<osg::Vec3Array> m_axesVerts;          ///< 坐标轴顶点数组
    osg::ref_ptr<osg::Vec4Array> m_axesColors;         ///< 坐标轴颜色数组
    osg::ref_ptr<osg::DrawElementsUInt> m_axesIndices; ///< 坐标轴索引数组

    osg::Vec4 m_planeBase;   ///< 当前标记投影屏基色（append 期间临时使用）
    osg::Vec4 m_lineBase;    ///< 当前标记线框基色（已提亮，同上）

    /** @brief 顶点 ID → 颜色数组分区范围的映射（用于增量颜色更新）
     *         由 beginAppend()/endAppend() 在添加标记时填充 */
    std::unordered_map<long, SphereRange> m_sphereRanges;

    /** @brief 按标记的 alpha 覆盖（聚焦淡化目标标记），updateSphereColor()
     *         与 setOpacity() 保留该覆盖，避免轻量路径冲掉聚焦效果 */
    std::unordered_map<long, float> m_alphaOverrides;
};
