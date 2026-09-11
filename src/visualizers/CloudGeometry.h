// ============================================================================
// CloudGeometry.h
// 点云渲染几何体：帧局部系顶点 + 逐顶点帧号 + 动态世界包围盒
//
// 为什么是 osg::Geometry 的子类，而不是自定义 osg::Drawable：
//   顶点上传、VAO 管理、glDrawArrays/glDrawElements 全部由 osg::Geometry 承担，
//   本类只改两件事 —— 包围盒怎么算、以及要不要把图元交给原生相交器。
//   自己写 Drawable 意味着手写 VBO 绑定与 drawImplementation，风险与收益不成比例。
//   （另：osg::Drawable 的 accept(PrimitiveFunctor&) 本身就是空实现
//     —— osg/Drawable:503 —— 但 osg::Geometry 把它覆写成"喂出顶点数组"
//     并且 supports() 返回 true —— osg/Geometry:203 —— 所以必须显式屏蔽。）
//
// 三个关键点：
//   1. **顶点是帧局部系坐标**，世界坐标由顶点着色器用位姿表算出
//      （CloudRenderData.h 提供位姿，CloudPoseBuffer.h 提供纹理）。
//      因此几何体与位姿无关：位姿优化后不需要重建、不需要重传顶点。
//   2. **computeBoundingBox() 返回动态世界 AABB**。OSG 的视锥剔除读的正是这个值，
//      所以它必须是"当前位姿下的真实世界包围盒"，由外层在每次位姿变化后调用
//      setWorldBounds() 刷新。这是让 OSG 原生剔除在位姿变化后仍然正确的关键，
//      也是 Phase 3 的核心。
//   3. **屏蔽原生相交器**。osg::Geometry 默认会把**帧局部系**顶点喂给
//      LineSegmentIntersector / PolytopeIntersector，而射线是世界系的 —— 那会
//      产出"看起来合理但世界系错误"的交点（典型症状：双击点云后相机飞到原点
//      附近）。这里宁可返回"无命中"也不返回错命中。
//
// ⚠️ 接入前置条件（重要，别踩）：
//   把渲染路径切到本类之前，必须先落地 Phase 4 的 CloudRayIntersector。
//   否则双击拾取会从"结果错误"变成"完全没有结果"—— 两者都是回归，后者更诚实，
//   但必须与 Phase 4 一起交付，不能单独上线。
// ============================================================================

#pragma once

#include <osg/BoundingBox>
#include <osg/CopyOp>
#include <osg/Geometry>
#include <osg/Object>
#include <osg/PrimitiveSet>

#include <vector>

namespace hdl_graph_slam {

/**
 * @brief 关键帧点云的渲染几何体（帧局部系顶点 + 逐顶点帧号 + 动态世界 AABB）
 *
 * 顶点数据由 `buildChunks()`（CloudRenderData.h）产出，本类只负责装载与
 * LOD 级别切换。切换级别是**替换 primitive set**，不触碰顶点数组，
 * 因此没有任何 GPU 重传：level 0 用 `DrawArrays`（全量），level L>=1 用
 * 预先建好的 `DrawElementsUInt`（指向同一份顶点）。
 */
class CloudGeometry : public osg::Geometry {
public:
    CloudGeometry();
    CloudGeometry(const CloudGeometry& rhs,
                  const osg::CopyOp& copyop = osg::CopyOp::SHALLOW_COPY);

    META_Object(hdl_graph_slam, CloudGeometry)

    /**
     * @brief 帧号自定义属性的 attribute location
     *
     * 必须避开 OSG 内建名（osg_Vertex / osg_Normal / osg_Color /
     * osg_MultiTexCoord0..7 等）占用的槽位；取 13 是保守选择。
     * 着色器侧必须用 `layout(location = 13) in float aFrameId;` 显式声明，
     * 因为 `aFrameId` 不在 OSG 的默认绑定表里，不声明则绑定不确定
     * （症状：帧号读成 0 → 所有点堆到同一帧的位姿上）。
     */
    static constexpr unsigned int kFrameIdAttribLocation = 13;

    /**
     * @brief 一次性装载点云数据（装载后自动落到 level 0）
     *
     * @param positions     帧局部系顶点（float3），按帧序连续
     * @param frameLocalIds 逐顶点帧号（chunk 内帧序号，长度须等于顶点数）
     * @param lodIndices    LOD 索引子集，lodIndices[0] 对应 level 1
     *                      （元素为 positions 的下标，升序且 < 顶点数）
     */
    void setCloudData(osg::Vec3Array* positions,
                      osg::UShortArray* frameLocalIds,
                      const std::vector<osg::ref_ptr<osg::DrawElementsUInt>>& lodIndices);

    // ------------------------------------------------------------------
    // LOD 级别
    // ------------------------------------------------------------------

    /**
     * @brief 切换到指定 LOD 级别（0 = 全量）
     *
     * 只替换 primitive set，不修改顶点数组 → 无 GPU 重传。
     * 越界值会被夹到 [0, lodLevelCount()-1]。
     */
    void setActiveLodLevel(int level);

    /** @brief 当前激活级别（0 = 全量） */
    int activeLodLevel() const { return m_activeLod; }

    /** @brief 级别总数（含 level 0） */
    int lodLevelCount() const { return 1 + static_cast<int>(m_lodIndex.size()); }

    // ------------------------------------------------------------------
    // 动态世界包围盒（OSG 视锥剔除读取）
    // ------------------------------------------------------------------

    /**
     * @brief 设置当前位姿下的世界 AABB 并让 OSG 重算包围体
     *
     * 位姿变化后由外层调用（见 CloudRenderData.h 的 refreshAllWorldBounds）。
     * 代价 O(1)，与点数无关。
     */
    void setWorldBounds(const osg::BoundingBox& bb);

    /** @brief 当前世界 AABB（未设置时 invalid） */
    const osg::BoundingBox& worldBounds() const { return m_worldBounds; }

    // ------------------------------------------------------------------
    // OSG 覆写
    // ------------------------------------------------------------------

    /**
     * @brief 返回**世界** AABB（而不是顶点数组的局部 AABB）
     *
     * 刻意不调用基类实现：基类会从帧局部系顶点算盒，那是局部盒，
     * 直接导致视锥剔除失效（这正是现有实现的缺陷 B）。
     * 未设置世界盒时返回 invalid box —— OSG 对无效包围体的处理是
     * "不剔除"，这是安全的失败方向（宁可多画，不可错剔）。
     */
    osg::BoundingBox computeBoundingBox() const override;

    /** @brief 屏蔽：帧局部系图元不能交给世界系射线使用（见头注释第 3 点） */
    void accept(osg::PrimitiveFunctor&) const override {}
    void accept(osg::PrimitiveIndexFunctor&) const override {}
    bool supports(const osg::PrimitiveFunctor&) const override { return false; }
    bool supports(const osg::PrimitiveIndexFunctor&) const override { return false; }

private:
    /** @brief 依 m_activeLod 重建 primitive set（level 0 = DrawArrays 全量） */
    void applyActivePrimitiveSet();

    std::vector<osg::ref_ptr<osg::DrawElementsUInt>> m_lodIndex;  ///< level 1..n 的索引
    osg::ref_ptr<osg::DrawArrays> m_fullPrimitive;                ///< level 0 的全量绘制
    int m_activeLod = 0;                                          ///< 当前级别
    osg::BoundingBox m_worldBounds;                               ///< 当前位姿下的世界 AABB
};

}  // namespace hdl_graph_slam
