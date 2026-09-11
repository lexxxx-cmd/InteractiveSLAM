// ============================================================================
// CloudGeometry.cpp
// CloudGeometry 实现
// ============================================================================

#include "visualizers/CloudGeometry.h"

#include <osg/Array>
#include <osg/Drawable>
#include <osg/GL>

#include <cstddef>
#include <vector>

namespace hdl_graph_slam {

CloudGeometry::CloudGeometry() {
    // 点云顶点量大且只上传一次，用 VBO + VAO；不用显示列表
    setUseDisplayList(false);
    setUseVertexBufferObjects(true);
    setUseVertexArrayObject(true);
    setSupportsDisplayList(false);
    // DYNAMIC：本类不在正常情况下改数组，但保留宽容度 —— 若将来有路径
    // 复用/脏化数组，DYNAMIC 能保证重新上传，避免"改了不生效"的沉默 bug
    setDataVariance(osg::Object::DYNAMIC);
}

CloudGeometry::CloudGeometry(const CloudGeometry& rhs, const osg::CopyOp& copyop)
    : osg::Geometry(rhs, copyop),
      m_lodIndex(rhs.m_lodIndex),
      m_fullPrimitive(rhs.m_fullPrimitive),
      m_activeLod(rhs.m_activeLod),
      m_worldBounds(rhs.m_worldBounds) {}

// ----------------------------------------------------------------------------
// 数据装载
// ----------------------------------------------------------------------------

void CloudGeometry::setCloudData(
    osg::Vec3Array* positions, osg::UShortArray* frameLocalIds,
    const std::vector<osg::ref_ptr<osg::DrawElementsUInt>>& lodIndices) {
    setVertexArray(positions);

    // 逐顶点帧号 → 自定义属性。location 由着色器显式声明，见头文件说明。
    if (frameLocalIds) {
        setVertexAttribArray(kFrameIdAttribLocation, frameLocalIds,
                             osg::Array::BIND_PER_VERTEX);
    }

    m_lodIndex = lodIndices;
    m_activeLod = 0;
    applyActivePrimitiveSet();
    dirtyBound();  // 顶点数组换了，让 OSG 重新收集（世界盒由 setWorldBounds 决定）
}

// ----------------------------------------------------------------------------
// LOD
// ----------------------------------------------------------------------------

void CloudGeometry::applyActivePrimitiveSet() {
    // 清空现有 primitive set。注意：这只影响绘制命令，不会释放顶点数组，
    // 因此不会引起任何顶点重传。
    removePrimitiveSet(0, getNumPrimitiveSets());

    const int maxLevel = lodLevelCount() - 1;
    if (m_activeLod <= 0 || m_activeLod > maxLevel) {
        // level 0：全量直绘，点数从顶点数组推导（不缓存，避免与数组不同步）
        if (!m_fullPrimitive.valid()) {
            m_fullPrimitive = new osg::DrawArrays(GL_POINTS, 0, 0);
        }
        const osg::Array* va = getVertexArray();
        m_fullPrimitive->setCount(
            va ? static_cast<GLsizei>(va->getNumElements()) : 0);
        addPrimitiveSet(m_fullPrimitive.get());
        return;
    }

    // level L>=1：索引子集，指向同一份顶点数组
    addPrimitiveSet(m_lodIndex[static_cast<size_t>(m_activeLod) - 1].get());
}

void CloudGeometry::setActiveLodLevel(int level) {
    const int maxLevel = lodLevelCount() - 1;
    if (level < 0) level = 0;
    if (level > maxLevel) level = maxLevel;
    if (level == m_activeLod && getNumPrimitiveSets() > 0) return;

    m_activeLod = level;
    applyActivePrimitiveSet();
}

// ----------------------------------------------------------------------------
// 世界包围盒
// ----------------------------------------------------------------------------

void CloudGeometry::setWorldBounds(const osg::BoundingBox& bb) {
    m_worldBounds = bb;
    // 让 OSG 重算剔除用的包围体。代价 O(1)，与点数无关 ——
    // 这就是"位姿变化后视锥剔除仍然正确"的全部代价。
    dirtyBound();
}

osg::BoundingBox CloudGeometry::computeBoundingBox() const {
    return m_worldBounds;
}

}  // namespace hdl_graph_slam
