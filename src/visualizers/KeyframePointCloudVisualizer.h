// ============================================================================
// KeyframePointCloudVisualizer.h
// 关键帧点云可视化器
//
// 功能：在世界坐标系中构建合并的点云几何体。每个关键帧的局部激光雷达
//       点通过 g2o 位姿在 CPU 上变换到世界坐标系，然后追加到单个 VBO 中。
//       与 EdgeLineVisualizer 和 VertexSphereVisualizer 使用相同的模式。
//
//       记录每个关键帧的顶点范围，使得选中关键帧时可以仅重新着色
//       该帧的点云，而无需重建整个 VBO。
//
// 大点云支持（方案 A：点预算自适应降采样）：
//       - 全量世界点始终保留在内存中（m_allWorldPoints），用于 Z 范围统计、
//         高程颜色映射、高亮重着色等；
//       - 渲染时根据点预算 setPointBudget() 对合并点云做体素降采样，仅将
//         降采样后的子集上传到 GPU。体素边长通过"测量-调整"迭代自动选取，
//         使渲染点数收敛到预算以内（上限语义，不逐帧调节，避免 FPS 反馈振荡）；
//       - 渲染索引经 m_renderIndices 映射回全量点，m_renderRanges 记录每个
//         关键帧在渲染数组中的范围，因此选中高亮 / 重着色在降采样后依然正确；
//       - 预算 ≤ 0 表示全量渲染（与旧行为一致）。
//
// 着色方案：默认使用 Turbo 颜色映射根据高程（Z 值）着色。
//           选中关键帧的点云高亮为白色。
// ============================================================================

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

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "visualizers/TurboColormap.h"
#include "visualizers/CoreShaders.h"

/**
 * @brief 关键帧点云可视化器
 *
 * 将所有关键帧的点云合并到单个几何体中，通过 CPU 变换将每个点
 * 从关键帧局部坐标系变换到世界坐标系。
 *
 * 核心功能：
 *   1. appendCloud() —— 添加一个关键帧的点云（按位姿变换到世界坐标）
 *   2. finish() —— 完成点云构建，应用 Turbo 颜色映射并上传到 GPU
 *   3. recolorHighlight() —— 高亮选中的关键帧点云（白色）
 *   4. Z 轴裁剪（着色器级别，无需重建 VBO）
 *   5. 高程颜色范围动态调整（CPU 重新着色，无需重建顶点）
 *   6. 点预算自适应降采样（体素化，渲染点数 ≤ 预算）
 */
class KeyframePointCloudVisualizer {
public:
    /** @brief 构造函数：初始化点云几何体和着色器 */
    KeyframePointCloudVisualizer() {
        m_geom = new osg::Geometry;
        m_geom->setUseDisplayList(false);
        m_geom->setUseVertexBufferObjects(true);
        m_geom->setUseVertexArrayObject(true);
        m_geom->setDataVariance(osg::Object::DYNAMIC);

        // 顶点和颜色数组
        m_vertices = new osg::Vec3Array;
        m_colors   = new osg::Vec4Array;

        m_geom->setVertexArray(m_vertices);
        m_geom->setColorArray(m_colors, osg::Array::BIND_PER_VERTEX);
        m_geom->addPrimitiveSet(new osg::DrawArrays(GL_POINTS, 0, 0));

        // 设置点云着色器
        auto* ss = m_geom->getOrCreateStateSet();
        m_pointSizeUniform = applyPointCloudShader(ss, m_pointSize);

        // 查找 Z 轴裁剪 uniform（由 applyPointCloudShader 添加）
        m_zClipUniform  = ss->getUniform("z_clipping");
        m_zRangeUniform = ss->getUniform("z_range");

        // 透明度控制（使用 per-vertex alpha，支持逐帧差异化透明度）
        ss->setAttributeAndModes(
            new osg::BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA),
            osg::StateAttribute::ON);

        m_geode = new osg::Geode;
        m_geode->addDrawable(m_geom);

        // 包围盒初始化为空
        m_bMin = Eigen::Vector3d( std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max());
        m_bMax = Eigen::Vector3d(-std::numeric_limits<double>::max(),
                                 -std::numeric_limits<double>::max(),
                                 -std::numeric_limits<double>::max());
    }

    /**
     * @brief 添加一个关键帧的点云（变换到世界坐标系）
     *
     * 将每个点通过给定的位姿变换到世界坐标系，并更新 Z 值范围与包围盒。
     *
     * @param cloud    局部坐标系下的点云（const 引用）
     * @param pose     关键帧的 SE3 位姿（世界坐标系）
     * @param vertexId 关键帧对应的顶点 ID
     */
    void appendCloud(pcl::PointCloud<pcl::PointXYZI>::ConstPtr cloud,
                     const Eigen::Isometry3d& pose,
                     long vertexId) {
        if (!cloud || cloud->empty()) return;

        size_t start = m_allWorldPoints.size();

        // 将每个点通过位姿变换到世界坐标系
        for (const auto& pt : cloud->points) {
            Eigen::Vector3d wp = pose * Eigen::Vector3d(pt.x, pt.y, pt.z);
            float wz = static_cast<float>(wp.z());
            // 更新 Z 值范围（用于 Turbo 颜色映射和裁剪）
            if (wz < m_zMin) m_zMin = wz;
            if (wz > m_zMax) m_zMax = wz;
            // 更新包围盒（用于降采样体素边长估计）
            if (wp.x() < m_bMin.x()) m_bMin.x() = wp.x();
            if (wp.y() < m_bMin.y()) m_bMin.y() = wp.y();
            if (wp.z() < m_bMin.z()) m_bMin.z() = wp.z();
            if (wp.x() > m_bMax.x()) m_bMax.x() = wp.x();
            if (wp.y() > m_bMax.y()) m_bMax.y() = wp.y();
            if (wp.z() > m_bMax.z()) m_bMax.z() = wp.z();
            m_allWorldPoints.push_back(wp);
        }

        size_t count = m_allWorldPoints.size() - start;
        m_cloudRanges.push_back({start, count, vertexId});
    }

    /**
     * @brief 完成点云构建，上传数据并应用初始 Turbo 颜色映射
     *
     * 在添加完所有关键帧的点云后调用，执行以下操作：
     *   1. 初始化颜色范围（自动模式）
     *   2. 初始化 Z 轴裁剪范围（首次加载）
     *   3. 按点预算构建渲染顶点/颜色数据并上传到 GPU
     *   4. 应用 Turbo 颜色映射（根据 Z 值着色）
     */
    void finish() {
        if (m_allWorldPoints.empty()) return;

        // 防止 Z 值范围过小导致的颜色映射异常
        if (m_zMax - m_zMin < 0.001f) {
            m_zMin -= 0.5f;
            m_zMax += 0.5f;
        }

        // 仅在自动模式下从数据初始化颜色范围
        if (m_useAutoColorRange) {
            m_colorZMin = m_zMin;
            m_colorZMax = m_zMax;
        }

        // 首次加载时从数据初始化裁剪范围
        if (!m_clipRangeInitialized) {
            m_zClipMin = m_zMin;
            m_zClipMax = m_zMax;
            m_clipRangeInitialized = true;
        }

        // 将 Z 轴裁剪范围推送到 GPU
        if (m_zRangeUniform)
            m_zRangeUniform->set(osg::Vec2(m_zClipMin, m_zClipMax));

        // 按点预算构建渲染数据（内部处理降采样 / 全量两种路径）
        rebuildRenderCloud();
    }

    /**
     * @brief 对指定关键帧集合的点云差异化透明度
     *
     * 在 finish() 之后调用。选中/高亮帧保持当前透明度 m_opacity，
     * 非选中帧透明度变为 m_opacity × 0.5，形成视觉层次。
     * 传入空集合时所有帧降为半透明（"暗淡"效果，恢复完整透明度
     * 请调用 setOpacity）。
     *
     * @param highlightIds 需要保持完全可见的顶点 ID 集合。
     */
    void recolorHighlight(const std::set<long>& highlightIds) {
        m_highlightIds = highlightIds;
        applyHighlight();
    }

    /**
     * @brief 清除所有数据（用于完全重建）
     */
    void clear() {
        m_allWorldPoints.clear();
        m_cloudRanges.clear();
        m_renderIndices.clear();
        m_renderRanges.clear();
        m_renderFullRes = true;
        m_highlightIds.clear();
        m_vertices->clear();
        m_colors->clear();
        m_zMin =  std::numeric_limits<float>::max();
        m_zMax = -std::numeric_limits<float>::max();
        m_bMin = Eigen::Vector3d( std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max(),
                                  std::numeric_limits<double>::max());
        m_bMax = Eigen::Vector3d(-std::numeric_limits<double>::max(),
                                 -std::numeric_limits<double>::max(),
                                 -std::numeric_limits<double>::max());
        m_useAutoColorRange = true;
        m_clipRangeInitialized = false;
    }

    /** @brief 设置点的大小（像素单位） */
    void setPointSize(float size) {
        m_pointSize = size;
        if (m_pointSizeUniform)
            m_pointSizeUniform->set(size);
    }

    /** @brief 设置点云透明度 */
    void setOpacity(float opacity) {
        m_opacity = opacity;
        // 透明度变化需更新所有顶点的 alpha 通道
        recolorAll();
    }

    // ========================================================================
    // 点预算自适应降采样（方案 A）
    // ========================================================================

    /**
     * @brief 设置渲染点预算（点数上限）
     *
     * 当全量点数超过预算时，对合并点云做体素降采样，使渲染点数收敛到
     * 预算以内（体素边长自动迭代选取）。全量点数据保留在内存中不变。
     *
     * @param maxPoints 渲染点数上限；≤ 0 表示全量渲染（不降采样）
     */
    void setPointBudget(int maxPoints) {
        m_maxRenderPoints = maxPoints;
        rebuildRenderCloud();
    }

    /** @brief 查询当前点预算（≤ 0 表示全量） */
    int pointBudget() const { return m_maxRenderPoints; }

    /** @brief 当前实际渲染到 GPU 的点数（降采样后的数量） */
    int renderPointCount() const {
        return static_cast<int>(m_vertices->size());
    }

    /** @brief 全量点云总点数（降采样前的原始数量） */
    size_t totalPointCount() const { return m_allWorldPoints.size(); }

    // ========================================================================
    // Z 轴裁剪控制（着色器级别，无需重建 VBO）
    // ========================================================================

    /** @brief 启用/禁用 Z 轴裁剪 */
    void setZClipping(bool enabled) {
        m_zClipping = enabled;
        if (m_zClipUniform)
            m_zClipUniform->set(enabled ? 1 : 0);
    }

    /** @brief 设置 Z 轴裁剪范围 */
    void setZClipRange(float minZ, float maxZ) {
        m_zClipMin = minZ;
        m_zClipMax = maxZ;
        if (m_zRangeUniform)
            m_zRangeUniform->set(osg::Vec2(minZ, maxZ));
    }

    /** @brief 查询 Z 裁剪状态 */
    bool isZClipping() const { return m_zClipping; }
    float getZClipMin() const { return m_zClipMin; }
    float getZClipMax() const { return m_zClipMax; }

    // ========================================================================
    // 高程颜色范围控制（CPU 重新着色，无需重建顶点）
    // ========================================================================

    /** @brief 手动设置颜色 Z 值范围（将关闭自动范围） */
    void setColorZRange(float minZ, float maxZ) {
        m_colorZMin = minZ;
        m_colorZMax = maxZ;
        m_useAutoColorRange = false;
        recolorAll();
    }

    /** @brief 设置是否自动计算颜色范围 */
    void setAutoColorRange(bool autoRange) {
        m_useAutoColorRange = autoRange;
        if (autoRange) {
            m_colorZMin = m_zMin;
            m_colorZMax = m_zMax;
        }
        recolorAll();
    }

    /** @brief 查询是否使用自动颜色范围 */
    bool isAutoColorRange() const { return m_useAutoColorRange; }

    // ========================================================================
    // 数据范围访问（用于 UI 初始化）
    // ========================================================================

    float getDataZMin() const { return m_zMin; }     ///< 数据 Z 最小值
    float getDataZMax() const { return m_zMax; }     ///< 数据 Z 最大值
    float getColorZMin() const { return m_colorZMin; } ///< 颜色映射 Z 最小值
    float getColorZMax() const { return m_colorZMax; } ///< 颜色映射 Z 最大值

    /** @brief 获取 OSG 节点 */
    osg::ref_ptr<osg::Geode> getNode() const { return m_geode; }

    /** @brief 返回点云中的渲染点数（降采样后） */
    int pointCount() const {
        auto* prim = static_cast<osg::DrawArrays*>(m_geom->getPrimitiveSet(0));
        return prim ? prim->getCount() : 0;
    }

private:
    /**
     * @brief 每个关键帧点云的范围结构
     *
     * 用于快速定位每个关键帧对应的顶点范围，避免在重新着色时遍历所有顶点。
     */
    struct CloudRange {
        size_t startVertex;   ///< 该关键帧在（渲染）顶点数组中的起始索引
        size_t vertexCount;   ///< 该关键帧的（渲染）顶点数量
        long   vertexId;      ///< 关键帧对应的顶点 ID
    };

    /**
     * @brief 体素键哈希（FNV-1a 64 位）
     *
     * 将三维整数体素坐标混合为 64 位键。哈希碰撞的概率极低，
     * 即使发生也只会合并两个相邻体素（视觉上无影响）。
     */
    static uint64_t voxelKey(float x, float y, float z, float invLeaf) {
        int64_t ix = static_cast<int64_t>(std::floor(x * invLeaf));
        int64_t iy = static_cast<int64_t>(std::floor(y * invLeaf));
        int64_t iz = static_cast<int64_t>(std::floor(z * invLeaf));
        uint64_t h = 14695981039346656037ull;  // FNV-1a 64 位偏移基值
        h = (h ^ static_cast<uint64_t>(ix)) * 1099511628211ull;
        h = (h ^ static_cast<uint64_t>(iy)) * 1099511628211ull;
        h = (h ^ static_cast<uint64_t>(iz)) * 1099511628211ull;
        return h;
    }

    /**
     * @brief 按点预算构建渲染顶点/颜色数据
     *
     * 两条路径：
     *   - 全量（预算 ≤ 0 或点数未超预算）：渲染索引与全量索引一一对应；
     *   - 降采样：迭代选取体素边长，使体素数（≈ 渲染点数）收敛到预算以内，
     *     然后构建渲染索引与逐帧渲染范围。
     * 无论哪条路径，都从全量点重新生成顶点/颜色数组并上传 GPU。
     */
    void rebuildRenderCloud() {
        if (m_allWorldPoints.empty()) return;

        const bool decimate = (m_maxRenderPoints > 0 &&
                               m_allWorldPoints.size() > (size_t)m_maxRenderPoints);
        if (decimate) {
            // 自适应迭代：测量当前体素边长下的幸存点数，超预算则放大边长
            // 表面分布近似 count ∝ 1/leaf²，取 0.5 次方调整可快速收敛
            float leaf = estimateLeaf();
            for (int iter = 0; iter < 6; ++iter) {
                size_t c = countVoxels(leaf);
                if (c == 0 || c <= (size_t)m_maxRenderPoints) break; // 已满足预算
                double r = (double)c / (double)m_maxRenderPoints;
                leaf *= (float)(std::pow(r, 0.5) * 1.03);
            }
            buildDecimated(leaf);
        } else {
            // 全量渲染：渲染索引与全量索引一一对应
            m_renderRanges = m_cloudRanges;
            m_renderIndices.clear();
            m_renderFullRes = true;
        }

        // 构建 GPU 顶点和颜色数组（从渲染索引对应的全量点生成）
        const size_t renderCount =
            m_renderFullRes ? m_allWorldPoints.size() : m_renderIndices.size();
        m_vertices->clear();
        m_colors->clear();
        m_vertices->reserve(renderCount);
        m_colors->reserve(renderCount);

        for (size_t i = 0; i < renderCount; ++i) {
            const Eigen::Vector3d& wp =
                m_allWorldPoints[m_renderFullRes ? i : m_renderIndices[i]];
            m_vertices->push_back(osg::Vec3(
                static_cast<float>(wp.x()),
                static_cast<float>(wp.y()),
                static_cast<float>(wp.z())));
            osg::Vec4 color = turboColor(
                static_cast<float>(wp.z()), m_colorZMin, m_colorZMax);
            color.a() = m_opacity;
            m_colors->push_back(color);
        }

        // 标记数据为脏，使 OSG 重新上传到 GPU
        m_vertices->dirty();
        m_colors->dirty();

        // 更新图元计数
        auto* prim = static_cast<osg::DrawArrays*>(m_geom->getPrimitiveSet(0));
        if (prim) prim->setCount(m_vertices->size());
        m_geom->dirtyBound();

        // 若存在高亮（选中顶点 / 播放高亮），重建后重新应用
        if (!m_highlightIds.empty()) applyHighlight();
    }

    /**
     * @brief 统计给定体素边长下合并点云的体素数（≈ 渲染点数）
     * @param leaf 体素边长（米）
     * @return 体素数量
     */
    size_t countVoxels(float leaf) const {
        const float inv = 1.0f / leaf;
        std::unordered_set<uint64_t> seen;
        seen.reserve(std::min<size_t>(m_allWorldPoints.size(), 4u * 1024u * 1024u));
        for (const auto& p : m_allWorldPoints) {
            seen.insert(voxelKey(static_cast<float>(p.x()),
                                 static_cast<float>(p.y()),
                                 static_cast<float>(p.z()), inv));
        }
        return seen.size();
    }

    /**
     * @brief 按给定体素边长构建降采样后的渲染索引与逐帧渲染范围
     *
     * 每个体素保留按原始顺序最先出现的点，因此渲染索引天然升序、
     * 逐帧范围可直接按关键帧切分计算。每帧至少保留一个点，
     * 保证选中/播放高亮时该帧仍有可见点。
     *
     * @param leaf 体素边长（米）
     */
    void buildDecimated(float leaf) {
        const float inv = 1.0f / leaf;
        // 体素键 -> 该体素首个点的全量索引
        std::unordered_map<uint64_t, size_t> kept;
        kept.reserve(std::min<size_t>(m_allWorldPoints.size(),
                                      (size_t)std::max(m_maxRenderPoints, 1) * 2));

        std::vector<size_t> sel;
        sel.reserve(std::min<size_t>(m_allWorldPoints.size(),
                                     (size_t)std::max(m_maxRenderPoints, 1)));

        m_renderRanges.clear();
        m_renderRanges.reserve(m_cloudRanges.size());

        for (const auto& range : m_cloudRanges) {
            size_t rstart = sel.size();
            for (size_t j = range.startVertex;
                 j < range.startVertex + range.vertexCount; ++j) {
                const Eigen::Vector3d& p = m_allWorldPoints[j];
                uint64_t k = voxelKey(static_cast<float>(p.x()),
                                      static_cast<float>(p.y()),
                                      static_cast<float>(p.z()), inv);
                if (kept.emplace(k, j).second) {
                    sel.push_back(j);
                }
            }
            // 每帧至少保留一个点
            if (sel.size() == rstart && range.vertexCount > 0) {
                sel.push_back(range.startVertex);
            }
            m_renderRanges.push_back({rstart, sel.size() - rstart, range.vertexId});
        }

        m_renderIndices.swap(sel);
        m_renderFullRes = false;
    }

    /**
     * @brief 估计初始体素边长（从包围盒体积与预算反推）
     *
     * 初始值刻意偏大（×2），使首轮体素数偏小，避免极端分布下
     * 临时哈希表占用过大的内存峰值。
     *
     * @return 初始体素边长（米）
     */
    float estimateLeaf() const {
        double extX = m_bMax.x() - m_bMin.x();
        double extY = m_bMax.y() - m_bMin.y();
        double extZ = m_bMax.z() - m_bMin.z();
        double vol = extX * extY * extZ;
        if (!(vol > 0.0)) vol = 1.0;
        double budget = (double)std::max(1, m_maxRenderPoints);
        float leaf = static_cast<float>(std::cbrt(vol / budget));
        if (!(leaf > 1e-4f)) leaf = 0.1f;
        return leaf * 2.0f;
    }

    /**
     * @brief 应用当前高亮集合（选中帧白色，其余帧半透明）
     *
     * 遍历渲染范围而非全量范围，索引经 m_renderIndices 映射回全量点
     * 以获取正确的 Z 值。降采样后逐帧范围仍保持完整。
     */
    void applyHighlight() {
        if (!m_colors || m_renderRanges.empty()) return;

        const osg::Vec4 white(1.0f, 1.0f, 1.0f, 1.0f);
        // 遍历每个关键帧的渲染顶点范围
        for (const auto& range : m_renderRanges) {
            float alpha = m_highlightIds.count(range.vertexId)
                              ? m_opacity          // 选中帧：保持当前透明度
                              : m_opacity * 0.5f;  // 非选中帧：半透明度
            for (size_t i = range.startVertex;
                 i < range.startVertex + range.vertexCount; ++i) {
                size_t fi = m_renderFullRes ? i : m_renderIndices[i];
                float wz = static_cast<float>(m_allWorldPoints[fi].z());
                if (alpha == m_opacity) (*m_colors)[i] = white;
                else {
                    (*m_colors)[i] = turboColor(wz, m_colorZMin, m_colorZMax);
                    (*m_colors)[i].a() = alpha;
                }
            }
        }
        m_colors->dirty();
    }

    /**
     * @brief 根据当前颜色 Z 值范围重新计算所有渲染顶点颜色
     *
     * 不重建顶点数据，仅更新颜色数组。
     */
    void recolorAll() {
        if (!m_colors || m_vertices->empty()) return;
        if (m_colorZMax - m_colorZMin < 0.001f) return;

        for (size_t i = 0; i < m_vertices->size(); ++i) {
            size_t fi = m_renderFullRes ? i : m_renderIndices[i];
            float wz = static_cast<float>(m_allWorldPoints[fi].z());
            (*m_colors)[i] = turboColor(wz, m_colorZMin, m_colorZMax);
            (*m_colors)[i].a() = m_opacity;
        }
        m_colors->dirty();
    }

    // —— OSG 对象 ——
    osg::ref_ptr<osg::Geode>     m_geode;         ///< 叶节点
    osg::ref_ptr<osg::Geometry>  m_geom;          ///< 点云几何体
    osg::ref_ptr<osg::Vec3Array> m_vertices;      ///< 渲染顶点数组（降采样后）
    osg::ref_ptr<osg::Vec4Array> m_colors;        ///< 渲染颜色数组（降采样后）
    // BlendColor 已移除，透明度通过 per-vertex alpha 控制
    osg::ref_ptr<osg::Uniform>    m_pointSizeUniform; ///< 点大小 uniform
    osg::ref_ptr<osg::Uniform>    m_zClipUniform;     ///< Z 轴裁剪开关 uniform
    osg::ref_ptr<osg::Uniform>    m_zRangeUniform;    ///< Z 轴裁剪范围 uniform

    // —— 世界坐标系点数据（全量保留，供着色、高亮、统计、导出使用） ——
    std::vector<Eigen::Vector3d,
                Eigen::aligned_allocator<Eigen::Vector3d>> m_allWorldPoints; ///< 所有世界坐标点

    // —— 每个关键帧的全量云范围（用于降采样时按帧统计） ——
    std::vector<CloudRange> m_cloudRanges;

    // —— 降采样渲染状态 ——
    int  m_maxRenderPoints = -1;           ///< 渲染点预算（≤0 = 全量）
    std::vector<size_t> m_renderIndices;   ///< 渲染索引 -> 全量索引（仅降采样时使用）
    std::vector<CloudRange> m_renderRanges;///< 每个关键帧在渲染数组中的范围
    bool m_renderFullRes = true;           ///< true = 渲染索引与全量索引一致（不降采样）
    std::set<long> m_highlightIds;         ///< 当前高亮的顶点 ID 集合（重建后重新应用）

    // —— 数据范围 ——
    float m_zMin   =  std::numeric_limits<float>::max(); ///< 数据 Z 最小值
    float m_zMax   = -std::numeric_limits<float>::max(); ///< 数据 Z 最大值
    Eigen::Vector3d m_bMin;  ///< 世界包围盒最小值（用于体素边长估计）
    Eigen::Vector3d m_bMax;  ///< 世界包围盒最大值（用于体素边长估计）
    float m_pointSize = 3.0f;  ///< 点大小（像素）
    float m_opacity   = 1.0f;  ///< 透明度（1.0 不透明）

    // —— Z 轴裁剪状态（着色器级别，变化时无需重建 VBO） ——
    bool  m_zClipping = false;           ///< Z 裁剪开关
    float m_zClipMin  = -10.0f;          ///< Z 裁剪最小值
    float m_zClipMax  = 10.0f;           ///< Z 裁剪最大值
    bool  m_clipRangeInitialized = false; ///< 是否已初始化裁剪范围

    // —— 高程颜色范围（CPU 重新着色） ——
    float m_colorZMin = 0.0f;   ///< 颜色映射 Z 最小值
    float m_colorZMax = 1.0f;   ///< 颜色映射 Z 最大值
    bool  m_useAutoColorRange = true; ///< 是否使用自动颜色范围
};
