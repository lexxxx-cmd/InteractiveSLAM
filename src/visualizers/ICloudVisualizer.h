// ============================================================================
// ICloudVisualizer.h
// Control/query interface shared by the two point-cloud back-ends
//
// Why this exists (design doc §890): the doc's Phase 2 change list says to keep
// KeyframePointCloudVisualizer's public method signatures and swap its internals
// so that ViewportWidget / PlaybackPanel / MainWindow need minimal edits.
// Doing that literally means rewriting a ~1000-line class that is currently
// working. Instead this small interface captures the surface GraphSceneVisualizer
// actually delegates to, and lets the old and the new back-end coexist:
//
//   KeyframePointCloudVisualizer  -> old path (world-space vertices, CPU colours)
//   ChunkCloudVisualizer          -> new path (frame-local vertices + pose texture
//                                    + Turbo LUT + shared geometry for both layers)
//
// The compiler then enforces that both satisfy the same surface, instead of us
// hand-matching ~35 method signatures.
//
// Deliberately NOT in this interface:
//   * commit entry points -- the two back-ends commit different result types
//     (PointCloudBuildResult vs ChunkBuildResult), and ViewportWidget is the
//     component that orchestrates builds, so it calls the matching concrete
//     method. See the note in the commit-plumbing section below.
//
// Optional members have default implementations so a back-end only implements
// what it actually supports. The defaults are deliberately the *safe* answers
// (e.g. "no highlight", "nothing pending"), never the optimistic ones.
// ============================================================================

#pragma once

#include <Eigen/Geometry>
#include <osg/Node>

#include <cstdint>
#include <set>

namespace hdl_graph_slam {

/**
 * @brief Back-end-agnostic control and query surface for the point cloud
 */
class ICloudVisualizer {
public:
    virtual ~ICloudVisualizer() = default;

    // ------------------------------------------------------------------
    // Scene graph
    // ------------------------------------------------------------------

    /**
     * @brief Root node of the optimized (pose-optimized) layer
     *
     * Returned as osg::Node* rather than osg::Geode* on purpose: the old path
     * hands out a Geode holding per-chunk geometries, while the new path hands
     * out a Group whose StateSet carries the pose texture and colour semantics
     * (design doc §3.4: layer state lives on the parent Group, not on the
     * geometry, so that both layers can share the same geometry objects).
     */
    virtual osg::Node* cloudNode() const = 0;

    /** @brief Root node of the original (frozen odometry) layer; nullptr if absent */
    virtual osg::Node* originalNode() const = 0;

    // ------------------------------------------------------------------
    // Rendering controls
    // ------------------------------------------------------------------

    virtual void  setPointSize(float size) = 0;
    virtual void  setOpacity(float opacity) = 0;

    virtual void  setZClipping(bool enabled) = 0;
    virtual void  setZClipRange(float minZ, float maxZ) = 0;
    virtual bool  isZClipping() const = 0;
    virtual float getZClipMin() const = 0;
    virtual float getZClipMax() const = 0;

    virtual void  setColorZRange(float minZ, float maxZ) = 0;
    virtual void  setAutoColorRange(bool autoRange) = 0;
    virtual bool  isAutoColorRange() const = 0;

    virtual float getDataZMin() const = 0;
    virtual float getDataZMax() const = 0;
    virtual float getColorZMin() const = 0;
    virtual float getColorZMax() const = 0;

    // ------------------------------------------------------------------
    // LOD
    // ------------------------------------------------------------------

    /** @brief Switch LOD level (new path: swaps the primitive set, no vertex upload) */
    virtual void setLodLevel(int level) = 0;
    virtual int  currentLodLevel() const = 0;
    /** @brief Number of levels including level 0 */
    virtual int  lodLevelCount() const { return 1; }

    // ------------------------------------------------------------------
    // Progressive upload
    // ------------------------------------------------------------------

    /**
     * @brief Advance the progressive upload by one chunk (no-op = fully uploaded)
     *
     * The old path reveals chunk geometries one frame at a time to avoid a single
     * huge VBO upload stalling the GPU. The new path has no such staging yet, so
     * it reports "nothing pending" and this default is correct for it.
     */
    virtual void advanceChunkUpload() {}
    virtual bool chunkUploadPending() const { return false; }

    // ------------------------------------------------------------------
    // Original (frozen) layer
    // ------------------------------------------------------------------

    virtual void setOdomLayerVisible(bool visible) { (void)visible; }
    virtual bool odomLayerVisible() const { return false; }
    virtual bool odomLayerReady() const { return false; }
    /**
     * @brief Content signature of the committed original layer (0 = never built)
     *
     * Used by ViewportWidget to decide whether the frozen layer can be reused
     * instead of rebuilt.
     */
    virtual uint64_t committedOdomSignature() const { return 0; }
    /** @brief Update the original layer's opacity in place (no rebuild) */
    virtual void setOriginalLayerOpacity(float opacity) { (void)opacity; }

    // ------------------------------------------------------------------
    // Frame-metadata bounds
    // ------------------------------------------------------------------

    /**
     * @brief Centre/radius derived from frame metadata, not from the live graph
     *
     * Deliberately separate from osg's own bound: under paging (Phase 6) the
     * nodes for evicted chunks leave the graph, so scene->getBound() drifts.
     * These stay correct regardless of what is resident.
     */
    virtual Eigen::Vector3d boundsCenter() const = 0;
    virtual double boundsRadius() const = 0;
    /** @brief Whether the Z clip range has been initialised from the data yet */
    virtual bool isClipRangeInitialized() const = 0;

    // ------------------------------------------------------------------
    // Highlight
    // ------------------------------------------------------------------

    // The new path moves highlighting into the shader (design doc §4.8 / Phase 5),
    // so these are optional. Defaults report "nothing highlighted", which makes a
    // back-end that has not implemented them behave as if no highlight is active
    // rather than pretending one is.
    virtual void clearHighlight() {}
    virtual void recolorHighlight(const std::set<long>& highlightIds) { (void)highlightIds; }
    virtual void recolorHighlightPrefix(long endFrameId, const std::set<long>& extraIds = {}) {
        (void)endFrameId;
        (void)extraIds;
    }
    virtual bool prefixHighlightActive() const { return false; }
    virtual bool highlightActive() const { return false; }
    virtual long highlightPrefixEnd() const { return -1; }

    // ------------------------------------------------------------------
    // Lifecycle / stats
    // ------------------------------------------------------------------

    /** @brief Drop all point data and detach the nodes */
    virtual void clear() = 0;
    /** @brief Number of vertices currently held (layers share one copy) */
    virtual int  pointCount() const = 0;
};

}  // namespace hdl_graph_slam
