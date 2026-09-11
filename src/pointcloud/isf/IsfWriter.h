// ============================================================================
// IsfWriter.h
// .isf / .isf.idx 打包器 —— 把地图的关键帧点云写成一帧局部系的 LOD 存储
//
// 一次写盘、永不重写：文件里只有帧局部系坐标与原始（里程计）位姿，
// 优化位姿不进文件。因此 g2o 优化之后完全不需要重新打包。
//
// 输出两块：
//   cloud.isf      —— 文件头 + 全局粗预览块 + 逐帧 L0 顶点块与 LOD 索引块
//   cloud.isf.idx  —— 索引文件头 + FrameRecord[] + PageRecord[]（常驻内存，几 MB）
// ============================================================================

#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <string>

#include "pointcloud/isf/IsfFormat.h"

namespace hdl_graph_slam {

class InteractiveGraph;

namespace isf {

/**
 * @brief 打包选项
 */
struct WriteOptions {
    uint32_t framesPerPage = kDefaultFramesPerPage;  ///< 每页帧数
    uint32_t chunkFrames   = kDefaultChunkFrames;    ///< 每 chunk 帧数
    uint32_t lodLevels     = kMaxLodLevels;          ///< 生成级别数（含 L0）
    uint64_t previewTarget = kPreviewTargetPoints;   ///< 预览块目标点数
    bool     buildPreview  = true;                   ///< 是否生成全局粗预览块

    /**
     * @brief 进度回调（可选）：已处理帧数 / 总帧数
     *
     * 用 std::function 而不是 ProgressInterface，避免把数据层接口拖进来，
     * 也让 CLI 与 GUI 都能直接塞 lambda。
     */
    std::function<void(uint64_t done, uint64_t total)> onProgress;
};

/**
 * @brief 打包结果与统计
 */
struct WriteResult {
    bool        ok = false;      ///< 是否成功
    std::string error;           ///< 失败原因（ok=true 时为空）
    std::string isfPath;         ///< 输出 .isf 路径
    std::string idxPath;         ///< 输出 .isf.idx 路径

    uint64_t frameCount    = 0;  ///< 帧数
    uint64_t pageCount     = 0;  ///< 页数
    uint64_t totalPoints   = 0;  ///< 全量点数
    uint64_t previewPoints = 0;  ///< 预览块点数
    uint64_t isfBytes      = 0;  ///< .isf 文件字节数
    uint64_t idxBytes      = 0;  ///< .isf.idx 文件字节数

    // —— 各级点数（[0] = L0 全量） ——
    uint64_t lodPoints[kMaxLodLevels] = {0, 0, 0, 0};

    // —— 耗时（毫秒） ——
    double totalMs   = 0.0;  ///< 端到端
    double lodMs     = 0.0;  ///< LOD 抽稀累计
    double frameMs   = 0.0;  ///< 写帧数据累计
    double previewMs = 0.0;  ///< 预览块累计
};

/**
 * @brief 把地图打包为 .isf / .isf.idx
 *
 * 线程安全：只在开始时于 optimization_mutex 保护下做一次毫秒级快照
 * （帧 ID / 位姿 / 点云 ConstPtr），之后无锁顺序写盘。因此可以放在
 * 后台线程执行，不阻塞优化。
 *
 * 内存：逐帧流式写入，峰值内存 ≈ 单帧点数 × (12 B 临时坐标 + 索引)，
 * 与总点数无关（1 亿点地图不会因此爆内存）。
 *
 * @param graph   已加载的地图
 * @param isfPath 输出 .isf 路径（.isf.idx 由它加后缀得到）
 * @param opt     打包选项
 * @return 结果与统计；失败时 ok=false 并给出 error
 */
WriteResult write(const std::shared_ptr<InteractiveGraph>& graph,
                  const std::string& isfPath, const WriteOptions& opt = WriteOptions());

}  // namespace isf
}  // namespace hdl_graph_slam
