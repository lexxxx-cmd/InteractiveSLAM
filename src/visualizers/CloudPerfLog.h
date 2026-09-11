// ============================================================================
// CloudPerfLog.h
// 点云性能日志（Phase 0 基线测量基础设施）
//
// 背景：本程序是 WIN32 GUI 可执行文件，没有 qInstallMessageHandler，也没有
//       任何文件日志。qDebug/qInfo 在没有附加调试器时基本不可见，而 Phase 0
//       要测的基线数字（双份点云实际上限、优化后重建耗时、显存峰值）必须
//       能被直接读到。因此这里提供一个最小的落地通道：
//
//   1. CloudPerfLog::write()  —— 带时间戳写入 <exe目录>/cloud_perf.log，
//                                同时转发给 qInfo()，方便 IDE 里直接看；
//   2. GpuMemory::query()     —— 通过动态加载 nvml.dll 读取显存占用。
//
// 为什么用 NVML 而不是 GL 扩展：GL_NVX_gpu_memory_info 需要当前线程持有 GL
// 上下文，而点云构建/日志发生在后台线程与主线程的多处位置；NVML 与上下文
// 无关，可在任意线程调用。动态加载（LoadLibrary）也避免了给项目引入链接依赖。
//
// 本文件只声明，实现在 CloudPerfLog.cpp（避免 windows.h 污染其他翻译单元）。
// ============================================================================

#pragma once

#include <QString>

namespace hdl_graph_slam {

/**
 * @brief 进程级性能日志（追加写，线程安全）
 *
 * 首次写入时在日志开头打印一条分隔横幅，包含可执行文件路径与显存信息，
 * 便于区分多次运行的基线数据。所有状态由实现文件内的函数局部静态持有，
 * 因此本类本身无成员、无生命周期管理负担。
 */
class CloudPerfLog {
public:
    /** @brief 单例（进程内唯一） */
    static CloudPerfLog& instance();

    /**
     * @brief 写一行日志（自动加时间戳）
     * @param line 日志正文（不含换行）
     */
    void write(const QString& line);

    /**
     * @brief 日志文件绝对路径（<exe目录>/cloud_perf.log）
     *
     * 文件尚未创建时也返回预期路径，便于 UI 直接提示用户位置。
     */
    QString logFilePath() const;

private:
    CloudPerfLog() = default;
    CloudPerfLog(const CloudPerfLog&) = delete;
    CloudPerfLog& operator=(const CloudPerfLog&) = delete;
};

/**
 * @brief 显存占用查询（NVIDIA NVML，动态加载）
 *
 * 查询失败（非 NVIDIA 显卡 / 驱动无 nvml.dll / 符号缺失）时 valid = false，
 * 调用方应据此跳过显存相关输出，而不是报错。
 */
class GpuMemory {
public:
    struct Info {
        bool valid = false;               ///< 本次查询是否成功
        QString deviceName;               ///< 显卡名（查询成功时填充）
        unsigned long long totalMiB = 0;  ///< 显存总量
        unsigned long long usedMiB  = 0;  ///< 已占用
        unsigned long long freeMiB  = 0;  ///< 可用
    };

    /**
     * @brief 查询当前显存占用
     *
     * 首次调用时动态加载 nvml.dll 并缓存设备句柄；之后每次调用只做一次
     * nvmlDeviceGetMemoryInfo。线程安全（内部加锁）。
     */
    static Info query();

    /** @brief 返回一行可直接写日志的显存摘要（查询失败时说明原因） */
    static QString summaryLine();
};

}  // namespace hdl_graph_slam
