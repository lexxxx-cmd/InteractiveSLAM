// ============================================================================
// CloudPerfLog.cpp
// CloudPerfLog / GpuMemory 的实现
//
// windows.h 只在本翻译单元引入（并收窄 WIN32_LEAN_AND_MEAN + NOMINMAX），
// 避免污染其它包含 CloudPerfLog.h 的代码。
// ============================================================================

#include "visualizers/CloudPerfLog.h"

#include <QCoreApplication>
#include <QDateTime>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QTextStream>
#include <QtGlobal>

#include <mutex>

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>

namespace hdl_graph_slam {

// ============================================================================
// CloudPerfLog
// ============================================================================

namespace {

/** @brief 日志状态：函数局部静态，magic static 保证线程安全的惰性构造 */
struct LogState {
    QFile      file;
    QString    path;
    bool       openAttempted = false;
    std::mutex mutex;
};

LogState& logState() {
    static LogState s;
    return s;
}

/** @brief 计算日志路径（幂等；applicationDirPath 为空时退化为当前目录） */
QString ensureLogPath(LogState& s) {
    if (s.path.isEmpty()) {
        const QString dir = QCoreApplication::applicationDirPath().isEmpty()
                                ? QDir::currentPath()
                                : QCoreApplication::applicationDirPath();
        s.path = QDir(dir).filePath(QStringLiteral("cloud_perf.log"));
    }
    return s.path;
}

/** @brief 首次写入时打开文件并打印运行横幅（调用方须已持锁） */
void openIfNeeded(LogState& s) {
    ensureLogPath(s);
    if (s.openAttempted) return;
    s.openAttempted = true;

    s.file.setFileName(s.path);
    // Append：保留历次运行的基线便于纵向比较，每次运行写一条横幅分隔
    if (s.file.open(QIODevice::WriteOnly | QIODevice::Append | QIODevice::Text)) {
        QTextStream ts(&s.file);
        ts << "\n"
           << "==============================================================\n"
           << " cloud_perf.log  session start: "
           << QDateTime::currentDateTime().toString("yyyy-MM-dd HH:mm:ss") << "\n"
           << " exe: " << QCoreApplication::applicationFilePath() << "\n"
           << " " << GpuMemory::summaryLine() << "\n"
           << "==============================================================\n";
        ts.flush();
    }
}

}  // namespace

CloudPerfLog& CloudPerfLog::instance() {
    static CloudPerfLog s;
    return s;
}

QString CloudPerfLog::logFilePath() const {
    LogState& s = logState();
    std::lock_guard<std::mutex> lock(s.mutex);
    ensureLogPath(s);
    return s.path;
}

void CloudPerfLog::write(const QString& line) {
    const QString stamped =
        QStringLiteral("[%1] %2")
            .arg(QDateTime::currentDateTime().toString("HH:mm:ss.zzz"), line);

    // 同时走 qInfo：在 Qt Creator / 附加调试器时可直接看到
    qInfo().noquote() << stamped;

    LogState& s = logState();
    std::lock_guard<std::mutex> lock(s.mutex);
    openIfNeeded(s);
    if (s.file.isOpen()) {
        QTextStream ts(&s.file);
        ts << stamped << '\n';
        ts.flush();
    }
}

// ============================================================================
// GpuMemory —— NVML 动态加载
// ============================================================================

namespace {

// NVML 的最小必要声明（只用到 total/free/used 三个字段，布局与官方头一致）
using nvmlDevice_t = void*;
struct nvmlMemory_t {
    unsigned long long total;
    unsigned long long free;
    unsigned long long used;
};

using nvmlInit_t                   = int (*)(void);
using nvmlDeviceGetHandleByIndex_t = int (*)(unsigned int, nvmlDevice_t*);
using nvmlDeviceGetMemoryInfo_t    = int (*)(nvmlDevice_t, nvmlMemory_t*);
using nvmlDeviceGetName_t          = int (*)(nvmlDevice_t, char*, unsigned int);

/** @brief NVML 加载状态（进程内一次；NVML_SUCCESS == 0） */
struct NvmlState {
    bool    attempted = false;
    bool    available = false;
    QString failReason;
    nvmlDevice_t device = nullptr;
    QString deviceName;
    nvmlDeviceGetMemoryInfo_t getMemoryInfo = nullptr;
    std::mutex mutex;
};

NvmlState& nvmlState() {
    static NvmlState s;
    return s;
}

/** @brief 取符号：新版优先，回退到无 _v2 的旧名 */
template <typename Fn>
Fn resolveSymbol(HMODULE mod, const char* v2Name, const char* plainName) {
    FARPROC p = GetProcAddress(mod, v2Name);
    if (!p && plainName) p = GetProcAddress(mod, plainName);
    return reinterpret_cast<Fn>(p);
}

/** @brief 加载 nvml.dll 并取设备句柄（调用方须已持锁） */
void initNvmlLocked(NvmlState& s) {
    s.attempted = true;

    HMODULE mod = LoadLibraryW(L"nvml.dll");
    if (!mod) {
        s.failReason = QStringLiteral("nvml.dll 加载失败 (err=%1)").arg(GetLastError());
        return;
    }

    auto init      = resolveSymbol<nvmlInit_t>(mod, "nvmlInit_v2", "nvmlInit");
    auto getHandle = resolveSymbol<nvmlDeviceGetHandleByIndex_t>(
        mod, "nvmlDeviceGetHandleByIndex_v2", "nvmlDeviceGetHandleByIndex");
    s.getMemoryInfo =
        resolveSymbol<nvmlDeviceGetMemoryInfo_t>(mod, "nvmlDeviceGetMemoryInfo", nullptr);
    auto getName = resolveSymbol<nvmlDeviceGetName_t>(mod, "nvmlDeviceGetName", nullptr);

    if (!init || !getHandle || !s.getMemoryInfo) {
        s.failReason = QStringLiteral("nvml.dll 缺少所需符号");
        return;
    }
    if (init() != 0) {
        s.failReason = QStringLiteral("nvmlInit 失败");
        return;
    }
    if (getHandle(0, &s.device) != 0 || s.device == nullptr) {
        s.failReason = QStringLiteral("nvmlDeviceGetHandleByIndex(0) 失败");
        return;
    }
    if (getName) {
        char buf[96] = {0};
        if (getName(s.device, buf, sizeof(buf)) == 0) {
            s.deviceName = QString::fromLatin1(buf).trimmed();
        }
    }
    s.available = true;
}

constexpr unsigned long long kBytesPerMiB = 1024ull * 1024ull;

}  // namespace

GpuMemory::Info GpuMemory::query() {
    Info info;
    NvmlState& s = nvmlState();

    std::lock_guard<std::mutex> lock(s.mutex);
    if (!s.attempted) initNvmlLocked(s);
    if (!s.available) return info;  // valid == false

    nvmlMemory_t mem{};
    if (s.getMemoryInfo(s.device, &mem) != 0) return info;

    info.valid      = true;
    info.deviceName = s.deviceName;
    info.totalMiB   = mem.total / kBytesPerMiB;
    info.usedMiB    = mem.used / kBytesPerMiB;
    info.freeMiB    = mem.free / kBytesPerMiB;
    return info;
}

QString GpuMemory::summaryLine() {
    const Info info = query();
    if (!info.valid) {
        NvmlState& s = nvmlState();
        const QString why =
            s.failReason.isEmpty() ? QStringLiteral("不可用") : s.failReason;
        return QStringLiteral("VRAM: 查询不可用（%1）").arg(why);
    }
    return QStringLiteral("VRAM: %1  used=%2 MiB / total=%3 MiB  free=%4 MiB")
        .arg(info.deviceName.isEmpty() ? QStringLiteral("(unknown)") : info.deviceName)
        .arg(info.usedMiB)
        .arg(info.totalMiB)
        .arg(info.freeMiB);
}

}  // namespace hdl_graph_slam
