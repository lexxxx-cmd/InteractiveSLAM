// ============================================================================
// IsfReader.cpp
// .isf / .isf.idx 读取器实现
//
// mmap 策略：Windows 用 CreateFileMappingW + MapViewOfFile，POSIX 用 open+mmap。
// 映射失败**不视为打开失败** —— 索引照常可用，坐标与索引改走 readFrame* 的
// 顺序读兜底路径。这样 CLI 校验器在受限环境（例如沙箱）里也能工作。
//
// windows.h 只在本翻译单元引入。
// ============================================================================

#include "pointcloud/isf/IsfReader.h"

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <vector>

#ifdef _WIN32
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#else
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
#endif

namespace hdl_graph_slam {
namespace isf {

namespace {

#ifdef _WIN32
/** @brief UTF-8（std::string，Qt 侧约定）→ UTF-16，供宽字符 Win32 API 使用 */
std::wstring toWide(const std::string& s) {
    if (s.empty()) return std::wstring();
    const int n = MultiByteToWideChar(CP_UTF8, 0, s.c_str(),
                                      static_cast<int>(s.size()), nullptr, 0);
    if (n <= 0) return std::wstring(s.begin(), s.end());  // 退化：按字节扩宽
    std::wstring w(static_cast<size_t>(n), L'\0');
    MultiByteToWideChar(CP_UTF8, 0, s.c_str(), static_cast<int>(s.size()), &w[0], n);
    return w;
}
#endif

/** @brief 读 POD 头（读满 sizeof(T) 才算成功） */
template <typename T>
bool readPod(std::ifstream& is, T& out) {
    is.read(reinterpret_cast<char*>(&out), static_cast<std::streamsize>(sizeof(T)));
    return static_cast<size_t>(is.gcount()) == sizeof(T);
}

}  // namespace

IsfReader::~IsfReader() { close(); }

void IsfReader::close() {
    delete static_cast<std::ifstream*>(m_stream);
    m_stream = nullptr;

    if (m_mapped) {
#ifdef _WIN32
        UnmapViewOfFile(m_mapped);
        if (m_mapHandle)  CloseHandle(static_cast<HANDLE>(m_mapHandle));
        if (m_fileHandle) CloseHandle(static_cast<HANDLE>(m_fileHandle));
        m_mapHandle = m_fileHandle = nullptr;
#else
        ::munmap(const_cast<uint8_t*>(m_mapped), static_cast<size_t>(m_mapSize));
        if (m_fd >= 0) { ::close(m_fd); m_fd = -1; }
#endif
        m_mapped = nullptr;
    }
    m_mapSize = 0;
    m_loaded  = false;
    m_frames.clear();
    m_pages.clear();
}

bool IsfReader::loadIndex(const std::string& idxPath, std::string* error) {
    std::ifstream is(idxPath, std::ios::binary);
    if (!is) {
        if (error) *error = "无法打开索引文件：" + idxPath;
        return false;
    }
    if (!readPod(is, m_indexHeader)) {
        if (error) *error = "索引文件头读取失败：" + idxPath;
        return false;
    }
    if (std::memcmp(m_indexHeader.magic, kIndexMagic, sizeof(kIndexMagic)) != 0) {
        if (error) *error = "索引文件魔数不匹配（不是 .isf.idx？）";
        return false;
    }
    if (m_indexHeader.version != kVersion) {
        if (error) *error = "索引文件版本不支持：" +
                           std::to_string(m_indexHeader.version);
        return false;
    }
    if (m_indexHeader.frameCount == 0) {
        if (error) *error = "索引文件帧数为 0";
        return false;
    }

    is.seekg(static_cast<std::streamoff>(m_indexHeader.headerBytes), std::ios::beg);
    m_frames.resize(static_cast<size_t>(m_indexHeader.frameCount));
    is.read(reinterpret_cast<char*>(m_frames.data()),
            static_cast<std::streamsize>(m_frames.size() * sizeof(FrameRecord)));
    if (!is) {
        if (error) *error = "帧表读取不完整（文件被截断？）";
        return false;
    }

    m_pages.resize(static_cast<size_t>(m_indexHeader.pageCount));
    if (!m_pages.empty()) {
        is.read(reinterpret_cast<char*>(m_pages.data()),
                static_cast<std::streamsize>(m_pages.size() * sizeof(PageRecord)));
        if (!is) {
            if (error) *error = "页表读取不完整（文件被截断？）";
            return false;
        }
    }
    return true;
}

bool IsfReader::open(const std::string& isfPath, std::string* error) {
    close();
    m_isfPath = isfPath;
    m_idxPath = isfPath + ".idx";

    if (!loadIndex(m_idxPath, error)) return false;

    // —— 读 .isf 文件头 ——
    {
        std::ifstream is(isfPath, std::ios::binary);
        if (!is) {
            if (error) *error = "无法打开 .isf：" + isfPath;
            return false;
        }
        if (!readPod(is, m_header)) {
            if (error) *error = ".isf 文件头读取失败";
            return false;
        }
        if (std::memcmp(m_header.magic, kMagic, sizeof(kMagic)) != 0) {
            if (error) *error = ".isf 魔数不匹配（不是 .isf 文件？）";
            return false;
        }
        if (m_header.version != kVersion) {
            if (error) *error = ".isf 版本不支持：" + std::to_string(m_header.version);
            return false;
        }
        if (m_header.frameCount != m_indexHeader.frameCount) {
            if (error) *error = ".isf 与 .isf.idx 的帧数不一致（文件不配对）";
            return false;
        }
    }

    // —— 顺序读兜底流（始终打开） ——
    auto* stream = new std::ifstream(isfPath, std::ios::binary);
    if (!*stream) {
        delete stream;
        if (error) *error = "无法打开 .isf 读取流";
        return false;
    }
    m_stream = stream;

    // —— 尽力建立只读映射；失败不视为打开失败 ——
#ifdef _WIN32
    {
        const std::wstring wpath = toWide(isfPath);
        HANDLE hFile = CreateFileW(wpath.c_str(), GENERIC_READ, FILE_SHARE_READ,
                                   nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL,
                                   nullptr);
        if (hFile != INVALID_HANDLE_VALUE) {
            HANDLE hMap = CreateFileMappingW(hFile, nullptr, PAGE_READONLY, 0, 0, nullptr);
            if (hMap) {
                void* base = MapViewOfFile(hMap, FILE_MAP_READ, 0, 0, 0);
                if (base) {
                    m_mapped       = static_cast<const uint8_t*>(base);
                    m_mapSize      = m_header.headerBytes + m_header.dataBytes;
                    m_fileHandle   = hFile;
                    m_mapHandle    = hMap;
                } else {
                    CloseHandle(hMap);
                    CloseHandle(hFile);
                }
            } else {
                CloseHandle(hFile);
            }
        }
    }
#else
    {
        const int fd = ::open(isfPath.c_str(), O_RDONLY);
        if (fd >= 0) {
            struct stat st{};
            if (::fstat(fd, &st) == 0 && st.st_size > 0) {
                void* base = ::mmap(nullptr, static_cast<size_t>(st.st_size),
                                    PROT_READ, MAP_PRIVATE, fd, 0);
                if (base != MAP_FAILED) {
                    m_mapped  = static_cast<const uint8_t*>(base);
                    m_mapSize = static_cast<uint64_t>(st.st_size);
                    m_fd      = fd;
                } else {
                    ::close(fd);
                }
            } else {
                ::close(fd);
            }
        }
    }
#endif

    m_loaded = true;
    return true;
}

int64_t IsfReader::indexOfFrame(long frameId) const {
    // 帧表按 frameId 升序（打包时排序保证），二分查找
    const auto it = std::lower_bound(
        m_frames.begin(), m_frames.end(), frameId,
        [](const FrameRecord& rec, long id) { return rec.frameId < id; });
    if (it == m_frames.end() || it->frameId != frameId) return -1;
    return static_cast<int64_t>(it - m_frames.begin());
}

const FrameRecord* IsfReader::findFrame(long frameId) const {
    const int64_t i = indexOfFrame(frameId);
    return (i < 0) ? nullptr : &m_frames[static_cast<size_t>(i)];
}

const float* IsfReader::framePositions(size_t frameIndex) const {
    if (!m_mapped || frameIndex >= m_frames.size()) return nullptr;
    const FrameRecord& rec = m_frames[frameIndex];
    const uint64_t need = static_cast<uint64_t>(rec.pointCount) * kL0Stride;
    if (rec.l0Offset + need > m_mapSize) return nullptr;  // 越界保护
    return reinterpret_cast<const float*>(m_mapped + rec.l0Offset);
}

const uint32_t* IsfReader::frameLodIndices(size_t frameIndex, uint32_t level,
                                           uint32_t& outCount) const {
    outCount = 0;
    if (!m_mapped || frameIndex >= m_frames.size()) return nullptr;
    const FrameRecord& rec = m_frames[frameIndex];
    if (level == 0 || level >= rec.lodCount || level >= kMaxLodLevels) return nullptr;
    const uint64_t need = static_cast<uint64_t>(rec.lodCounts[level]) * 4ull;
    if (rec.lodOffset[level] + need > m_mapSize) return nullptr;
    outCount = rec.lodCounts[level];
    return reinterpret_cast<const uint32_t*>(m_mapped + rec.lodOffset[level]);
}

const PreviewPoint* IsfReader::previewPoints() const {
    if (!m_mapped || m_header.previewPoints == 0) return nullptr;
    const uint64_t need = m_header.previewPoints * kPreviewStride;
    if (m_header.previewOffset + need > m_mapSize) return nullptr;
    return reinterpret_cast<const PreviewPoint*>(m_mapped + m_header.previewOffset);
}

bool IsfReader::readFramePositions(size_t frameIndex, std::vector<float>& out) const {
    if (frameIndex >= m_frames.size()) return false;
    const FrameRecord& rec = m_frames[frameIndex];
    const uint64_t bytes = static_cast<uint64_t>(rec.pointCount) * kL0Stride;

    // 优先走 mmap（无需 IO）
    if (const float* p = framePositions(frameIndex)) {
        out.assign(p, p + static_cast<size_t>(rec.pointCount) * 3);
        return true;
    }

    auto* is = static_cast<std::ifstream*>(m_stream);
    if (!is) return false;
    out.resize(static_cast<size_t>(rec.pointCount) * 3);
    is->clear();
    is->seekg(static_cast<std::streamoff>(rec.l0Offset), std::ios::beg);
    if (!*is) return false;
    if (bytes > 0) {
        is->read(reinterpret_cast<char*>(out.data()),
                 static_cast<std::streamsize>(bytes));
    }
    return static_cast<uint64_t>(is->gcount()) == bytes;
}

bool IsfReader::readFrameLodIndices(size_t frameIndex, uint32_t level,
                                    std::vector<uint32_t>& out,
                                    uint32_t& outCount) const {
    outCount = 0;
    out.clear();
    if (frameIndex >= m_frames.size()) return false;
    const FrameRecord& rec = m_frames[frameIndex];
    if (level == 0 || level >= rec.lodCount || level >= kMaxLodLevels) {
        return level == 0;  // level 0 = 全量，无索引数组，视为成功且 outCount=0
    }
    const uint32_t count = rec.lodCounts[level];

    if (const uint32_t* p = frameLodIndices(frameIndex, level, outCount)) {
        out.assign(p, p + count);
        return true;
    }

    auto* is = static_cast<std::ifstream*>(m_stream);
    if (!is) return false;
    out.resize(count);
    is->clear();
    is->seekg(static_cast<std::streamoff>(rec.lodOffset[level]), std::ios::beg);
    if (!*is) return false;
    if (count > 0) {
        is->read(reinterpret_cast<char*>(out.data()),
                 static_cast<std::streamsize>(static_cast<uint64_t>(count) * 4ull));
    }
    if (static_cast<uint64_t>(is->gcount()) != static_cast<uint64_t>(count) * 4ull) {
        return false;
    }
    outCount = count;
    return true;
}

}  // namespace isf
}  // namespace hdl_graph_slam
