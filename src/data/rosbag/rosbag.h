// MIT License

// Copyright (c) 2024 Saurabh Gupta

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// ============================================================================
// rosbag.h —— 精简版 ROS1 bag 读取器（从 SCPGO 的 ros1bag_reader 移植）
//
// 改动（相对原版）：
//   - 移除 messages/messages.h 依赖（不再提供 saveAsPLY 等消息级导出）
//   - 保留核心：bag 索引解析 + getRawPayloads()（按 topic 多线程读取原始
//     消息 payload，返回 (时间戳ns, 字节) 列表）
//   - 仅支持未压缩的 ROS1 bag V2.0（chunk 压缩类型只记录不解压）
// ============================================================================
#pragma once

#include <atomic>
#include <cstdint>
#include <fstream>
#include <map>
#include <memory>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

struct ChunkInfo {
    int64_t chunk_pos{};
    int64_t start_time{};
    int64_t end_time{};
    int64_t num_msgs{};
    int conn_count{};
};

struct MesssageDataInfo {
    int64_t buffer_offset{};
    int conn_id{};
    int64_t time{};
    int data_len{};  // 记录数据长度，便于后续直接读取
};

struct ConnectionInfo {
    std::string msg_type;
    std::string topic_name;
    int num_msgs{};
    std::vector<MesssageDataInfo> messages_info;
};

class Rosbag {
    using Connection = std::map<int, ConnectionInfo>;
    using FieldValMap =
            std::map<std::string, std::pair<std::shared_ptr<char[]>, int>>;

public:
    Rosbag(const std::string &rosbag_path);
    void printInfo() const;
    void printAvailableTopics() const;
    std::vector<std::string> getAvailableTopics() const;
    std::vector<std::string> getAvailableTypes() const;

    /**
     * @brief 读取指定 topic 的全部消息原始 payload
     * @param topic_name ROS topic 名（如 "/odom"）
     * @return (消息时间戳 ns, 原始消息字节) 列表；topic 不存在返回空
     *
     * 多线程并行读取（每线程独立文件句柄），结果按消息顺序返回。
     * 仅支持未压缩 chunk。
     */
    std::vector<std::pair<int64_t, std::vector<uint8_t>>> getRawPayloads(const std::string& topic_name);

private:
    void readString(std::string &str, int n_bytes);
    std::tuple<std::string, std::string> readStringField(int &header_len);
    FieldValMap readRecordHeader();
    void readBagHeaderRecord();
    void readChunkRecord(int64_t num_of_msgs);
    void readConnectionRecord();
    void readMessageDataRecord();
    void readChunkInfoRecord();

    void readData();
    void readBagInfo();

private:
    std::string rosbag_path_;
    std::ifstream rosbag_;

    std::string version_;

    int64_t index_pos_{};
    int num_unique_conns_{};
    int num_chunk_records_{};

    FieldValMap fields_;

    Connection connections_;
    std::map<std::string, int> topic_to_conn_id_;
    std::vector<std::string> chunk_compression_types_;
    std::vector<ChunkInfo> chunk_info_records_;

    // Guard against redundant readData() calls when multiple topics are read
    std::atomic<bool> m_dataLoaded{false};
};
