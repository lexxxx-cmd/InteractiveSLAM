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
// rosbag.cpp —— 精简版 ROS1 bag 读取器实现（从 SCPGO 的 ros1bag_reader 移植）
//
// 改动（相对原版）：移除 messages/messages.h 依赖与 saveAsPLY 相关实现，
// 保留 bag 索引解析与 getRawPayloads() 多线程原始消息读取。
// ============================================================================
#include "data/rosbag/rosbag.h"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <memory>
#include <numeric>
#include <string>
#include <thread>
#include <tuple>
#include <utility>

inline int64_t parseRosTime(const char* ptr) {
    uint32_t secs = *reinterpret_cast<const uint32_t*>(ptr);
    uint32_t nsecs = *reinterpret_cast<const uint32_t*>(ptr + 4);
    return static_cast<int64_t>(secs) * 1000000000LL + nsecs;
}

using FieldValMap =
        std::map<std::string, std::pair<std::shared_ptr<char[]>, int>>;

void Rosbag::readString(std::string &str, const int n_bytes) {
    std::unique_ptr<char[]> const buffer(new char[n_bytes + 1]);
    buffer.get()[n_bytes] = '\0';
    rosbag_.read(buffer.get(), n_bytes);
    str = buffer.get();
}

std::tuple<std::string, std::string> Rosbag::readStringField(int &header_len) {
    int field_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&field_len), 4);
    std::string field_name;
    std::getline(rosbag_, field_name, '=');
    std::string field_val;
    this->readString(field_val,
                     field_len - static_cast<int>(field_name.size()) - 1);
    header_len -= (field_len + 4);
    return {field_name, field_val};
}

FieldValMap Rosbag::readRecordHeader() {
    int header_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&header_len), 4);

    int field_len = 0;
    std::string field_name;
    FieldValMap fields;

    while (header_len != 0) {
        rosbag_.read(reinterpret_cast<char *>(&field_len), 4);
        std::getline(rosbag_, field_name, '=');
        const int field_val_nbytes =
                field_len - 1 - static_cast<int>(field_name.size());

        std::shared_ptr<char[]> buffer(new char[field_val_nbytes]);
        rosbag_.read(buffer.get(), field_val_nbytes);
        fields.insert({{field_name, std::make_pair(buffer, field_val_nbytes)}});
        header_len -= (4 + field_len);
    }
    return fields;
}

Rosbag::Rosbag(const std::string &rosbag_path)
    : rosbag_path_(rosbag_path),
      rosbag_(rosbag_path, std::ios_base::in | std::ios_base::binary) {
    if (!rosbag_) {
        std::cerr << "Invalid path to the bag file" << std::endl;
        exit(EXIT_FAILURE);
    }

    std::getline(rosbag_, version_);
    if (version_.find("V2.0") != std::string::npos) {
        version_ = "2.0";
    } else {
        std::cerr << "Supports only Rosbag Version 2.0" << std::endl;
        exit(EXIT_FAILURE);
    }

    this->readBagInfo();

    for (auto &[conn_id, conn_info] : connections_) {
        conn_info.messages_info.reserve(conn_info.num_msgs);
    }
}

void Rosbag::readBagHeaderRecord() {
    fields_ = this->readRecordHeader();
    index_pos_ =
            *reinterpret_cast<int64_t *>(fields_["index_pos"].first.get());
    num_unique_conns_ =
            *reinterpret_cast<int *>(fields_["conn_count"].first.get());
    num_chunk_records_ =
            *reinterpret_cast<int *>(fields_["chunk_count"].first.get());

    chunk_info_records_.reserve(num_chunk_records_);
    chunk_compression_types_.reserve(num_chunk_records_);

    // Ignore Data
    int data_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&data_len), 4);
    rosbag_.ignore(data_len);
}

void Rosbag::readBagInfo() {
    this->readBagHeaderRecord();

    rosbag_.seekg(index_pos_);
    for (int i = 0; i < num_unique_conns_; i++) {
        this->readConnectionRecord();
    }
    for (int i = 0; i < num_chunk_records_; i++) {
        this->readChunkInfoRecord();
    }
}

void Rosbag::readData() {
    // Only read chunk data once; subsequent calls are no-ops.
    if (m_dataLoaded.exchange(true)) {
        return;
    }
    std::for_each(chunk_info_records_.cbegin(), chunk_info_records_.cend(),
                  [&](const ChunkInfo chunk_info) {
                      rosbag_.seekg(chunk_info.chunk_pos);
                      this->readChunkRecord(chunk_info.num_msgs);
                  });
}

void Rosbag::readChunkRecord(int64_t num_of_msgs) {
    fields_ = this->readRecordHeader();
    int opval = int{*reinterpret_cast<uint8_t *>(fields_["op"].first.get())};
    if (opval == 5) {
        chunk_compression_types_.emplace_back(
                fields_["compression"].first.get());
        int data_len = 0;
        rosbag_.read(reinterpret_cast<char *>(&data_len), 4);
        while (num_of_msgs != 0) {
            fields_ = this->readRecordHeader();
            opval = int{
                    *reinterpret_cast<uint8_t *>(fields_["op"].first.get())};
            if (opval == 2) {
                this->readMessageDataRecord();
                num_of_msgs--;
            } else {
                int data_len = 0;
                rosbag_.read(reinterpret_cast<char *>(&data_len), 4);
                rosbag_.ignore(data_len);
            }
        }
        return;
    }
    std::cerr << "Expected to Read Chunk Record" << std::endl;
}

void Rosbag::readConnectionRecord() {
    fields_ = this->readRecordHeader();
    int const opval =
            int{*reinterpret_cast<uint8_t *>(fields_["op"].first.get())};
    if (opval == 7) {
        ConnectionInfo info{};
        info.topic_name = std::string(fields_["topic"].first.get(),
                                      fields_["topic"].second);
        auto conn_id = *reinterpret_cast<int *>(fields_["conn"].first.get());

        // Data - Connection Header
        int data_len = 0;
        rosbag_.read(reinterpret_cast<char *>(&data_len), 4);

        std::map<std::string, std::string> data_fields;
        while (data_len != 0) {
            auto [field_name, field_val] = this->readStringField(data_len);
            data_fields[field_name] = field_val;
        }

        info.msg_type = data_fields["type"];
        topic_to_conn_id_[info.topic_name] = conn_id;
        connections_[conn_id] = info;
        return;
    }
    std::cerr << "Expected to Read Connection Record" << std::endl;
}

void Rosbag::readMessageDataRecord() {
    auto conn_id = *reinterpret_cast<int *>(fields_["conn"].first.get());
    auto time = parseRosTime(fields_["time"].first.get());

    int data_len = 0;
    rosbag_.read(reinterpret_cast<char *>(&data_len), 4);
    connections_[conn_id].messages_info.emplace_back(
            MesssageDataInfo{ static_cast<int64_t>(rosbag_.tellg()), conn_id, time, data_len});
    rosbag_.ignore(data_len);
}

void Rosbag::readChunkInfoRecord() {
    fields_ = this->readRecordHeader();
    int const opval =
            int{*reinterpret_cast<uint8_t *>(fields_["op"].first.get())};
    if (opval == 6) {
        ChunkInfo info{};
        info.chunk_pos =
                *reinterpret_cast<int64_t *>(fields_["chunk_pos"].first.get());
        info.start_time = parseRosTime(fields_["start_time"].first.get());
        info.end_time = parseRosTime(fields_["end_time"].first.get());
        info.conn_count =
                *reinterpret_cast<int *>(fields_["count"].first.get());

        // Data
        int data_len = 0;
        rosbag_.read(reinterpret_cast<char *>(&data_len), 4);
        int conn = 0;
        int count = 0;
        for (int i = 0; i < info.conn_count; i++) {
            rosbag_.read(reinterpret_cast<char *>(&conn), 4);
            rosbag_.read(reinterpret_cast<char *>(&count), 4);
            connections_[conn].num_msgs += count;
            info.num_msgs += count;
        }
        chunk_info_records_.emplace_back(info);
        return;
    }
    std::cerr << "Expected to Read Chunk Info Record" << std::endl;
}

void Rosbag::printInfo() const {
    std::cout << "path:\t\t\t" << rosbag_path_ << std::endl;
    std::cout << "version:\t\t" << version_ << std::endl;

    auto start = std::min_element(chunk_info_records_.cbegin(),
                                  chunk_info_records_.cend(),
                                  [](const auto &rec1, const auto &rec2) {
                                      return rec1.start_time < rec2.start_time;
                                  });

    auto end = std::max_element(chunk_info_records_.cbegin(),
                                chunk_info_records_.cend(),
                                [](const auto &rec1, const auto &rec2) {
                                    return rec1.end_time < rec2.end_time;
                                });

    std::cout << "duration:\t\t" << end->end_time - start->start_time
              << std::endl;
    std::cout << "start:\t\t\t" << start->start_time << std::endl;
    std::cout << "end:\t\t\t" << end->end_time << std::endl;

    auto num_of_messages = std::accumulate(
            chunk_info_records_.cbegin(), chunk_info_records_.cend(), int64_t{ 0 },
            [](int64_t sum, ChunkInfo info) { return sum + info.num_msgs; });
    std::cout << "messages:\t\t" << num_of_messages << std::endl;

    std::cout << "topics:" << std::endl;
    std::for_each(
            connections_.cbegin(), connections_.cend(),
            [](const auto connection) {
                std::cout << "\t" << connection.second.topic_name << std::endl;
                std::cout << "\t\t" << connection.second.num_msgs << " msgs"
                          << std::endl;
                std::cout << "\t\tmsg type: " << connection.second.msg_type
                          << std::endl;
            });
}

void Rosbag::printAvailableTopics() const {
    std::cout << "topics:" << std::endl;
    std::for_each(connections_.cbegin(), connections_.cend(),
                  [](const auto connection) {
                      std::cout << "\t" << connection.second.topic_name
                                << std::endl;
                  });
}

std::vector<std::string> Rosbag::getAvailableTopics() const {
    std::vector<std::string> topics;
    topics.reserve(connections_.size());
    std::transform(connections_.cbegin(), connections_.cend(),
        std::back_inserter(topics),
        [](const auto& connection) { return connection.second.topic_name; });
    return topics;
}

std::vector<std::string> Rosbag::getAvailableTypes() const {
    std::vector<std::string> types;
    types.reserve(connections_.size());
    std::transform(connections_.cbegin(), connections_.cend(),
        std::back_inserter(types),
        [](const auto& connection) { return connection.second.msg_type; });
    return types;
}

std::vector<std::pair<int64_t, std::vector<uint8_t>>> Rosbag::getRawPayloads(const std::string& topic_name) {
    std::vector<std::pair<int64_t, std::vector<uint8_t>>> payloads;
    if (topic_to_conn_id_.find(topic_name) == topic_to_conn_id_.cend()) {
        return payloads;
    }
    auto conn_id = topic_to_conn_id_.at(topic_name);
    auto& conn_info = connections_.at(conn_id);

    this->readData(); // ensures metadata is loaded (idempotent)

    const auto& messages_info = conn_info.messages_info;
    const size_t n = messages_info.size();
    if (n == 0) {
        return payloads;
    }

    // Pre-allocate result slots so each thread writes to its own index.
    payloads.resize(n);

    // Use one thread per hardware core, capped to the number of messages.
    const unsigned int hw_threads = std::thread::hardware_concurrency();
    const unsigned int num_threads =
        std::max(1u, std::min(static_cast<unsigned int>(n), hw_threads));

    const size_t chunk_size = (n + num_threads - 1) / num_threads;

    std::vector<std::thread> threads;
    threads.reserve(num_threads);

    for (unsigned int t = 0; t < num_threads; ++t) {
        const size_t start = t * chunk_size;
        const size_t end   = std::min(start + chunk_size, n);
        if (start >= end) break;

        // Each thread opens its own file handle to avoid seek/read contention.
        threads.emplace_back([&, start, end]() {
            std::ifstream local_stream(rosbag_path_,
                                       std::ios::in | std::ios::binary);
            if (!local_stream.is_open()) {
                std::cerr << "getRawPayloads: failed to open " << rosbag_path_
                          << " in worker thread" << std::endl;
                return;
            }
            for (size_t i = start; i < end; ++i) {
                const auto& msg_info = messages_info[i];
                std::vector<uint8_t> buf(static_cast<size_t>(msg_info.data_len));
                local_stream.seekg(msg_info.buffer_offset);
                local_stream.read(reinterpret_cast<char*>(buf.data()),
                                  msg_info.data_len);
                payloads[i] = std::make_pair(msg_info.time, std::move(buf));
            }
        });
    }

    for (auto& th : threads) {
        th.join();
    }

    return payloads;
}
