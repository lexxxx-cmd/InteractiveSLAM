/**
 * @file bag_importer.cpp
 * @brief ROS1 bag 导入器实现 —— 对齐 SCPGO 位姿图数据流
 *
 * 管线（参考 SCPGO pose_pcd_exporter + saveKeyframes，去掉回环/优化）：
 *   1. YAML 配置解析（topic / 外参 / 抽稀 / 输出）
 *   2. 读取 odom 与 PointCloud2 原始 payload，手写解析消息
 *   3. 时间戳最近邻匹配
 *   4. 关键帧抽稀（passthrough 全量 或 meter/deg gap）
 *   5. 外参变换：T_WL → T_WI，点云变换到 IMU 局部系
 *   6. 导出 graph.g2o + keyframes/{data, raw.pcd, cloud.pcd}
 */
#include "data/hdl_graph_slam/bag_importer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

#include <boost/filesystem.hpp>
#include <boost/format.hpp>

#include <yaml-cpp/yaml.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>

#include "data/rosbag/rosbag.h"

namespace hdl_graph_slam {

namespace {

using PointT = pcl::PointXYZI;
using PointCloud = pcl::PointCloud<PointT>;

/** @brief 快速预检 bag 文件头（避免 Rosbag 构造失败时内部 exit 杀进程） */
bool isRosbagFile(const std::string& path) {
    std::ifstream ifs(path, std::ios::binary);
    if (!ifs) return false;
    char magic[32] = {0};
    ifs.read(magic, sizeof(magic) - 1);
    return std::string(magic).find("#ROSBAG") != std::string::npos;
}

/** @brief 解析 nav_msgs/Odometry 原始 payload（读取 header + 位姿） */
struct OdomFrame {
    uint64_t timestamp = 0;
    double x = 0, y = 0, z = 0;
    double qx = 0, qy = 0, qz = 0, qw = 1;
};

OdomFrame parseOdomPayload(const uint8_t* payload, size_t /*length*/) {
    OdomFrame odom;
    size_t offset = 0;

    uint32_t seq = *(uint32_t*)(payload + offset); offset += 4;
    uint32_t sec = *(uint32_t*)(payload + offset); offset += 4;
    uint32_t nsec = *(uint32_t*)(payload + offset); offset += 4;
    odom.timestamp = (uint64_t)sec * 1000000000ULL + nsec;

    uint32_t frame_id_len = *(uint32_t*)(payload + offset); offset += 4;
    offset += frame_id_len;

    uint32_t child_frame_id_len = *(uint32_t*)(payload + offset); offset += 4;
    offset += child_frame_id_len;

    odom.x = *(double*)(payload + offset); offset += 8;
    odom.y = *(double*)(payload + offset); offset += 8;
    odom.z = *(double*)(payload + offset); offset += 8;
    odom.qx = *(double*)(payload + offset); offset += 8;
    odom.qy = *(double*)(payload + offset); offset += 8;
    odom.qz = *(double*)(payload + offset); offset += 8;
    odom.qw = *(double*)(payload + offset); offset += 8;

    return odom;
}

/** @brief 解析 sensor_msgs/PointCloud2 原始 payload（提取 x/y/z/intensity） */
struct PcdFrame {
    uint64_t timestamp = 0;
    PointCloud::Ptr cloud;
};

PcdFrame parseSensorPc2Payload(const uint8_t* payload, size_t length) {
    PcdFrame frame;
    frame.cloud.reset(new PointCloud());
    size_t offset = 0;

    uint32_t seq = *(uint32_t*)(payload + offset); offset += 4;
    uint32_t sec = *(uint32_t*)(payload + offset); offset += 4;
    uint32_t nsec = *(uint32_t*)(payload + offset); offset += 4;
    frame.timestamp = (uint64_t)sec * 1000000000ULL + nsec;

    uint32_t frame_id_len = *(uint32_t*)(payload + offset); offset += 4;
    offset += frame_id_len;

    uint32_t height = *(uint32_t*)(payload + offset); offset += 4;
    uint32_t width = *(uint32_t*)(payload + offset); offset += 4;
    uint32_t point_num = height * width;

    uint32_t fields_size = *(uint32_t*)(payload + offset); offset += 4;
    struct FieldInfo {
        std::string name; uint32_t offset; uint8_t datatype; uint32_t count;
    };
    std::vector<FieldInfo> fields;
    for (uint32_t i = 0; i < fields_size; ++i) {
        uint32_t name_len = *(uint32_t*)(payload + offset); offset += 4;
        std::string name((const char*)(payload + offset), name_len); offset += name_len;
        uint32_t field_offset = *(uint32_t*)(payload + offset); offset += 4;
        uint8_t datatype = *(uint8_t*)(payload + offset); offset += 1;
        uint32_t count = *(uint32_t*)(payload + offset); offset += 4;
        fields.push_back({name, field_offset, datatype, count});
    }

    // is_bigendian（v1 不支持大端，罕见）
    uint8_t is_bigendian = *(uint8_t*)(payload + offset); offset += 1;
    (void)is_bigendian;

    uint32_t point_step = *(uint32_t*)(payload + offset); offset += 4;
    uint32_t row_step = *(uint32_t*)(payload + offset); offset += 4;

    uint32_t data_len = *(uint32_t*)(payload + offset); offset += 4;
    const uint8_t* data_ptr = payload + offset; offset += data_len;

    // is_dense
    offset += 1;

    int x_idx = -1, y_idx = -1, z_idx = -1, intensity_idx = -1;
    for (size_t i = 0; i < fields.size(); ++i) {
        if (fields[i].name == "x") x_idx = (int)i;
        if (fields[i].name == "y") y_idx = (int)i;
        if (fields[i].name == "z") z_idx = (int)i;
        if (fields[i].name == "intensity") intensity_idx = (int)i;
    }
    if (x_idx < 0 || y_idx < 0 || z_idx < 0) {
        return frame;  // 没有 XYZ 字段，无法使用
    }

    if (data_len < point_num * point_step) point_num = data_len / point_step;

    frame.cloud->points.resize(point_num);
    for (uint32_t i = 0; i < point_num; ++i) {
        size_t base = (size_t)i * point_step;
        frame.cloud->points[i].x = *(float*)(data_ptr + base + fields[x_idx].offset);
        frame.cloud->points[i].y = *(float*)(data_ptr + base + fields[y_idx].offset);
        frame.cloud->points[i].z = *(float*)(data_ptr + base + fields[z_idx].offset);
        frame.cloud->points[i].intensity =
            intensity_idx >= 0
                ? *(float*)(data_ptr + base + fields[intensity_idx].offset)
                : 0.0f;
    }
    frame.cloud->width = point_num;
    frame.cloud->height = 1;
    frame.cloud->is_dense = false;
    return frame;
}

/** @brief Odometry 位姿（T_WL 语义）→ Eigen::Isometry3d */
Eigen::Isometry3d odomToIsometry(const OdomFrame& o) {
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::Quaterniond q(o.qw, o.qx, o.qy, o.qz);
    q.normalize();
    T.linear() = q.toRotationMatrix();
    T.translation() = Eigen::Vector3d(o.x, o.y, o.z);
    return T;
}

/**
 * @brief 外参变换：T_WL → T_WI（IMU in World），并给出点云到 IMU 局部系的变换
 *
 * 照 SCPGO：P_IMU = R_IL·P_Lidar + t_IL → T_LI = T_IL⁻¹
 *   T_WI = T_WL · T_LI
 *   点云变换 T_IW = T_WI⁻¹（世界 → IMU 局部系，用于关键帧局部点云）
 */
struct ImuCalibration {
    Eigen::Isometry3d T_WI;      ///< IMU 在世界系
    Eigen::Affine3d  T_IW;       ///< 世界 → IMU 局部（点云用）
};

ImuCalibration applyCalibration(const Eigen::Isometry3d& T_WL,
                                const Eigen::Matrix3d& R_IL,
                                const Eigen::Vector3d& t_IL) {
    ImuCalibration calib;
    // T_LI = T_IL⁻¹
    Eigen::Matrix3d R_LI = R_IL.transpose();
    Eigen::Vector3d t_LI = -R_IL.transpose() * t_IL;

    calib.T_WI = Eigen::Isometry3d::Identity();
    calib.T_WI.linear() = T_WL.linear() * R_LI;
    calib.T_WI.translation() = T_WL.linear() * t_LI + T_WL.translation();

    Eigen::Matrix3d R_IW = calib.T_WI.linear().transpose();
    Eigen::Vector3d t_IW = -calib.T_WI.linear().transpose() * calib.T_WI.translation();
    calib.T_IW = Eigen::Affine3d::Identity();
    calib.T_IW.linear() = R_IW;
    calib.T_IW.translation() = t_IW;
    return calib;
}

/** @brief 写 VERTEX_SE3:QUAT 行（g2o 标准） */
std::string vertexLine(int id, const Eigen::Isometry3d& T) {
    Eigen::Quaterniond q(T.linear());
    q.normalize();
    Eigen::Vector3d t = T.translation();
    std::ostringstream oss;
    oss << std::setprecision(9)
        << "VERTEX_SE3:QUAT " << id << " "
        << t.x() << " " << t.y() << " " << t.z() << " "
        << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
    return oss.str();
}

/** @brief 写 EDGE_SE3:QUAT 行（对角信息矩阵：平移 1e6，旋转 1e4） */
std::string edgeLine(int idFrom, int idTo, const Eigen::Isometry3d& rel) {
    Eigen::Quaterniond q(rel.linear());
    q.normalize();
    Eigen::Vector3d t = rel.translation();
    std::ostringstream oss;
    oss << std::setprecision(9)
        << "EDGE_SE3:QUAT " << idFrom << " " << idTo << " "
        << t.x() << " " << t.y() << " " << t.z() << " "
        << q.x() << " " << q.y() << " " << q.z() << " " << q.w();
    // 6×6 对角信息矩阵上三角（21 个值）：前 3 平移 1e6，后 3 旋转 1e4
    const double infoDiag[6] = {1e6, 1e6, 1e6, 1e4, 1e4, 1e4};
    for (int i = 0; i < 6; ++i) {
        for (int j = i; j < 6; ++j) {
            oss << " " << (i == j ? infoDiag[i] : 0.0);
        }
    }
    return oss.str();
}

/** @brief 从 YAML 节点读取参数，带默认值回退（yaml-cpp 0.9 安全 const 访问） */
template <typename T>
T getParamOrDefault(const YAML::Node& node, const std::string& key, const T& def) {
    if (!node.IsMap() || !node[key]) return def;
    try {
        return node[key].as<T>();
    } catch (const YAML::Exception&) {
        return def;
    }
}

}  // namespace

BagImportConfig BagImporter::loadConfig(const std::string& yamlPath) {
    BagImportConfig cfg;
    try {
        YAML::Node root = YAML::LoadFile(yamlPath);
        if (!root.IsMap()) return cfg;

        cfg.odomTopic  = getParamOrDefault<std::string>(root, "odom_topic", cfg.odomTopic);
        cfg.cloudTopic = getParamOrDefault<std::string>(root, "cloud_topic", cfg.cloudTopic);

        // 外参 R_IL / t_IL
        if (root["calib_r"] && root["calib_r"].IsSequence()) {
            auto r = root["calib_r"].as<std::vector<double>>();
            if (r.size() == 9) {
                for (int row = 0; row < 3; ++row)
                    for (int col = 0; col < 3; ++col)
                        cfg.calibR(row, col) = r[row * 3 + col];
            }
        }
        if (root["calib_t"] && root["calib_t"].IsSequence()) {
            auto t = root["calib_t"].as<std::vector<double>>();
            if (t.size() == 3) cfg.calibT = Eigen::Vector3d(t[0], t[1], t[2]);
        }

        cfg.passthroughMode = getParamOrDefault<bool>(root, "passthrough_mode", cfg.passthroughMode);
        cfg.meterGap        = getParamOrDefault<double>(root, "meter_gap", cfg.meterGap);
        cfg.degGap          = getParamOrDefault<double>(root, "deg_gap", cfg.degGap);
        cfg.saveFullCloud   = getParamOrDefault<bool>(root, "save_full_cloud", cfg.saveFullCloud);
    } catch (const YAML::Exception& e) {
        std::cerr << "[BagImporter] YAML parse error: " << e.what()
                  << " — using defaults" << std::endl;
    }
    return cfg;
}

std::vector<std::string> BagImporter::listTopics(const std::string& bagPath) {
    if (!isRosbagFile(bagPath)) return {};
    try {
        Rosbag bag(bagPath);
        return bag.getAvailableTopics();
    } catch (const std::exception& e) {
        std::cerr << "[BagImporter] listTopics failed: " << e.what() << std::endl;
        return {};
    }
}

bool BagImporter::isOutputDirNonEmpty(const std::string& path) {
    try {
        boost::filesystem::path dir(path);
        if (!boost::filesystem::exists(dir)) return false;
        return !boost::filesystem::is_empty(dir);
    } catch (const std::exception&) {
        return false;  // 无法判断时按"可写入"处理
    }
}

bool BagImporter::clearDirectory(const std::string& path) {
    try {
        boost::filesystem::path dir(path);
        if (!boost::filesystem::exists(dir)) return true;  // 不存在视为已清空
        // 删除目录内全部内容，保留目录本身
        for (boost::filesystem::directory_iterator it(dir), end; it != end; ++it) {
            boost::filesystem::remove_all(it->path());
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "[BagImporter] clearDirectory failed: " << e.what() << std::endl;
        return false;
    }
}

BagImportResult BagImporter::import(const BagImportConfig& cfg,
                                    ProgressInterface& progress) {
    try {
        return importImpl(cfg, progress);
    } catch (const boost::filesystem::filesystem_error& e) {
        BagImportResult r;
        r.error = std::string("filesystem error: ") + e.what();
        return r;
    } catch (const std::exception& e) {
        BagImportResult r;
        r.error = std::string("import error: ") + e.what();
        return r;
    } catch (...) {
        BagImportResult r;
        r.error = "unknown import error";
        return r;
    }
}

/**
 * @brief 导入主体（可能抛出异常，由 import() 捕获并转为错误结果）
 */
BagImportResult BagImporter::importImpl(const BagImportConfig& cfg,
                                        ProgressInterface& progress) {
    BagImportResult result;
    progress.set_title("Opening " + cfg.bagPath);

    if (cfg.bagPath.empty() || !isRosbagFile(cfg.bagPath)) {
        result.error = "not a ROS1 bag file: " + cfg.bagPath;
        return result;
    }

    // ---- 1. 读取 odom / cloud payload ----
    Rosbag bag(cfg.bagPath);

    progress.set_text("reading odometry");
    auto odomPayloads = bag.getRawPayloads(cfg.odomTopic);
    if (odomPayloads.empty()) {
        result.error = "no frames on odometry topic: " + cfg.odomTopic;
        return result;
    }
    std::vector<OdomFrame> odomFrames;
    odomFrames.reserve(odomPayloads.size());
    for (auto& e : odomPayloads) {
        odomFrames.push_back(parseOdomPayload(e.second.data(), e.second.size()));
    }
    std::sort(odomFrames.begin(), odomFrames.end(),
              [](const OdomFrame& a, const OdomFrame& b) { return a.timestamp < b.timestamp; });

    progress.set_text("reading point clouds");
    auto cloudPayloads = bag.getRawPayloads(cfg.cloudTopic);
    if (cloudPayloads.empty()) {
        result.error = "no frames on point cloud topic: " + cfg.cloudTopic;
        return result;
    }
    std::vector<PcdFrame> pcdFrames;
    pcdFrames.reserve(cloudPayloads.size());
    for (auto& e : cloudPayloads) {
        PcdFrame f = parseSensorPc2Payload(e.second.data(), e.second.size());
        if (f.cloud && !f.cloud->empty()) {
            pcdFrames.push_back(std::move(f));
        }
    }
    std::sort(pcdFrames.begin(), pcdFrames.end(),
              [](const PcdFrame& a, const PcdFrame& b) { return a.timestamp < b.timestamp; });

    if (pcdFrames.empty()) {
        result.error = "no usable PointCloud2 frames on topic: " + cfg.cloudTopic;
        return result;
    }

    // ---- 2. 时间戳最近邻匹配（一对一，odom/点云均只前进） ----
    struct Match {
        OdomFrame odom;
        PcdFrame  pcd;
    };
    std::vector<Match> matches;
    {
        size_t oi = 0, pi = 0;
        while (oi < odomFrames.size() && pi < pcdFrames.size()) {
            int64_t ts_p = (int64_t)pcdFrames[pi].timestamp;
            int64_t d_cur = std::llabs(ts_p - (int64_t)odomFrames[oi].timestamp);
            int64_t d_next = (oi + 1 < odomFrames.size())
                                 ? std::llabs(ts_p - (int64_t)odomFrames[oi + 1].timestamp)
                                 : INT64_MAX;
            if (d_next < d_cur) {
                ++oi;  // 下一条 odom 更近，前进 odom
            } else {
                matches.push_back({odomFrames[oi], pcdFrames[pi]});
                ++oi; ++pi;
            }
        }
    }
    if (matches.empty()) {
        result.error = "no odometry/point-cloud pairs matched";
        return result;
    }

    // ---- 3. 关键帧抽稀（passthrough 全量 或 meter/deg gap） ----
    // 照 SCPGO process_pg：累计相邻帧位移/转角，超过阈值才保留关键帧。
    std::vector<size_t> keyframeIdx;  // matches 中的索引
    if (cfg.passthroughMode) {
        keyframeIdx.resize(matches.size());
        for (size_t i = 0; i < matches.size(); ++i) keyframeIdx[i] = i;
    } else {
        double transAccum = 1e9;  // 首个帧必取（同 SCPGO 初始大值）
        double rotAccum = 1e9;
        const double meterGap = std::max(1e-6, cfg.meterGap);
        const double radGap = cfg.degGap * 0.017453292519943295;  // deg → rad
        Eigen::Isometry3d prevPose = odomToIsometry(matches.front().odom);
        for (size_t i = 0; i < matches.size(); ++i) {
            Eigen::Isometry3d pose = odomToIsometry(matches[i].odom);
            Eigen::Isometry3d rel = prevPose.inverse() * pose;
            double dt = rel.translation().norm();
            double dr = Eigen::AngleAxisd(rel.linear()).angle();
            transAccum += dt;
            rotAccum += dr;
            if (transAccum > meterGap || rotAccum > radGap) {
                keyframeIdx.push_back(i);
                transAccum = 0.0;
                rotAccum = 0.0;
                prevPose = pose;
            }
        }
    }
    if (keyframeIdx.empty()) {
        result.error = "no keyframes after thinning";
        return result;
    }

    // ---- 4. 导出标准地图目录 ----
    // 结构必须与 InteractiveGraph::load_map_data / dump 兼容：
    //   <outDir>/graph.g2o            （根目录）
    //   <outDir>/000000/ 000001/ …    （关键帧子目录**平铺**在根目录，无 keyframes/ 层）
    std::string outDir = cfg.outputDir;
    result.mapDirectory = outDir;  // 由调用方保证非空（GraphManager 处理临时目录）
    boost::filesystem::create_directories(outDir);

    progress.set_maximum((int)keyframeIdx.size());
    progress.set_text("writing keyframes");

    std::ofstream g2oStream(outDir + "/graph.g2o");
    if (!g2oStream) {
        result.error = "cannot create graph.g2o in " + outDir;
        return result;
    }

    pcl::VoxelGrid<PointT> voxel;
    voxel.setLeafSize(0.02f, 0.02f, 0.02f);

    Eigen::Isometry3d prevWorldPose = Eigen::Isometry3d::Identity();
    int count = 0;
    for (size_t kf : keyframeIdx) {
        const Match& m = matches[kf];
        // 关键帧子目录平铺在地图根目录：outDir/000000、outDir/000001 …
        const std::string kfDir = (boost::format("%s/%06d") % outDir % count).str();
        boost::filesystem::create_directories(kfDir);

        // 外参变换：T_WL → T_WI；点云变换到 IMU 局部系
        Eigen::Isometry3d T_WL = odomToIsometry(m.odom);
        ImuCalibration calib = applyCalibration(T_WL, cfg.calibR, cfg.calibT);
        const Eigen::Isometry3d& T_WI = calib.T_WI;

        // 局部（IMU 系）点云
        PointCloud::Ptr localCloud(new PointCloud());
        pcl::transformPointCloud(*m.pcd.cloud, *localCloud, calib.T_IW.cast<float>());

        // data 文件（与 KeyFrame::load 兼容）；写入失败即中止（避免残缺地图）
        {
            std::ofstream ofs(kfDir + "/data");
            if (!ofs) {
                result.error = "cannot create data file in " + kfDir;
                return result;
            }
            uint64_t sec = m.odom.timestamp / 1000000000ULL;
            uint64_t nsec = m.odom.timestamp % 1000000000ULL;
            ofs << "stamp " << sec << " " << nsec << "\n";
            ofs << "estimate\n" << T_WI.matrix() << "\n";
            ofs << "odom\n" << T_WI.matrix() << "\n";
            ofs << "id " << count << "\n";
        }

        // raw.pcd（全分辨率，可选）
        if (cfg.saveFullCloud) {
            pcl::io::savePCDFileBinary(kfDir + "/raw.pcd", *localCloud);
        }

        // cloud.pcd（0.02m 体素降采样）
        PointCloud::Ptr sampled(new PointCloud());
        voxel.setInputCloud(localCloud);
        voxel.filter(*sampled);
        pcl::io::savePCDFileBinary(kfDir + "/cloud.pcd", *sampled);

        // g2o 顶点 + 相邻边（纯里程计链）
        g2oStream << vertexLine(count, T_WI) << "\n";
        if (count > 0) {
            Eigen::Isometry3d rel = prevWorldPose.inverse() * T_WI;
            g2oStream << edgeLine(count - 1, count, rel) << "\n";
        }
        prevWorldPose = T_WI;

        ++count;
        progress.set_text((boost::format("keyframe %d/%d") % count % keyframeIdx.size()).str());
        progress.increment();
    }
    g2oStream.close();

    if (count == 0) {
        result.error = "no keyframes written";
        return result;
    }

    result.success = true;
    result.keyframeCount = count;
    return result;
}

}  // namespace hdl_graph_slam
