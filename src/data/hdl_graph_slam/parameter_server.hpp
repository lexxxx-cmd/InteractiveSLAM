/**
 * @file parameter_server.hpp
 * @brief 轻量级参数服务器定义
 *
 * ParameterServer 是一个简单的键值对参数存储结构，用于替代 ROS 的
 * ParameterServer。它使用 std::unordered_map 存储参数，支持：
 *   - 任意类型的参数值（通过 boost::any 实现类型擦除）
 *   - 默认值机制（首次访问时自动插入默认值）
 *   - 字符串到数值的自动类型转换（通过 boost::lexical_cast）
 *
 * 使用方式：
 *   double val = params.param<double>("param_name", 1.0);
 *   如果 "param_name" 不存在，则插入默认值 1.0 并返回。
 *   如果 "param_name" 是字符串类型，自动尝试转换为 double。
 *
 * 注意：
 *   本实现是非线程安全的，适用于单线程场景。
 *   参数首次访问时通过默认值隐式注册，无需显式声明。
 */
#pragma once
#include <iostream>
#include <unordered_map>
#include <boost/any.hpp>
#include <boost/lexical_cast.hpp>

namespace hdl_graph_slam {

/**
 * @brief 轻量级参数服务器
 *
 * 替代 ROS ParameterServer 的简单键值存储实现。
 * 在 InteractiveGraph 中用于存储优化器参数和信息矩阵参数。
 *
 * 特性：
 *   - 基于 std::unordered_map 的键值存储
 *   - 使用 boost::any 实现类型擦除，支持任意类型
 *   - 自动字符串类型转换（使用 boost::lexical_cast）
 *   - 延迟注册：参数在首次访问时自动创建（使用默认值）
 *
 * 类型安全：
 *   如果已存储的参数类型与请求类型不一致：
 *   - 如果存储的是字符串，尝试转换为目标类型
 *   - 否则输出警告并返回默认值
 */
struct ParameterServer {
public:
    /**
     * @brief 获取参数值（带默认值）
     * @tparam T      参数类型（由调用者指定）
     * @param name         参数名称
     * @param default_value 默认值（参数不存在时返回此值并被注册）
     * @return 参数值
     *
     * 行为说明：
     *   1. 如果参数不存在，插入默认值并返回
     *   2. 如果参数已存在且类型匹配，直接返回
     *   3. 如果参数已存在但类型不匹配：
     *      - 若存储的是 std::string，尝试用 lexical_cast 转换为目标类型
     *      - 否则输出类型不匹配警告并返回默认值
     */
    template<typename T>
    T param(const std::string& name, const T& default_value) {
        auto found = params.find(name);
        if (found == params.end()) {
            // 参数不存在，插入默认值
            params.insert(std::make_pair(name, default_value));
            found = params.find(name);
        }

        if (found->second.type() == typeid(T)) {
            // 类型匹配，直接返回
            return boost::any_cast<T>(found->second);
        } else if (found->second.type() == typeid(std::string)) {
            // 存储的是字符串，尝试转换为目标类型
            auto str = boost::any_cast<std::string>(found->second);
            T p = boost::lexical_cast<T>(str);
            found->second = p;  // 缓存转换结果，避免重复转换
            return p;
        }

        // 类型不匹配且无法转换
        std::cerr << "warning: param " << name << "'s type does not match!!" << std::endl;
        return default_value;
    }

private:
    std::unordered_map<std::string, boost::any> params;  ///< 参数键值存储
};

}  // namespace hdl_graph_slam
