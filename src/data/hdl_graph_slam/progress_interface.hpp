/**
 * @file progress_interface.hpp
 * @brief 抽象进度回调接口定义
 *
 * ProgressInterface 是一个纯虚接口类，定义了一个与 GUI 框架无关的
 * 进度更新回调协议。任何耗时操作（如地图加载、保存、导出）都可以
 * 通过此接口向调用者报告当前进度。
 *
 * 设计目的：
 *   替代原 HDL Graph SLAM 中的 guik::ProgressInterface，使数据层
 *   与 GUI 框架（Qt/ImGui/控制台）解耦。
 *
 * 使用方法：
 *   1. 实现此接口（例如在 Qt 面板中连接进度条信号）
 *   2. 将其传递给耗时操作（如 load_map_data、dump、save_pointcloud 等）
 *   3. 操作过程中调用相应方法更新进度
 *
 * 默认实现：
 *   所有方法都有空默认实现，调用者可以根据需要只覆盖感兴趣的方法。
 *   这允许调用者在不显示进度的情况下传递默认的 ProgressInterface 实例。
 */
#pragma once

#include <string>

namespace hdl_graph_slam {

/**
 * @brief 抽象的进度回调接口 —— 与 GUI 框架无关
 *
 * 提供以下进度更新回调方法：
 *   - set_title()：设置进度标题
 *   - set_text()：设置进度描述文本
 *   - set_maximum()：设置进度最大值
 *   - set_current()：设置当前进度值
 *   - increment()：进度加一
 *
 * 典型使用流程：
 *   1. set_title("Loading map")
 *   2. set_maximum(100)
 *   3. 循环中调用 increment() 或 set_current()
 *   4. 可选在循环中调用 set_text() 更新当前操作描述
 */
struct ProgressInterface {
    virtual ~ProgressInterface() = default;  ///< 虚析构函数（确保正确析构派生类）

    /**
     * @brief 设置进度标题
     * @param title 标题字符串
     */
    virtual void set_title(const std::string&) {}

    /**
     * @brief 设置进度描述文本
     * @param text 描述文本
     */
    virtual void set_text(const std::string&) {}

    /**
     * @brief 设置进度标题（带参模板）
     *
     * 与 set_title() 的区别：key 是不含参数的模板（如 "progress.opening"），
     * arg 是运行期参数（如目录路径）。UI 侧按 key 查翻译模板（含 %1）
     * 后填参，保证翻译发生在含参格式串上（spec C-3.3）。
     * 默认实现退化为把 key 原样传出（无翻译需求的后端调用方可忽略）。
     *
     * @param key 模板 key（登记于 UI 翻译层）
     * @param arg 填入模板 %1 的参数
     */
    virtual void set_title_fmt(const std::string& key, const std::string& arg) {
        set_title(key);
    }

    /**
     * @brief 设置进度描述文本（带参模板，两个整型参数）
     *
     * 典型场景：计数文案 "keyframe %1/%2"。key/参数分开传，
     * 翻译发生在含 %1/%2 的模板上（spec C-3.3）。
     *
     * @param key 模板 key（登记于 UI 翻译层）
     * @param a   填入模板 %1 的参数
     * @param b   填入模板 %2 的参数
     */
    virtual void set_text_fmt(const std::string& key, int a, int b) {
        set_text(key);
    }

    /**
     * @brief 设置进度最大值（即总量）
     * @param max 最大值
     */
    virtual void set_maximum(int) {}

    /**
     * @brief 设置当前进度值
     * @param current 当前值
     */
    virtual void set_current(int) {}

    /**
     * @brief 进度递增 1
     *
     * 在循环中每次完成一个子任务时调用。
     */
    virtual void increment() {}
};

}  // namespace hdl_graph_slam
