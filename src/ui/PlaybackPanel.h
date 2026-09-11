/**
 * @file PlaybackPanel.h
 * @brief 播放轴面板头文件
 *
 * PlaybackPanel 提供关键帧播放轴功能：
 * - 从 idx=0 自动播放至最后一个关键帧（采样感知）
 * - 播放/暂停、上一帧/下一帧、跳转首尾
 * - 位置滑块拖动
 * - 播放倍速切换（1x/2x/4x/8x）
 * - 采样步长联动（仅播放已渲染的球体）
 *
 * 性能设计：
 * - 滑块拖动时使用 highlightPlaybackVertex() 增量更新球体颜色
 * - 配合 30ms 节流定时器防止过度重绘
 * - 滑块释放或按钮点击时调用完整 selectVertex()
 *
 * 高亮生命周期（单一状态源）：
 * - 播放/拖动期间：highlightPlaybackVertex() 轻量红色高亮（会话进行中）
 * - 暂停/单步/跳转/滑块释放/播完 = 会话结束：先 highlightPlaybackVertex(-1)
 *   清理播放通道，再 selectVertex() 切换为选中高亮（红色让位给选中态）
 * - 关闭面板/关闭地图：暂停并清理播放通道，场景不残留红色标记
 *
 * 点云留存（"Retain played cloud"）：
 * - 勾选后高亮范围 = 所有 id <= 当前帧的关键帧整帧前缀，由当前帧推导，
 *   与播放路径无关：前进增长、倒放/倒拖滑块自动回退
 * - 会话结束（暂停/单步/拖动结束）前缀高亮保留，仅播放下标切换为选中态
 * - 关闭复选框或关闭地图时清除前缀（onGraphClosed 复位复选框）
 */

#pragma once

#include <QWidget>
#include <QSlider>
#include <QPushButton>
#include <QLabel>
#include <QComboBox>
#include <QCheckBox>
#include <QTimer>
#include <vector>

class ViewportWidget;

/**
 * @brief 播放轴浮动面板
 *
 * 提供关键帧序列的播放控制，自动适配采样步长（sampleStride）。
 * 仅在采样后的帧集上操作，与渲染视图保持一致。
 */
class PlaybackPanel : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param viewport 关联的 3D 视口（用于 selectVertex / highlightPlaybackVertex）
     * @param parent   父级 Qt 组件
     */
    explicit PlaybackPanel(ViewportWidget* viewport, QWidget* parent = nullptr);

    /**
     * @brief 设置关键帧 ID 列表（在加载地图后调用）
     * @param keyframeIds 所有关键帧的顶点 ID 列表（已排序）
     */
    void setKeyframeIds(const std::vector<long>& keyframeIds);

    /**
     * @brief 暂停播放（幂等，不改变当前帧、不做选中）
     *
     * 供外部在用户主动选择其他帧时调用（选择让位语义）：
     * 停止定时器并复位播放按钮图标。播放高亮的清理由调用方
     * highlightPlaybackVertex(-1) 完成，本方法不触碰视口。
     */
    void pausePlayback();

public slots:
    /**
     * @brief 响应采样步长变化
     * @param stride 新的采样步长
     *
     * 重建播放帧列表，重置当前位置为 0。
     */
    void onSampleStrideChanged(int stride);

    /** @brief 重置面板状态（关闭地图时调用） */
    void onGraphClosed();

private slots:
    /** @brief 播放定时器 tick，前进一帧 */
    void onPlayTick();
    /** @brief 滑块拖动时的节流更新 */
    void onThrottleTick();
    /** @brief 滑块值变化（拖动中） */
    void onSliderMoved(int pos);
    /** @brief 滑块释放 */
    void onSliderReleased();

private:
    /** @brief 根据采样步长重建播放帧列表 */
    void rebuildFrameList();
    /** @brief 跳转到指定索引位置 */
    void goToIndex(int index);
    /** @brief 更新 UI 标签文本 */
    void updateLabel();
    /** @brief 确保当前索引在有效范围内 */
    void clampIndex();

    // ── UI 控件 ──
    QPushButton* m_skipStartBtn = nullptr;  ///< 跳转到开头
    QPushButton* m_prevBtn = nullptr;       ///< 上一帧
    QPushButton* m_playBtn = nullptr;       ///< 播放/暂停
    QPushButton* m_nextBtn = nullptr;       ///< 下一帧
    QPushButton* m_skipEndBtn = nullptr;    ///< 跳转到末尾
    QSlider*     m_slider = nullptr;        ///< 位置滑块
    QLabel*      m_frameLabel = nullptr;    ///< 帧号标签 "42 / 15000"
    QComboBox*   m_speedCombo = nullptr;    ///< 倍速选择
    QCheckBox*   m_retainCloudCb = nullptr; ///< "留存已播放点云"复选框

    // ── 播放状态 ──
    ViewportWidget* m_viewport = nullptr;   ///< 关联的 3D 视口
    std::vector<long> m_allKeyframeIds;     ///< 所有关键帧 ID（已排序）
    std::vector<long> m_playbackFrames;     ///< 采样后的播放帧 ID 列表
    int  m_currentIndex = 0;                ///< 当前在 m_playbackFrames 中的索引
    int  m_sampleStride = 1;                ///< 当前采样步长
    bool m_isPlaying = false;               ///< 是否正在播放

    // ── 定时器 ──
    QTimer* m_playTimer = nullptr;          ///< 播放定时器（根据倍速设置间隔）
    QTimer* m_throttleTimer = nullptr;      ///< 滑块节流定时器（30ms single-shot）
    int     m_throttlePendingIndex = -1;    ///< 节流待处理的索引位置
};
