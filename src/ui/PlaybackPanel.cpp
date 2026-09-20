/**
 * @file PlaybackPanel.cpp
 * @brief 播放轴面板实现
 *
 * 实现关键帧播放轴的所有功能：
 * - 播放/暂停控制（QTimer 定时器驱动）
 * - 上一帧/下一帧、跳转首尾
 * - 滑块拖动与 30ms 节流防抖
 * - 采样步长联动过滤帧列表
 * - 增量球体颜色更新（不重建几何体）
 * - 当前帧菜单（"⋮"按钮或面板右键）：等价于右键当前帧视锥体
 */

#include "ui/PlaybackPanel.h"
#include "ui/ViewportWidget.h"

#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QContextMenuEvent>
#include <algorithm>

// ── 基础播放间隔（毫秒） ──
static constexpr int BASE_PLAY_INTERVAL_MS = 500;

// ── 滑块拖动节流间隔 ──
static constexpr int THROTTLE_INTERVAL_MS = 30;

// ── 倍率映射表 ──
static constexpr int SPEED_MULTIPLIERS[] = {1, 2, 4, 8};
static constexpr int SPEED_COUNT = sizeof(SPEED_MULTIPLIERS) / sizeof(SPEED_MULTIPLIERS[0]);

// ============================================================================
// 构造
// ============================================================================

PlaybackPanel::PlaybackPanel(ViewportWidget* viewport, QWidget* parent)
    : QWidget(parent), m_viewport(viewport) {

    // ── 按钮行 ──
    auto* btnLayout = new QHBoxLayout;
    btnLayout->setSpacing(2);
    btnLayout->setContentsMargins(0, 0, 0, 0);

    auto makeBtn = [this, btnLayout](const QString& text, const QString& tooltip) -> QPushButton* {
        auto* btn = new QPushButton(text, this);
        btn->setFixedSize(28, 28);
        btn->setToolTip(tooltip);
        btnLayout->addWidget(btn);
        return btn;
    };

    m_skipStartBtn = makeBtn(QStringLiteral("⏮"), tr("Skip to start"));
    m_prevBtn      = makeBtn(QStringLiteral("◀"),  tr("Previous frame"));
    m_playBtn      = makeBtn(QStringLiteral("▶"),  tr("Play / Pause"));
    m_nextBtn      = makeBtn(QStringLiteral("▶"),  tr("Next frame"));
    m_skipEndBtn   = makeBtn(QStringLiteral("⏭"), tr("Skip to end"));
    // 当前帧菜单：等价于右键该帧视锥体（为视锥体过密、又不想用采样丢帧时提供入口）
    m_frameMenuBtn = makeBtn(QStringLiteral("⋮"),
        tr("Menu for the current frame (Loop Begin / Loop End) — "
           "same as right-clicking this frame's frustum"));
    connect(m_frameMenuBtn, &QPushButton::clicked, this, [this]() {
        // 弹在按钮下方
        showCurrentFrameMenu(m_frameMenuBtn->mapToGlobal(QPoint(0, m_frameMenuBtn->height())));
    });

    // 设置播放按钮样式（占位稍宽以容纳文字变化）
    m_playBtn->setFixedWidth(40);
    m_playBtn->setText(QStringLiteral("▶"));

    // ── 帧号标签 ──
    m_frameLabel = new QLabel(tr("0 / 0"), this);
    m_frameLabel->setToolTip(tr("Current frame / Total frames"));

    // ── 倍速选择 ──
    m_speedCombo = new QComboBox(this);
    for (int i = 0; i < SPEED_COUNT; ++i) {
        m_speedCombo->addItem(QStringLiteral("%1x").arg(SPEED_MULTIPLIERS[i]));
    }
    m_speedCombo->setCurrentIndex(0);  // 默认 1x
    m_speedCombo->setToolTip(tr("Playback speed multiplier"));

    // ── 信息行（帧号 + 倍速） ──
    auto* infoLayout = new QHBoxLayout;
    infoLayout->setContentsMargins(0, 0, 0, 0);
    infoLayout->addWidget(m_frameLabel);
    infoLayout->addStretch();
    infoLayout->addWidget(m_speedCombo);

    // ── 留存选项 ──
    m_retainCloudCb = new QCheckBox(tr("Retain played cloud"), this);
    m_retainCloudCb->setToolTip(
        tr("Keep every keyframe up to the current one highlighted (white) "
           "instead of only the current frame. The highlight follows the "
           "current frame, so it rolls back when you step or drag backwards."));
    connect(m_retainCloudCb, &QCheckBox::toggled, this, [this](bool checked) {
        m_viewport->setPlaybackRetain(checked);
    });

    // ── 跟随视角选项 ──
    m_followViewCb = new QCheckBox(tr("Follow frame view"), this);
    m_followViewCb->setToolTip(
        tr("Jump the camera to the current keyframe's pose view on every playback "
           "step (same camera move as Ctrl+double-click on that frame). Works while "
           "playing, stepping and dragging."));
    connect(m_followViewCb, &QCheckBox::toggled, this, [this](bool checked) {
        // 勾选立即跳到当前帧视角；取消勾选不做任何相机操作（相机停在原处）
        if (checked) applyFollowView();
    });

    // ── 滑块 ──
    m_slider = new QSlider(Qt::Horizontal, this);
    m_slider->setRange(0, 0);
    m_slider->setToolTip(tr("Drag to seek through keyframes"));

    // ── 主布局 ──
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(4, 4, 4, 4);
    mainLayout->setSpacing(4);
    mainLayout->addLayout(btnLayout);
    mainLayout->addLayout(infoLayout);
    mainLayout->addWidget(m_slider);
    mainLayout->addWidget(m_retainCloudCb);
    mainLayout->addWidget(m_followViewCb);

    // ── 定时器 ──
    m_playTimer = new QTimer(this);
    m_playTimer->setSingleShot(false);
    connect(m_playTimer, &QTimer::timeout, this, &PlaybackPanel::onPlayTick);

    m_throttleTimer = new QTimer(this);
    m_throttleTimer->setSingleShot(true);
    connect(m_throttleTimer, &QTimer::timeout, this, &PlaybackPanel::onThrottleTick);

    // ── 信号连接 ──

    // 播放/暂停按钮
    connect(m_playBtn, &QPushButton::clicked, this, [this]() {
        if (m_playbackFrames.empty()) return;

        if (m_isPlaying) {
            // 暂停 = 播放会话结束：清理播放通道（红色让位给选中态），
            // 再执行完整 selectVertex（更新点云高亮+球体重建+信号）
            pausePlayback();
            m_viewport->highlightPlaybackVertex(-1);
            if (m_currentIndex >= 0 && m_currentIndex < (int)m_playbackFrames.size()) {
                m_viewport->selectVertex(m_playbackFrames[m_currentIndex]);
            }
        } else {
            // 播放
            // 如果已到末尾，从头开始
            if (m_currentIndex >= (int)m_playbackFrames.size() - 1) {
                m_currentIndex = 0;
            }
            m_isPlaying = true;
            m_playBtn->setText(QStringLiteral("⏸"));

            // 立即高亮当前帧
            goToIndex(m_currentIndex);
            // 开始播放前先对齐一次相机（跟随开关打开时）
            applyFollowView();

            // 计算播放间隔（基础间隔 / 倍率）
            int speedIdx = m_speedCombo->currentIndex();
            int interval = BASE_PLAY_INTERVAL_MS / SPEED_MULTIPLIERS[speedIdx];
            m_playTimer->start(interval);
        }
    });    // 上一帧
    connect(m_prevBtn, &QPushButton::clicked, this, [this]() {
        if (m_playbackFrames.empty()) return;
        // 会话结束：清理播放通道后再完整选中
        pausePlayback();
        m_viewport->highlightPlaybackVertex(-1);
        int newIdx = m_currentIndex - 1;
        if (newIdx < 0) newIdx = 0;
        goToIndex(newIdx);
        m_viewport->selectVertex(m_playbackFrames[m_currentIndex]);
        applyFollowView();
    });

    // 下一帧
    connect(m_nextBtn, &QPushButton::clicked, this, [this]() {
        if (m_playbackFrames.empty()) return;
        pausePlayback();
        m_viewport->highlightPlaybackVertex(-1);
        int newIdx = m_currentIndex + 1;
        if (newIdx >= (int)m_playbackFrames.size()) newIdx = (int)m_playbackFrames.size() - 1;
        goToIndex(newIdx);
        m_viewport->selectVertex(m_playbackFrames[m_currentIndex]);
        applyFollowView();
    });

    // 跳转开头
    connect(m_skipStartBtn, &QPushButton::clicked, this, [this]() {
        if (m_playbackFrames.empty()) return;
        pausePlayback();
        m_viewport->highlightPlaybackVertex(-1);
        goToIndex(0);
        m_viewport->selectVertex(m_playbackFrames[m_currentIndex]);
        applyFollowView();
    });

    // 跳转末尾
    connect(m_skipEndBtn, &QPushButton::clicked, this, [this]() {
        if (m_playbackFrames.empty()) return;
        pausePlayback();
        m_viewport->highlightPlaybackVertex(-1);
        goToIndex((int)m_playbackFrames.size() - 1);
        m_viewport->selectVertex(m_playbackFrames[m_currentIndex]);
        applyFollowView();
    });

    // 滑块拖动（拖拽中 —— 使用节流定时器防抖）
    connect(m_slider, &QSlider::sliderMoved, this, &PlaybackPanel::onSliderMoved);

    // 滑块释放
    connect(m_slider, &QSlider::sliderReleased, this, &PlaybackPanel::onSliderReleased);

    // 倍速变化
    connect(m_speedCombo, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int idx) {
        if (m_isPlaying) {
            // 正在播放时调整定时器间隔
            int interval = BASE_PLAY_INTERVAL_MS / SPEED_MULTIPLIERS[idx];
            m_playTimer->setInterval(interval);
        }
    });

    // 初始禁用状态
    setEnabled(false);
}

// ============================================================================
// 公共接口
// ============================================================================

void PlaybackPanel::setKeyframeIds(const std::vector<long>& keyframeIds) {
    m_allKeyframeIds = keyframeIds;
    rebuildFrameList();
    setEnabled(!m_playbackFrames.empty());
}

void PlaybackPanel::pausePlayback() {
    if (!m_isPlaying) return;
    m_isPlaying = false;
    m_playBtn->setText(QStringLiteral("▶"));
    m_playTimer->stop();
}

void PlaybackPanel::onSampleStrideChanged(int stride) {
    m_sampleStride = stride;
    rebuildFrameList();
}

void PlaybackPanel::onGraphClosed() {
    // 停止播放
    pausePlayback();
    m_throttleTimer->stop();

    // 清理播放通道高亮（避免红色标记残留在场景中）
    m_viewport->highlightPlaybackVertex(-1);
    // 复位留存选项（累积高亮随地图关闭一并清空）
    m_retainCloudCb->blockSignals(true);
    m_retainCloudCb->setChecked(false);
    m_retainCloudCb->blockSignals(false);
    m_viewport->setPlaybackRetain(false);

    // 复位跟随视角选项（跟随开关随地图关闭一并复位，且不触发相机操作）
    m_followViewCb->blockSignals(true);
    m_followViewCb->setChecked(false);
    m_followViewCb->blockSignals(false);

    // 清空数据
    m_allKeyframeIds.clear();
    m_playbackFrames.clear();
    m_currentIndex = 0;
    m_slider->setRange(0, 0);
    m_slider->setValue(0);
    m_frameLabel->setText(tr("0 / 0"));
    setEnabled(false);
}

// ============================================================================
// 私有槽函数
// ============================================================================

void PlaybackPanel::onPlayTick() {
    if (m_playbackFrames.empty()) {
        pausePlayback();
        return;
    }

    int nextIdx = m_currentIndex + 1;

    // 到达末尾 → 自动暂停 = 会话结束：清理播放通道，
    // 停在最后一帧并切换为选中高亮
    if (nextIdx >= (int)m_playbackFrames.size()) {
        pausePlayback();
        m_viewport->highlightPlaybackVertex(-1);
        goToIndex(m_currentIndex);
        m_viewport->selectVertex(m_playbackFrames[m_currentIndex]);
        return;
    }

    // 前进一帧（使用轻量级高亮）
    goToIndex(nextIdx);
    m_viewport->highlightPlaybackVertex(m_playbackFrames[m_currentIndex]);
    // 跟随开关打开时，播放推进同步把相机切到该帧位姿视角
    applyFollowView();
}

void PlaybackPanel::onThrottleTick() {
    if (m_throttlePendingIndex >= 0 && m_throttlePendingIndex < (int)m_playbackFrames.size()) {
        m_currentIndex = m_throttlePendingIndex;
        m_throttlePendingIndex = -1;

        // 更新滑块和标签
        m_slider->setValue(m_currentIndex);
        updateLabel();

        // 轻量级高亮（不重建球体几何体）
        m_viewport->highlightPlaybackVertex(m_playbackFrames[m_currentIndex]);
        // 拖动过程中的节流跟随（跟随开关打开时逐帧对齐相机）
        applyFollowView();
    }
}

void PlaybackPanel::onSliderMoved(int pos) {
    if (m_playbackFrames.empty()) return;

    // 如果正在播放，暂停（拖动期间播放通道持续由节流更新着色，不清理）
    pausePlayback();

    // 记录待处理位置，启动 30ms 节流定时器
    m_throttlePendingIndex = pos;
    if (!m_throttleTimer->isActive()) {
        m_throttleTimer->start(THROTTLE_INTERVAL_MS);
    }
}

void PlaybackPanel::onSliderReleased() {
    if (m_throttlePendingIndex >= 0 && m_throttlePendingIndex < (int)m_playbackFrames.size()) {
        m_throttleTimer->stop();
        m_currentIndex = m_throttlePendingIndex;
        m_throttlePendingIndex = -1;

        m_slider->setValue(m_currentIndex);
        updateLabel();

        // 拖动结束 = 会话结束：清理播放通道后完整选中（含点云高亮 + 信号）
        m_viewport->highlightPlaybackVertex(-1);
        m_viewport->selectVertex(m_playbackFrames[m_currentIndex]);
        applyFollowView();
    }
}

// ============================================================================
// 私有辅助方法
// ============================================================================

void PlaybackPanel::rebuildFrameList() {
    m_playbackFrames.clear();

    if (m_allKeyframeIds.empty()) {
        m_slider->setRange(0, 0);
        m_slider->setValue(0);
        m_frameLabel->setText(tr("0 / 0"));
        setEnabled(false);
        return;
    }

    // 按采样步长过滤：只保留 id % stride == 0 的关键帧
    for (long id : m_allKeyframeIds) {
        if (id % m_sampleStride == 0) {
            m_playbackFrames.push_back(id);
        }
    }

    // 如果过滤后为空（极少情况），至少保留第 0 帧
    if (m_playbackFrames.empty() && !m_allKeyframeIds.empty()) {
        m_playbackFrames.push_back(m_allKeyframeIds.front());
    }

    // 重置位置
    m_currentIndex = 0;

    // 更新滑块范围
    int maxIdx = std::max(0, (int)m_playbackFrames.size() - 1);
    m_slider->setRange(0, maxIdx);
    m_slider->setValue(0);

    updateLabel();
    setEnabled(!m_playbackFrames.empty());
}

void PlaybackPanel::goToIndex(int index) {
    if (m_playbackFrames.empty()) return;
    m_currentIndex = std::max(0, std::min(index, (int)m_playbackFrames.size() - 1));

    m_slider->setValue(m_currentIndex);
    updateLabel();
}

void PlaybackPanel::updateLabel() {
    int total = (int)m_playbackFrames.size();
    m_frameLabel->setText(tr("%1 / %2").arg(m_currentIndex).arg(total));
}

void PlaybackPanel::clampIndex() {
    if (m_playbackFrames.empty()) {
        m_currentIndex = 0;
        return;
    }
    m_currentIndex = std::max(0, std::min(m_currentIndex, (int)m_playbackFrames.size() - 1));
}

/**
 * @brief 若勾选跟随则把相机切到当前帧视角
 *
 * 三条前置校验保证幂等安全：未勾选 / 无播放帧 / 下标越界时静默返回。
 * 刻意不做会话/暂停判断——跟随只关心"当前帧"，播放、单步、拖动
 * 三条路径共用本方法。
 */
void PlaybackPanel::applyFollowView() {
    if (!m_followViewCb || !m_followViewCb->isChecked()) return;
    if (m_playbackFrames.empty()) return;
    if (m_currentIndex < 0 || m_currentIndex >= (int)m_playbackFrames.size()) return;
    m_viewport->followFrameView(m_playbackFrames[m_currentIndex]);
}

// ============================================================================
// 当前帧菜单（等价右键该帧视锥体）
// ============================================================================

/**
 * @brief 右键上下文菜单事件重写
 *
 * 面板上任意位置（含 QSlider / QLabel / QCheckBox / QPushButton 等子控件）
 * 的右键都会冒泡到本重写并弹出当前帧菜单。
 */
void PlaybackPanel::contextMenuEvent(QContextMenuEvent* event) {
    showCurrentFrameMenu(event->globalPos());
    event->accept();
}

/**
 * @brief 以当前帧为对象弹出顶点菜单（暂停播放并选中该帧后发信号）
 *
 * 菜单构建统一由 MainWindow::showVertexContextMenu 完成，
 * 本方法只负责确定"当前帧"并把状态切换到位。
 */
void PlaybackPanel::showCurrentFrameMenu(const QPoint& globalPos) {
    if (m_playbackFrames.empty()) return;
    if (m_currentIndex < 0 || m_currentIndex >= (int)m_playbackFrames.size()) return;
    // 菜单是模态的（QMenu::exec 起嵌套事件循环）：必须先停掉播放定时器，
    // 否则菜单期间帧号仍在推进，"当前帧"语义会漂移。
    pausePlayback();
    // 与暂停/单步/拖动释放同一套会话结束语义：清理播放通道后再完整选中，
    // 让该帧以选中态（红 2 倍）明确标出菜单作用于哪一帧。
    m_viewport->highlightPlaybackVertex(-1);
    m_viewport->selectVertex(m_playbackFrames[m_currentIndex]);
    emit frameContextMenuRequested(m_playbackFrames[m_currentIndex], globalPos);
}
