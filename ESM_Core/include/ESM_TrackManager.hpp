// ****************************************************************************
// ESM_TrackManager.hpp - M/N 航迹管理器
//
// 对应 AFSIM WsfSensorTracker 的 M/N 逻辑。
//
// 四种航迹状态：
//   NONE        — 从未检测到
//   INITIATING  — 检测到但未达到 M/N 建立标准
//   MAINTENANCE — 航迹已建立，正在维持
//   COAST       — 暂时丢失，但仍在 coast 时间内
//   DROPPED     — 已删除
//
// M/N 逻辑：
//   TrackInit:  N 次机会中检测到 M 次 → 建立航迹
//   TrackMaint: N 次机会中检测到 M 次 → 维持航迹，否则进入 COAST
//   Coast:      超过 coast_time_s 无检测 → 删除航迹
// ****************************************************************************
#pragma once

#include <deque>
#include <map>
#include <string>

struct TrackInitConfig
{
    int m_required = 1;      // M 次检测
    int n_opportunities = 1; // N 次机会
};

struct TrackMaintConfig
{
    int    m_required = 1;
    int    n_opportunities = 1;
    double coast_time_s = 5.0;
};

class TrackManager
{
public:
    enum TrackState
    {
        NONE,
        INITIATING,
        MAINTENANCE,
        COAST,
        DROPPED
    };

    void Configure(const TrackInitConfig& init, const TrackMaintConfig& maint);

    // 每次检测机会调用
    // detected: 本次是否探测成功
    // sim_time: 当前仿真时间
    void OnDetectionOpportunity(const std::string& emitter_id,
                                bool detected, double sim_time);

    // 获取航迹状态
    TrackState GetTrackState(const std::string& emitter_id) const;

    // 检查航迹是否刚刚建立（本次帧首次达到 MAINTENANCE）
    bool JustInitiated(const std::string& emitter_id) const;

    // 检查航迹是否刚刚丢失（本次帧从 MAINTENANCE/COAST 变为 DROPPED）
    bool JustDropped(const std::string& emitter_id) const;

    // 获取已建立（MAINTENANCE 状态）的航迹 ID 列表
    std::vector<std::string> GetActiveTrackIds() const;

    // 删除指定航迹
    void RemoveTrack(const std::string& emitter_id);

    // 获取本帧刚刚丢失的航迹 ID 列表
    std::vector<std::string> GetJustDroppedIds() const { return just_dropped_; }

    // 重置所有状态
    void Reset();

private:
    struct TrackHistory
    {
        std::deque<bool> init_window;    // 建立窗口
        std::deque<bool> maint_window;   // 维持窗口
        TrackState state = NONE;
        double last_detect_time = 0.0;
        double last_update_time = 0.0;
    };

    TrackInitConfig  init_cfg_;
    TrackMaintConfig  maint_cfg_;
    std::map<std::string, TrackHistory> histories_;
    std::vector<std::string> just_initiated_;
    std::vector<std::string> just_dropped_;
};
