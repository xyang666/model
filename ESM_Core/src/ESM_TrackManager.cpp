// ****************************************************************************
// ESM_TrackManager.cpp - M/N 航迹管理器实现
// ****************************************************************************

#include "ESM_TrackManager.hpp"
#include <algorithm>

namespace {

int count_detections(const std::deque<bool>& window)
{
    int count = 0;
    for (bool d : window)
        if (d) ++count;
    return count;
}

} // anonymous namespace

void TrackManager::Configure(const TrackInitConfig& init,
                             const TrackMaintConfig& maint)
{
    init_cfg_  = init;
    maint_cfg_ = maint;
}

void TrackManager::OnDetectionOpportunity(const std::string& emitter_id,
                                          bool detected, double sim_time)
{
    just_initiated_.clear();
    just_dropped_.clear();

    auto it = histories_.find(emitter_id);
    if (it == histories_.end())
    {
        if (!detected)
            return; // 未检测到，不创建航迹

        TrackHistory hist;
        hist.last_update_time = sim_time;
        histories_[emitter_id] = hist;
        it = histories_.find(emitter_id);
    }

    TrackHistory& hist = it->second;
    hist.last_update_time = sim_time;

    // 维护检测窗口
    hist.init_window.push_back(detected);
    while (static_cast<int>(hist.init_window.size()) > init_cfg_.n_opportunities)
        hist.init_window.pop_front();

    hist.maint_window.push_back(detected);
    while (static_cast<int>(hist.maint_window.size()) > maint_cfg_.n_opportunities)
        hist.maint_window.pop_front();

    if (detected)
        hist.last_detect_time = sim_time;

    // 状态转换
    TrackState prev_state = hist.state;

    switch (hist.state)
    {
    case NONE:
        if (detected)
            hist.state = INITIATING;
        break;

    case INITIATING:
        if (count_detections(hist.init_window) >= init_cfg_.m_required)
        {
            hist.state = MAINTENANCE;
            just_initiated_.push_back(emitter_id);
        }
        break;

    case MAINTENANCE:
        if (count_detections(hist.maint_window) < maint_cfg_.m_required)
            hist.state = COAST;
        break;

    case COAST:
        if (count_detections(hist.maint_window) >= maint_cfg_.m_required)
        {
            hist.state = MAINTENANCE;
        }
        else if ((sim_time - hist.last_update_time) > maint_cfg_.coast_time_s)
        {
            hist.state = DROPPED;
            just_dropped_.push_back(emitter_id);
        }
        break;

    case DROPPED:
        break;
    }
}

TrackManager::TrackState TrackManager::GetTrackState(const std::string& emitter_id) const
{
    auto it = histories_.find(emitter_id);
    if (it == histories_.end())
        return NONE;
    return it->second.state;
}

bool TrackManager::JustInitiated(const std::string& emitter_id) const
{
    return std::find(just_initiated_.begin(), just_initiated_.end(), emitter_id)
           != just_initiated_.end();
}

bool TrackManager::JustDropped(const std::string& emitter_id) const
{
    return std::find(just_dropped_.begin(), just_dropped_.end(), emitter_id)
           != just_dropped_.end();
}

std::vector<std::string> TrackManager::GetActiveTrackIds() const
{
    std::vector<std::string> ids;
    for (const auto& kv : histories_)
        if (kv.second.state == MAINTENANCE || kv.second.state == COAST)
            ids.push_back(kv.first);
    return ids;
}

void TrackManager::RemoveTrack(const std::string& emitter_id)
{
    histories_.erase(emitter_id);
}

void TrackManager::Reset()
{
    histories_.clear();
    just_initiated_.clear();
    just_dropped_.clear();
}

