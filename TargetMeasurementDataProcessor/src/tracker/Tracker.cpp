#include "tracker/Tracker.h"
#include <cstring>
#include <algorithm>

namespace TargetMeasurement
{

Tracker::Tracker()
    : m_initialized_(false)
    , m_lastSimTime_(0.0)
{
    std::memset(&m_config_, 0, sizeof(m_config_));
}

Tracker::~Tracker()
{
}

bool Tracker::Initialize(const TM_Config* cfg)
{
    if (!cfg)
        return false;

    m_config_ = *cfg;
    if (m_config_.track_init_n <= 0) m_config_.track_init_n = 1;
    if (m_config_.track_init_m <= 0) m_config_.track_init_m = 1;
    if (m_config_.track_maint_n <= 0) m_config_.track_maint_n = 1;
    if (m_config_.track_maint_m <= 0) m_config_.track_maint_m = 1;
    if (m_config_.track_coast_s <= 0) m_config_.track_coast_s = 5.0;

    m_lastSimTime_ = 0.0;
    m_initialized_ = true;
    return true;
}

int Tracker::Reset()
{
    m_tracks_.clear();
    m_initialized_ = false;
    return 0;
}

TrackData* Tracker::getTrack(int targetId)
{
    if (targetId < 0)
        return nullptr;
    auto it = m_tracks_.find(targetId);
    if (it != m_tracks_.end())
        return &it->second;
    return nullptr;
}

void Tracker::setSensorFunc(int sensorType, SensorObsFn func)
{
    if (sensorType >= 0 && sensorType <= 2)
        obsFuncs_[sensorType] = func;
}

void Tracker::deleteStaleTracks(double simTime)
{
    for (auto it = m_tracks_.begin(); it != m_tracks_.end();)
    {
        auto& t = it->second;
        if (t.filter && t.filter->isInitialized() &&
            (simTime - t.lastMeasTime) > m_config_.track_coast_s)
        {
            it = m_tracks_.erase(it);
        }
        else
        {
            ++it;
        }
    }
}

int Tracker::GetTrackCount() const
{
    return static_cast<int>(m_tracks_.size());
}

int Tracker::GetTrack(int index, TM_LocationOutput* output) const
{
    if (!output || index < 0)
        return -1;
    int i = 0;
    for (const auto& pair : m_tracks_)
    {
        if (i == index)
        {
            *output = pair.second.lastOutput;
            return 0;
        }
        ++i;
    }
    return -1;
}

} // namespace TargetMeasurement
