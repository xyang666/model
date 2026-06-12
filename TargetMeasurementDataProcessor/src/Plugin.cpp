#include "Plugin.h"
#include "tracker/Tracker.h"
#include "tracker/RadarTracker.h"
#include "tracker/EOTracker.h"
#include "tracker/IRTracker.h"
#include <cstring>

// 本文件实现 Plugin 的生命周期管理与 C API 适配。
// 内部使用 Tracker 子类作为实际目标跟踪器。
namespace
{
    using LocatorType = TargetMeasurement::Tracker;

    inline LocatorType* toLocator(void* ptr)
    {
        return static_cast<LocatorType*>(ptr);
    }

    inline const LocatorType* toLocator(const void* ptr)
    {
        return static_cast<const LocatorType*>(ptr);
    }

    void copyToTmOutput(const TM_LocationOutput& src,
                        TM_LocationOutput& dst)
    {
        std::memcpy(&dst, &src, sizeof(dst));
    }

}

namespace TargetMeasurement
{
    Plugin g_plugin;

    static TM_Config default_config()
    {
        TM_Config cfg;
        std::memset(&cfg, 0, sizeof(cfg));
        cfg.sensorType = TM_SENSOR_RADAR;
        return cfg;
    }

    Plugin::Plugin()
        : m_locator(nullptr)
        , m_initialized(false)
    {
        std::memset(&m_config, 0, sizeof(m_config));
    }

    Plugin::~Plugin()
    {
        Finalize();
    }

    bool Plugin::Initialize(const TM_Config* cfg)
    {
        if (m_initialized)
            return false;

        if (cfg != nullptr)
        {
            m_config = *cfg;
        }
        else
        {
            m_config = default_config();
        }

        // 工厂：根据传感器类型创建对应 Tracker 子类
        switch (m_config.sensorType)
        {
        case TM_SENSOR_RADAR:
            m_locator = new RadarTracker();
            break;
        case TM_SENSOR_EO:
            m_locator = new EOTracker();
            break;
        case TM_SENSOR_IR:
            m_locator = new IRTracker();
            break;
        default:
            m_locator = new RadarTracker();
            break;
        }
        if (m_locator == nullptr)
            return false;

        if (!toLocator(m_locator)->Initialize(&m_config))
        {
            delete toLocator(m_locator);
            m_locator = nullptr;
            return false;
        }

        m_initialized = true;
        return true;
    }

    bool Plugin::Finalize()
    {
        if (!m_initialized)
            return false;

        delete toLocator(m_locator);
        m_locator = nullptr;
        m_initialized = false;
        return true;
    }

    int Plugin::ProcessMeasurement(const void* measurement,
                                   int measSize,
                                   TM_LocationOutput* output)
    {
        if (!m_initialized || measurement == nullptr || output == nullptr)
            return -1;

        (void)measSize;

        return toLocator(m_locator)->ProcessMeasurement(measurement, output);
    }

    int Plugin::NoDetectUpdate(double simTime, TM_LocationOutput* output)
    {
        if (!m_initialized || output == nullptr)
            return -1;

        return toLocator(m_locator)->NoDetectUpdate(simTime, output);
    }

    int Plugin::Reset()
    {
        if (!m_initialized)
            return -1;
        return toLocator(m_locator)->Reset();
    }

    int Plugin::GetFilterState() const
    {
        if (!m_initialized)
            return TM_STATE_UNINITIALIZED;
        TM_LocationOutput out;
        std::memset(&out, 0, sizeof(out));
        if (toLocator(m_locator)->GetTrack(0, &out) == 0)
            return out.filterState;
        return TM_STATE_UNINITIALIZED;
    }

    double Plugin::GetChiSquaredMean() const
    {
        if (!m_initialized)
            return 0.0;
        TM_LocationOutput out;
        std::memset(&out, 0, sizeof(out));
        if (toLocator(m_locator)->GetTrack(0, &out) == 0)
            return out.chiSquaredMean;
        return 0.0;
    }

    int Plugin::GetTrackCount() const
    {
        if (!m_initialized)
            return 0;
        return toLocator(m_locator)->GetTrackCount();
    }

    int Plugin::GetTrack(int index, TM_LocationOutput* output) const
    {
        if (!m_initialized || !output)
            return -1;
        return toLocator(m_locator)->GetTrack(index, output);
    }

    int Plugin::SetConfig(const TM_Config* cfg)
    {
        if (!m_initialized || cfg == nullptr)
            return -1;

        m_config = *cfg;
        return toLocator(m_locator)->SetConfig(&m_config);
    }

    int Plugin::GetConfig(TM_Config* cfg) const
    {
        if (!m_initialized || cfg == nullptr)
            return -1;

        return toLocator(m_locator)->GetConfig(cfg);
    }

}

extern "C"
{
    TM_EXPORT bool TM_Initialize(const TM_Config* cfg)
    {
        return TargetMeasurement::g_plugin.Initialize(cfg);
    }

    TM_EXPORT bool TM_Finalize()
    {
        return TargetMeasurement::g_plugin.Finalize();
    }

    TM_EXPORT int TM_ProcessMeasurement(const void* measurement,
                                        int measSize,
                                        TM_LocationOutput* output)
    {
        return TargetMeasurement::g_plugin.ProcessMeasurement(measurement, measSize, output);
    }

    TM_EXPORT int TM_NoDetectUpdate(double simTime,
                                    TM_LocationOutput* output)
    {
        return TargetMeasurement::g_plugin.NoDetectUpdate(simTime, output);
    }

    TM_EXPORT int TM_Reset()
    {
        return TargetMeasurement::g_plugin.Reset();
    }

    TM_EXPORT int TM_GetFilterState()
    {
        return TargetMeasurement::g_plugin.GetFilterState();
    }

    TM_EXPORT double TM_GetChiSquaredMean()
    {
        return TargetMeasurement::g_plugin.GetChiSquaredMean();
    }

    TM_EXPORT int TM_SetConfig(const TM_Config* cfg)
    {
        return TargetMeasurement::g_plugin.SetConfig(cfg);
    }

    TM_EXPORT int TM_GetConfig(TM_Config* cfg)
    {
        return TargetMeasurement::g_plugin.GetConfig(cfg);
    }

    TM_EXPORT int TM_GetTrackCount()
    {
        return TargetMeasurement::g_plugin.GetTrackCount();
    }

    TM_EXPORT int TM_GetTrack(int index, TM_LocationOutput* output)
    {
        return TargetMeasurement::g_plugin.GetTrack(index, output);
    }
}
