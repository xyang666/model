#pragma once

#include "TM_Interface.h"

namespace TargetMeasurement
{
    /**
     * Plugin
     * ------
     * 封装底层定位器与外部引擎之间的桥接逻辑。
     *
     * Plugin 类负责：
     * - 初始化/释放定位模块；
     * - 接收外部测量输入；
     * - 转发输出结果；
     * - 管理运行时配置。
     */
    class Plugin
    {
    public:
        Plugin();
        ~Plugin();

        bool Initialize(const TM_Config* cfg);
        bool Finalize();

        int ProcessMeasurement(const void* measurement,
                                 int measSize,
                                 TM_LocationOutput* output);
        int NoDetectUpdate(double simTime, TM_LocationOutput* output);
        int Reset();
        int GetFilterState() const;
        double GetChiSquaredMean() const;
        int GetTrackCount() const;
        int GetTrack(int index, TM_LocationOutput* output) const;

        int SetConfig(const TM_Config* cfg);
        int GetConfig(TM_Config* cfg) const;

    private:
        void* m_locator;
        TM_Config m_config;
        bool      m_initialized;
    };

    extern Plugin g_plugin;
}
