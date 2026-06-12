#include "Framework.h"
#include "Plugin.h"
#include <cstring>

// Framework.cpp 提供引擎级别的事件接口与状态查询。
// 该层直接与外部集成框架对接。
static TM_State g_state = { TM_STATE_UNINITIALIZED, 0.0, false };
static TM_LocationOutput g_lastOutput;
static bool g_hasOutput = false;

DomainEvent** IntegratorInitialize(const char* modelName,
                                      int* size,
                                      double scan_interval_s,
                                      double coast_time_s)
{
    (void)modelName;
    (void)scan_interval_s;
    (void)coast_time_s;
    if (size)
        *size = 0;
    return nullptr;
}

DomainEvent** ProcessEvent(double time,
                              DomainEvent* event,
                              int* size)
{
    (void)time;
    if (event)
        delete event;
    if (size)
        *size = 0;
    return nullptr;
}

int Output()
{
    return g_hasOutput ? 1 : 0;
}

void Input(const char* data)
{
    (void)data;
}

void Update(double time)
{
    if (TargetMeasurement::g_plugin.GetFilterState() == TM_STATE_UNINITIALIZED)
        return;

    std::memset(&g_lastOutput, 0, sizeof(g_lastOutput));
    int result = TargetMeasurement::g_plugin.NoDetectUpdate(time, &g_lastOutput);
    g_state.filterState = TargetMeasurement::g_plugin.GetFilterState();
    g_state.simTime = time;
    g_state.initialized = (result == 0);
    g_hasOutput = (g_lastOutput.valid > 0);
}

TM_State* GetState()
{
    return &g_state;
}
