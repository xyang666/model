#include "Framework.h"
#include "Plugin.h"

// Framework.cpp
// 引擎级别的事件调度与状态查询接口。
// 该层直接与外部集成框架对接。

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
    if (ImageProcessor::g_plugin.GetFilterState() == IP_FILTER_UNINITIALIZED)
        return;

    IP_TrackOutput output;
    int result = ImageProcessor::g_plugin.NoDetectUpdate(time, &output);
    g_hasOutput = (result == 0 && output.valid != 0);
}
