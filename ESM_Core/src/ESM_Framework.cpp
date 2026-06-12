// ****************************************************************************
// ESM_Framework.cpp - 电子侦察模型框架调度接口实现
//
// 实现仿真引擎所需的标准时间相关模型接口：
//   - 离散事件：IntegratorInitialize / ProcessEvent / ESM_Output / ESM_Input
//   - 连续时间：Update / GetState
//
// 所有函数内部委托给全局单例 g_plugin（ESM_Plugin）。
// Initialize / Finalize 的定义在 ESM_Plugin.cpp 中，此处不重复定义。
//
// DomainEvent 事件类型：
//   "Scan"            — 扫描帧，bOutput=false
//   "Detection"        — 新辐射源被探测，bOutput=true
//   "Declaration"      — 发射器类型被首次声明（Phase 1），bOutput=true
//   "TargetDeclared"   — 目标平台类型被推断（Phase 1），bOutput=true
//   "TrackInitiated"   — M/N 航迹建立（Phase 2），bOutput=true
//   "TrackDropped"     — M/N 航迹丢失（Phase 2），bOutput=true
// ****************************************************************************

#include "stdafx.h"
#include "ESM_Framework.h"
#include "ESM_Plugin.h"

#include <cstring>
#include <cstdlib>
#include <vector>

// ============================================================================
// 模块级状态
// ============================================================================

static double  s_scan_interval_s = 1.0;
static double  s_current_sim_time = 0.0;
static ESM_State s_state = {0, 0.0, false};

static int s_last_track_count = 0;

// ============================================================================
// 辅助函数
// ============================================================================

static void update_state()
{
    s_state.track_count  = g_plugin.GetTrackCount();
    s_state.sim_time     = s_current_sim_time;
    s_state.psos_enabled = false;
}

static DomainEvent* make_event(double time, const char* type, bool bOutput,
                               void* data = nullptr)
{
    DomainEvent* e = new DomainEvent();
    e->time    = time;
    e->type    = type;
    e->bOutput = bOutput;
    e->data    = data;
    return e;
}

// 检查指定航迹是否刚刚发生某种状态变化
static bool track_just_declared(int index)
{
    const char* declared = g_plugin.GetTrackDeclaredType(index);
    const char* emitter_id = g_plugin.GetTrackId(index);
    return declared && declared[0] != '\0'
           && g_plugin.EmitterJustDeclared(emitter_id);
}

static bool track_just_initiated(int index)
{
    const char* emitter_id = g_plugin.GetTrackId(index);
    return g_plugin.TrackJustInitiated(emitter_id);
}

// ============================================================================
// 离散事件调度接口
// ============================================================================

extern "C"
{

DLL_EXPORT DomainEvent** IntegratorInitialize(
    const char* /*modelName*/, int* size,
    double scan_interval_s, double coast_time_s)
{
    g_plugin.Initialize();
    g_plugin.SetCoastTime(coast_time_s);

    s_scan_interval_s = scan_interval_s;
    s_last_track_count = 0;
    s_current_sim_time = 0.0;

    DomainEvent** events = new DomainEvent*[1];
    events[0] = make_event(0.0, "Scan", false);
    *size = 1;
    return events;
}

DLL_EXPORT DomainEvent** ProcessEvent(double time, DomainEvent* event, int* size)
{
    s_current_sim_time = time;

    if (!event)
    {
        *size = 0;
        return nullptr;
    }

    if (std::strcmp(event->type, "Scan") == 0)
    {
        // 执行一次扫描帧
        g_plugin.Update(time);
        int track_count = g_plugin.GetTrackCount();

        // 收集动态事件
        std::vector<DomainEvent*> event_list;

        // 总是调度下一个 "Scan"
        event_list.push_back(make_event(s_scan_interval_s, "Scan", false));

        // 新航迹探测
        int new_detections = track_count - s_last_track_count;
        if (new_detections < 0) new_detections = 0;
        s_last_track_count = track_count;
        if (new_detections > 0)
            event_list.push_back(make_event(0.0, "Detection", true));

        // 遍历航迹，检查 Phase 1/2 状态变化
        for (int i = 0; i < track_count; ++i)
        {
            const char* emitter_id = g_plugin.GetTrackId(i);
            if (!emitter_id || emitter_id[0] == '\0')
                continue;

            // Phase 1: 发射器类型首次声明
            if (track_just_declared(i))
                event_list.push_back(make_event(0.0, "Declaration", true));

            // Phase 2: 航迹刚建立
            if (track_just_initiated(i))
                event_list.push_back(make_event(0.0, "TrackInitiated", true));
        }

        // Phase 1: 目标平台类型推断
        if (g_plugin.TargetJustDeclared())
            event_list.push_back(make_event(0.0, "TargetDeclared", true));

        // Phase 2: 航迹丢失（通过 TrackManager 的 dropped 列表）
        std::vector<std::string> dropped = g_plugin.GetJustDroppedIds();
        for (const auto& dropped_id : dropped)
            event_list.push_back(make_event(0.0, "TrackDropped", true));

        // 将 event_list 转为数组
        *size = static_cast<int>(event_list.size());
        DomainEvent** events = new DomainEvent*[*size];
        for (int i = 0; i < *size; ++i)
            events[i] = event_list[i];

        delete event;
        return events;
    }

    // 未知事件类型
    *size = 0;
    delete event;
    return nullptr;
}

DLL_EXPORT int ESM_Output()
{
    update_state();
    return s_state.track_count;
}

DLL_EXPORT void ESM_Input(const char* /*data*/)
{
    // 预留：解析 JSON 字符串，动态更新辐射源参数
}

// ========================================================================
// 连续时间调度接口
// ========================================================================

DLL_EXPORT void Update(double time)
{
    s_current_sim_time = time;
    s_last_track_count = g_plugin.GetTrackCount();
    g_plugin.Update(time);
}

DLL_EXPORT ESM_State* GetState()
{
    update_state();
    return &s_state;
}

} // extern "C"
