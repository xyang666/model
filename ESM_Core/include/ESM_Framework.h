// ****************************************************************************
// ESM_Framework.h - 电子侦察模型框架调度接口
//
// 对外部仿真引擎暴露标准的时间相关模型接口，支持两种调度方式：
//   1）离散事件调度：DomainEvent + ProcessEvent（"Scan" 事件驱动扫描探测）
//   2）连续时间调度：Update + GetState（引擎每步长推送时间、拉取状态）
//
// 引擎调度流程（离散事件）：
//   IntegratorInitialize → 获取初始事件数组
//   ProcessEvent → 模型处理事件、返回新事件，引擎排序后循环
//     ┌────────────────────────────────────────────────────────┐
//     │ 事件           触发条件                   bOutput       │
//     │ "Scan"         扫描帧                     false         │
//     │ "Detection"    新辐射源被探测              true          │
//     │ "Declaration"  发射器类型首次声明(Phase 1)  true          │
//     │ "TargetDeclared" 平台类型推断(Phase 1)      true          │
//     │ "TrackInitiated" M/N 航迹建立(Phase 2)     true          │
//     │ "TrackDropped"  M/N 航迹丢失(Phase 2)      true          │
//     └────────────────────────────────────────────────────────┘
//   ESM_Output   → 引擎按需获取输出数据（航迹数）
//   ESM_Input    → 引擎路由其他模型的输出到本模型
//
// 引擎调度流程（连续时间）：
//   Update(time) → GetState() → 引擎路由状态到下游
//
// 配置函数见 ESM_Interface.h，内部实现见 ESM_Plugin.h / ESM_Framework.cpp
// ****************************************************************************
#pragma once

#include "stdafx.h"

// ============================================================================
// 事件结构定义
// ============================================================================
struct DomainEvent
{
    double      time;     // 事件触发相对时间 (s)，必须 >= 0
    const char* type;     // 事件类型字符串（"Scan" / "Detection" 等）
    bool        bOutput;  // 是否为输出事件（触发引擎调用 Output 函数）
    void*       data;     // 附加数据，由模型分配和解析
};

// ============================================================================
// ESM 传感器状态（供 GetState 返回）
// ============================================================================
struct ESM_State
{
    int    track_count;     // 当前航迹数
    double sim_time;        // 最近一次更新的仿真时间 (s)
    bool   psos_enabled;    // PSOS 模式是否启用
};

#ifdef __cplusplus
extern "C" {
#endif

// ========================================================================
// 模型生命周期
// ========================================================================
DLL_EXPORT bool Initialize();
DLL_EXPORT bool Finalize();

// ========================================================================
// 离散事件调度接口
// ========================================================================

// 初始化事件：引擎在仿真开始前调用，模型返回初始事件数组。
//   modelName       - 模型实例名称
//   size            - [出] 返回的事件数组长度
//   scan_interval_s - 扫描间隔 (s)，模型每隔此时长触发一次 "Scan" 事件
//   coast_time_s    - 航迹保持时间 (s)，超过此时长的旧航迹被删除
DLL_EXPORT DomainEvent** IntegratorInitialize(
    const char* modelName, int* size,
    double scan_interval_s, double coast_time_s);

// 事件处理：引擎按时间顺序将事件投递给模型。
// 模型根据 event->type 执行对应逻辑，返回新生成的事件数组。
//   time  - 当前仿真时间 (s)
//   event - 当前触发的事件对象（模型负责 delete）
//   size  - [出] 新生成的事件数量
// 返回值：新事件数组（引擎负责 delete[]），无事件时返回 nullptr 且 *size=0
DLL_EXPORT DomainEvent** ProcessEvent(double time, DomainEvent* event, int* size);

// 输出函数：当 ProcessEvent 返回的事件中 bOutput == true 时由引擎调用。
// 返回当前活跃航迹数量。
DLL_EXPORT int ESM_Output();

// 输入函数：引擎将其他模型的输出路由到本模型。
// data 为 JSON 字符串，用于动态更新辐射源参数。
DLL_EXPORT void ESM_Input(const char* data);

// ========================================================================
// 连续时间调度接口
// ========================================================================

// 状态更新：引擎强制将模型状态推进到 time 时刻。
// 内部调用 ESM_Sensor::update() 执行完整扫描帧。
DLL_EXPORT void Update(double time);

// 状态获取：返回 ESM_State 结构指针，调用 Update 后由引擎调用。
DLL_EXPORT ESM_State* GetState();

#ifdef __cplusplus
}
#endif
