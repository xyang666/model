#pragma once

#include "TM_Interface.h"

// Framework 提供了引擎级别的调度接口。
// 该模块对外暴露事件初始化、事件处理、输出计数、输入接收和状态查询。
#ifdef __cplusplus
extern "C" {
#endif

/**
 * DomainEvent
 * -----------
 * 引擎事件调度接口的通用事件结构。
 */
struct DomainEvent
{
    double      time;    /**< 事件触发时间（秒） */
    const char* type;    /**< 事件类型字符串 */
    bool        bOutput; /**< 是否为输出事件 */
    void*       data;    /**< 附加数据 */
};

/**
 * TM_State
 * --------
 * 连续更新接口的状态查询结果。
 */
struct TM_State
{
    int   filterState; /**< 当前滤波器状态 */
    double simTime;    /**< 最近一次更新时刻 */
    bool  initialized; /**< 是否已初始化 */
};

/**
 * IntegratorInitialize
 * --------------------
 * 引擎在仿真开始前初始化模型并返回初始事件数组。
 */
TM_EXPORT DomainEvent** IntegratorInitialize(const char* modelName,
                                               int* size,
                                               double scan_interval_s,
                                               double coast_time_s);

/**
 * ProcessEvent
 * ------------
 * 事件调度接口，按顺序处理引擎投递的事件。
 */
TM_EXPORT DomainEvent** ProcessEvent(double time,
                                       DomainEvent* event,
                                       int* size);

/**
 * Output
 * ------
 * 当事件为输出事件时，引擎调用此函数获取模型输出数量。
 */
TM_EXPORT int Output();

/**
 * Input
 * -----
 * 引擎将外部数据路由到本模型，数据格式可扩展为 JSON 字符串。
 */
TM_EXPORT void Input(const char* data);

/**
 * Update
 * ------
 * 连续时间调度接口，将模型状态推进至指定时刻。
 */
TM_EXPORT void Update(double time);

/**
 * GetState
 * --------
 * 获取连续时间调度下的当前模型状态。
 */
TM_EXPORT TM_State* GetState();

#ifdef __cplusplus
}
#endif
