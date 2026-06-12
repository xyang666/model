#pragma once

#include "IP_Interface.h"

/*
 * Framework
 * ---------
 * 引擎级别的事件调度接口。
 * 对应 TargetMeasurementDataProcessingModel 中 Framework.h 的职责。
 */

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
    double      time;
    const char* type;
    bool        bOutput;
    void*       data;
};

/**
 * 引擎在仿真开始前初始化模型。
 */
IP_EXPORT DomainEvent** IntegratorInitialize(const char* modelName,
                                               int* size,
                                               double scan_interval_s,
                                               double coast_time_s);

/**
 * 事件调度接口。
 */
IP_EXPORT DomainEvent** ProcessEvent(double time,
                                       DomainEvent* event,
                                       int* size);

/**
 * 获取模型输出数量。
 */
IP_EXPORT int Output();

/**
 * 引擎将外部数据路由到本模型。
 */
IP_EXPORT void Input(const char* data);

/**
 * 连续时间推进接口。
 */
IP_EXPORT void Update(double time);

#ifdef __cplusplus
}
#endif
