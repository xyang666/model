#pragma once

#include "IP_Interface.h"

namespace ImageProcessor
{

/**
 * Plugin
 * ------
 * 封装核心 ImageProcessor 与外部引擎之间的桥接逻辑。
 *
 * 职责：
 *   - 初始化/释放图像处理器模块
 *   - 接收图像帧输入
 *   - 转发处理结果
 *   - 管理运行时配置
 *
 * 对应 TargetMeasurementDataProcessingModel::Plugin 的架构角色。
 */
class Plugin
{
public:
    Plugin();
    ~Plugin();

    bool Initialize(const IP_Config* cfg);
    bool Finalize();

    int ProcessImage(const IP_ImageInput* input,
                     IP_ProcessOutput* output);

    int NoDetectUpdate(double simTime,
                       IP_TrackOutput* output);

    int Reset();
    int GetFilterState() const;

    int SetConfig(const IP_Config* cfg);
    int GetConfig(IP_Config* cfg) const;

private:
    void*    m_processor;
    IP_Config m_config;
    bool      m_initialized;
};

extern Plugin g_plugin;

} // namespace ImageProcessor
