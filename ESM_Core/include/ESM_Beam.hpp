// ****************************************************************************
// ESM_Beam.hpp - 波束结构体
//
// 每个波束独立配置天线方向和监听频段，对应 AFSIM PassiveBeam。
// 多个波束对同一目标独立检测后取最佳 SNR。
// ****************************************************************************
#pragma once

#include "Antenna.hpp"
#include <vector>

struct ESM_Beam
{
    Antenna antenna;                            // 独立天线（方向图 + 指向 + FOV）
    std::vector<FrequencyBand> frequency_bands;  // 本波束监听的频段
    int     beam_index = 1;                      // 波束编号 (1-based)
};
