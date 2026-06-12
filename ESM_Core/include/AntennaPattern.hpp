#ifndef ANTENNA_PATTERN_HPP
#define ANTENNA_PATTERN_HPP

#include "FreqTable.hpp"

// 天线辐射方向图模型。
//
// 支持的方向图类型：
//   ISOTROPIC  - 各方向增益均匀
//   GAUSSIAN   - 波束轴起高斯衰减（常用传感器近似）
//   SINC2      - sinc 平方方向图（孔径 / 相控阵近似）
//
// 所有增益值单位为 dBi。
// 偏置角 (off_az_rad, off_el_rad) 是信号来向与天线波束轴之间的
// 有符号角度差，分解为方位和仰角分量。

class AntennaPattern
{
public:
    enum class Type
    {
        ISOTROPIC,  // G = peak_gain_dBi（全向均匀）
        GAUSSIAN,   // G = peak - 12*(θ_az/BW_az)² - 12*(θ_el/BW_el)²  [dB]
        SINC2       // G = peak + 20*log10(|sinc(π*θ/BW)|²)，使用总偏置角
    };

    Type   type{Type::ISOTROPIC};
    double peak_gain_dBi{0.0};        // 波束轴峰值增益 (dBi)
    double az_beamwidth_rad{1.5708};  // 方位 3dB 波束宽度 (rad)，默认 90°
    double el_beamwidth_rad{1.5708};  // 仰角 3dB 波束宽度 (rad)，默认 90°
    double back_lobe_floor_dB{-30.0}; // 后瓣增益下限，相对峰值 (dB)

    // 频率相关增益修正表。
    // 非空时，gain_dBi() 将查表插值结果加到计算出的增益上。
    FreqTable gain_adjustment_table;

    // 硬增益下限 (dBi)。返回增益不会低于此值，优先级高于后瓣下限。
    double minimum_gain_dBi{-40.0};

    // 返回偏置角 (off_az_rad, off_el_rad) 处的天线增益 (dBi)。
    // frequency_hz > 0 且 gain_adjustment_table 非空时应用频率修正。
    double gain_dBi(double off_az_rad, double off_el_rad,
                    double frequency_hz = 0.0) const;

    // 计算方位区间 [min_az_rad, max_az_rad] 上归一化功率增益
    // 超过 threshold_norm 的角度占比。
    //
    // threshold_norm = G_req / G_peak（线性比值，[0, 1]）。
    // 用于 PSOS 方位重叠概率 (PA) 计算。
    //
    // GAUSSIAN 使用解析解，SINC2 在 NUM_PA_SAMPLE_POINTS 个等分点上采样，
    // ISOTROPIC 始终返回 1.0。
    double GetGainThresholdFraction(double threshold_norm,
                                    double min_az_rad,
                                    double max_az_rad) const;

    static constexpr int NUM_PA_SAMPLE_POINTS = 1000;

private:
    double gaussian_gain_dB(double off_az_rad, double off_el_rad) const;
    double sinc2_gain_dB(double off_az_rad, double off_el_rad) const;

    // 方位角 az_rad 处归一化功率增益（仰角 = 0），[0, 1]。
    double norm_power_at_azimuth(double az_rad) const;
};

#endif // ANTENNA_PATTERN_HPP
