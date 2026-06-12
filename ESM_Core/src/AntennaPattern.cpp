#include "AntennaPattern.hpp"

#include <cmath>
#include <algorithm>

static constexpr double kPi = 3.14159265358979323846;

double AntennaPattern::gain_dBi(double off_az_rad, double off_el_rad,
                                double frequency_hz) const
{
    double relative_dB = 0.0;

    switch (type)
    {
    case Type::ISOTROPIC:
        relative_dB = 0.0;
        break;
    case Type::GAUSSIAN:
        relative_dB = gaussian_gain_dB(off_az_rad, off_el_rad);
        break;
    case Type::SINC2:
        relative_dB = sinc2_gain_dB(off_az_rad, off_el_rad);
        break;
    }

    // 钳位到后瓣下限
    relative_dB = std::max(relative_dB, back_lobe_floor_dB);

    double gain = peak_gain_dBi + relative_dB;

    // 频率相关增益修正
    if (frequency_hz > 0.0 && !gain_adjustment_table.empty())
        gain += gain_adjustment_table.lookup(frequency_hz);

    // 硬增益下限
    gain = std::max(gain, minimum_gain_dBi);

    return gain;
}

// 高斯波束：G(az, el) = -12 * [(az/BW_az)² + (el/BW_el)²]  (dB)
// 精确匹配 3dB 波束宽度定义。
double AntennaPattern::gaussian_gain_dB(double off_az_rad, double off_el_rad) const
{
    double norm_az = (az_beamwidth_rad > 0.0) ? (off_az_rad / az_beamwidth_rad) : 0.0;
    double norm_el = (el_beamwidth_rad > 0.0) ? (off_el_rad / el_beamwidth_rad) : 0.0;
    return -12.0 * (norm_az * norm_az + norm_el * norm_el);
}

// Sinc 平方：使用总偏置角 theta。
// G(theta) = 20*log10(|sinc(π * theta / BW)|²)
// 其中 BW 为方位和仰角波束宽度的均值。
double AntennaPattern::sinc2_gain_dB(double off_az_rad, double off_el_rad) const
{
    double bw = 0.5 * (az_beamwidth_rad + el_beamwidth_rad);
    if (bw <= 0.0)
        return 0.0;

    double theta = std::sqrt(off_az_rad * off_az_rad + off_el_rad * off_el_rad);
    double x = kPi * theta / bw;

    // sinc(x) = sin(x)/x, sinc(0) = 1
    double sinc_val = (std::abs(x) < 1e-9) ? 1.0 : std::sin(x) / x;
    double sinc2 = sinc_val * sinc_val;

    return (sinc2 > 1e-12) ? 20.0 * std::log10(sinc2) : back_lobe_floor_dB;
}

// 归一化功率增益（线性域），方位面上（el = 0）的增益分布。
// 返回 [0, 1]，表示相对峰值功率的增益。
double AntennaPattern::norm_power_at_azimuth(double az_rad) const
{
    switch (type)
    {
    case Type::ISOTROPIC:
        return 1.0;

    case Type::GAUSSIAN:
    {
        // 高斯功率模式：G(az) = G_peak * exp(-a * az²)
        // 其中 a = 4*ln(2) / BW_az²，确保在 az = BW_az/2 处为 -3dB
        if (az_beamwidth_rad <= 0.0)
            return 1.0;
        double a = 2.772588722239781; // 4*ln(2)
        double norm = az_rad / az_beamwidth_rad;
        return std::exp(-a * norm * norm);
    }

    case Type::SINC2:
    {
        double bw = 0.5 * (az_beamwidth_rad + el_beamwidth_rad);
        if (bw <= 0.0)
            return 1.0;
        double x = kPi * std::abs(az_rad) / bw;
        if (x < 1e-9)
            return 1.0;
        double sinc = std::sin(x) / x;
        return sinc * sinc;
    }
    }
    return 1.0;
}

// 计算方位区间内归一化增益超过门限的比例。
// 使用归一化功率模式（el = 0），因为 PA 计算关注发射天线在
// 当前仰角切片上的方位增益覆盖。
double AntennaPattern::GetGainThresholdFraction(double threshold_norm,
                                                double min_az_rad,
                                                double max_az_rad) const
{
    // 无效门限或无扫描范围
    if (threshold_norm <= 0.0 || threshold_norm > 1.0)
        return (threshold_norm <= 0.0) ? 1.0 : 0.0;

    double span = max_az_rad - min_az_rad;
    if (span <= 0.0)
        return 0.0;

    switch (type)
    {
    case Type::ISOTROPIC:
        return 1.0;

    case Type::GAUSSIAN:
    {
        // 解析解：G_norm(az) = exp(-a * az²)
        // 解 az = sqrt(-ln(threshold_norm) / a)
        if (az_beamwidth_rad <= 0.0)
            return 1.0;
        double a = 2.772588722239781; // 4*ln(2)
        double az_half = az_beamwidth_rad * std::sqrt(-std::log(threshold_norm) / a);
        double coverage = 2.0 * az_half;
        return std::min(1.0, coverage / span);
    }

    case Type::SINC2:
    {
        // 数值采样
        int count_above = 0;
        for (int i = 0; i < NUM_PA_SAMPLE_POINTS; ++i)
        {
            double t = static_cast<double>(i) / static_cast<double>(NUM_PA_SAMPLE_POINTS - 1);
            double az = min_az_rad + t * span;
            if (norm_power_at_azimuth(az) >= threshold_norm)
                ++count_above;
        }
        return static_cast<double>(count_above) / static_cast<double>(NUM_PA_SAMPLE_POINTS);
    }
    }
    return 0.0;
}
