// ****************************************************************************
// ESM_ErrorModel.hpp - DOA 测量误差模型
//
// 对应 AFSIM WsfStandardSensorErrorModel。
//
// 支持三种误差叠加：
//   1. 固定标准差（azimuth_error_sigma, elevation_error_sigma, range_error）
//   2. 频率相关误差表（按频率插值得到标准差）
//   3. 距离百分比模式（误差 = % × true_range）
// ****************************************************************************
#pragma once

#include "FreqTable.hpp"

struct MeasurementErrorConfig
{
    // 固定标准差 (rad / m)
    double azimuth_error_sigma_rad   = 0.0;
    double elevation_error_sigma_rad = 0.0;
    double range_error_m             = 0.0;

    // 频率相关误差表（频率 Hz → 标准差 rad）
    FreqTable azimuth_error_table;
    FreqTable elevation_error_table;

    // 距离百分比模式：> 0 时 range_error = range_error_percent * true_range
    double range_error_percent = 0.0;

    // 测距时间：跟踪持续时间超过此值后距离才有效 (s)
    double ranging_time_s = 0.0;
};

class MeasurementErrorModel
{
public:
    void Configure(const MeasurementErrorConfig& cfg);

    // 对 DOA 真值施加高斯误差
    void ApplyAzElError(double true_az_rad, double true_el_rad,
                        double true_range_m, double frequency_hz,
                        double& noisy_az_rad, double& noisy_el_rad) const;

    // 对距离施加误差，track_age_s 用于判断测距时间
    void ApplyRangeError(double true_range_m, double track_age_s,
                         double& noisy_range_m, bool& range_valid) const;

    // 判断距离是否有效（跟踪时间 >= ranging_time_s）
    bool IsRangeValid(double track_age_s) const;

private:
    // 生成高斯噪声
    double gaussian_noise(double sigma) const;

    // 获取有效 sigma（固定值 + 频率表插值，取较大者）
    double effective_az_sigma(double frequency_hz) const;
    double effective_el_sigma(double frequency_hz) const;

    MeasurementErrorConfig config_;
};
