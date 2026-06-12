// ****************************************************************************
// ESM_ErrorModel.cpp - DOA 测量误差模型实现
// ****************************************************************************

#include "ESM_ErrorModel.hpp"

#include <cmath>
#include <random>

void MeasurementErrorModel::Configure(const MeasurementErrorConfig& cfg)
{
    config_ = cfg;
}

void MeasurementErrorModel::ApplyAzElError(double true_az_rad, double true_el_rad,
                                           double true_range_m, double frequency_hz,
                                           double& noisy_az_rad,
                                           double& noisy_el_rad) const
{
    double az_sigma = effective_az_sigma(frequency_hz);
    double el_sigma = effective_el_sigma(frequency_hz);

    // 距离百分比模式：负的固定 sigma 表示百分比
    if (config_.azimuth_error_sigma_rad < 0.0)
        az_sigma = std::max(az_sigma, -config_.azimuth_error_sigma_rad * 0.01 * true_range_m);
    if (config_.elevation_error_sigma_rad < 0.0)
        el_sigma = std::max(el_sigma, -config_.elevation_error_sigma_rad * 0.01 * true_range_m);

    noisy_az_rad = true_az_rad + gaussian_noise(az_sigma);
    noisy_el_rad = true_el_rad + gaussian_noise(el_sigma);
}

void MeasurementErrorModel::ApplyRangeError(double true_range_m, double track_age_s,
                                            double& noisy_range_m,
                                            bool& range_valid) const
{
    range_valid = IsRangeValid(track_age_s);

    double sigma = config_.range_error_m;
    if (config_.range_error_percent > 0.0)
    {
        double percent_sigma = config_.range_error_percent * 0.01 * true_range_m;
        sigma = std::max(sigma, percent_sigma);
    }

    noisy_range_m = true_range_m + gaussian_noise(sigma);
}

bool MeasurementErrorModel::IsRangeValid(double track_age_s) const
{
    if (config_.ranging_time_s <= 0.0)
        return true;
    return track_age_s >= config_.ranging_time_s;
}

double MeasurementErrorModel::effective_az_sigma(double frequency_hz) const
{
    double sigma = config_.azimuth_error_sigma_rad;
    if (!config_.azimuth_error_table.empty() && frequency_hz > 0.0)
        sigma = std::max(sigma, config_.azimuth_error_table.lookup(frequency_hz));
    return sigma;
}

double MeasurementErrorModel::effective_el_sigma(double frequency_hz) const
{
    double sigma = config_.elevation_error_sigma_rad;
    if (!config_.elevation_error_table.empty() && frequency_hz > 0.0)
        sigma = std::max(sigma, config_.elevation_error_table.lookup(frequency_hz));
    return sigma;
}

double MeasurementErrorModel::gaussian_noise(double sigma) const
{
    if (sigma <= 0.0)
        return 0.0;

    // 使用线程局部 RNG 避免全局状态问题
    static thread_local std::mt19937 rng(std::random_device{}());
    std::normal_distribution<double> dist(0.0, sigma);
    return dist(rng);
}
