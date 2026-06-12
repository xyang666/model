#include "ESM_Interaction.hpp"

#include <cmath>

static constexpr double kPi = 3.14159265358979323846;
static constexpr double SPEED_OF_LIGHT = 3.0e8; // m/s

bool ESM_Interaction::compute()
{
    // 使用 Beam 1（向后兼容）
    ESM_Beam beam1;
    beam1.antenna = receiver->antenna;
    beam1.frequency_bands = receiver->frequency_bands;
    beam1.beam_index = 1;
    return compute(beam1);
}

bool ESM_Interaction::compute(const ESM_Beam& beam)
{
    active_beam = &beam;

    if (!transmitter || !receiver)
        return false;

    if (!transmitter->active)
        return false;

    // 发射频率必须在至少一个监听频段内。
    if (!receiver->can_receive(transmitter->frequency_hz))
        return false;

    compute_geometry();

    // FOV 约束检查：使用指定波束的天线
    if (!beam.antenna.within_fov(azimuth_rad, elevation_rad, range_m))
        return false;

    compute_link_budget(beam.antenna, beam.frequency_bands);

    // --- 传播与环境（桩函数，当前返回默认值） ---
    compute_propagation_factor();
    compute_atmospheric_absorption();
    compute_terrain_masking();
    compute_horizon_masking();
    compute_zone_attenuation();
    compute_interference();
    compute_doppler();

    // 扫描调度检查：使用指定波束的频段
    scan_available = ESM_Receiver::is_scanning(transmitter->frequency_hz,
                                                sim_time, beam.frequency_bands);

    // 频率相关 SNR 检测门限（按信号类型路由）。
    effective_threshold_dB = receiver->get_detection_threshold_dB(transmitter->frequency_hz,
                                                                   transmitter->signal_type);

    // 频率相关灵敏度（最小可检测功率，按信号类型路由）。
    sensitivity_dBm = receiver->get_sensitivity_dBm(transmitter->frequency_hz,
                                                     transmitter->signal_type);

    // 脉冲信号：对有效 SNR 施加占空比惩罚。
    double snr_for_detection = snr_dB;
    if (transmitter->signal_type == SignalType::PULSED && transmitter->duty_cycle > 0.0)
        snr_for_detection += 10.0 * std::log10(transmitter->duty_cycle);

    snr_margin_dB = snr_for_detection - effective_threshold_dB;

    // 三个条件同时满足才判为直接探测成功。
    bool snr_ok = snr_for_detection >= effective_threshold_dB;
    bool power_ok = received_power_dBm >= sensitivity_dBm;
    detected = scan_available && snr_ok && power_ok;

    return true;
}

void ESM_Interaction::compute_geometry()
{
    Eigen::Vector3d delta = transmitter->position - receiver->position;

    range_m = delta.norm();

    // 方位角：在水平面 (XY) 内，从 +Y（北）起顺时针为正。
    azimuth_rad = std::atan2(delta.x(), delta.y());

    // 仰角：相对水平面的夹角。
    double horiz = std::sqrt(delta.x() * delta.x() + delta.y() * delta.y());
    elevation_rad = std::atan2(delta.z(), horiz);
}

void ESM_Interaction::compute_link_budget(const Antenna& use_antenna,
                                          const std::vector<FrequencyBand>& /*use_bands*/)
{
    // 自由空间路径损耗：FSPL = 20*log10(4π*R*f/c)
    double wavelength = SPEED_OF_LIGHT / transmitter->frequency_hz;
    path_loss_dB = (range_m >= 1.0)
                       ? 20.0 * std::log10(4.0 * kPi * range_m / wavelength)
                       : 0.0;

    // 接收天线在信号到达方向的增益 (dBi)，传递频率用于频率相关修正。
    antenna_gain_dBi = use_antenna.gain_dBi(azimuth_rad, elevation_rad,
                                             transmitter->frequency_hz);

    // 接收功率 (dBm)：P_r = EIRP + G_r(θ) - FSPL - 欧姆损耗 - 馈线损耗
    received_power_dBm = transmitter->eirp_dBm()
                       + antenna_gain_dBi
                       - path_loss_dB
                       - receiver->antenna_ohmic_loss_dB
                       - receiver->receive_line_loss_dB;

    // 极化失配损耗
    received_power_dBm += receiver->GetPolarizationEffect(transmitter->polarization);

    // 带宽失配损耗
    received_power_dBm += receiver->GetBandwidthEffect(transmitter->signal_bandwidth_hz);

    // 噪声底 (dBm)，传递仰角用于系统噪声温度模型
    noise_floor_dBm = receiver->noise_floor_dBm(elevation_rad);

    // 有效噪声底（含 noise_multiplier）
    double eff_noise_dBm = receiver->effective_noise_floor_dBm(elevation_rad);

    // SNR (dB) — 含杂波/干扰，脉冲惩罚前
    // 注：clutter/interference 暂未建模，传递极低功率对数值使其线性值≈0
    snr_dB = receiver->ComputeSignalToNoise(received_power_dBm,
                                            eff_noise_dBm,
                                            -1e30,           // clutter (暂未建模)
                                            -1e30);          // interference (暂未建模)

}

// ---------------------------------------------------------------------------
// 传播 / 环境桩函数 — 返回默认 / 无效应值。
// TODO: 逐一实现真实模型。
// ---------------------------------------------------------------------------

void ESM_Interaction::compute_propagation_factor()
{
    propagation_factor = 1.0; // 自由空间（无多径效应）
}

void ESM_Interaction::compute_atmospheric_absorption()
{
    absorption_factor = 1.0; // 无大气吸收损耗
}

void ESM_Interaction::compute_terrain_masking()
{
    terrain_masked = false; // 无地形遮蔽
}

void ESM_Interaction::compute_horizon_masking()
{
    horizon_masked = false; // 无地平线 / 地球曲率遮蔽
}

void ESM_Interaction::compute_zone_attenuation()
{
    zone_attenuation_dB = 0.0; // 无区域衰减
}

void ESM_Interaction::compute_interference()
{
    interference_power_dBm = -1e30; // 暂未建模 → 极低功率（接近 0 mW）
    interference_factor    = 0.0;
}

void ESM_Interaction::compute_doppler()
{
    doppler_frequency_hz = 0.0;
    doppler_speed_mps    = 0.0;
}