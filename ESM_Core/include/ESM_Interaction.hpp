#ifndef ESM_INTERACTION_HPP
#define ESM_INTERACTION_HPP

#include "ESM_Transmitter.hpp"
#include "ESM_Receiver.hpp"
#include "ESM_Beam.hpp"

// 单向电磁交互结果（发射源 → 接收机）。
// 实现 Friis 自由空间链路预算、定向天线增益、频率相关检测门限、
// 扫描调度和脉冲信号占空比惩罚。
//
// 用法：
//   ESM_Interaction r;
//   r.transmitter = &xmtr;
//   r.receiver    = &rcvr;
//   r.sim_time    = current_simulation_time;
//   bool ok = r.compute();   // false → 无需交互（发射源未激活或频段外）
//   if (r.detected) { ... }
struct ESM_Interaction
{
    const ESM_Transmitter* transmitter{nullptr};
    const ESM_Receiver*    receiver{nullptr};

    double sim_time{0.0}; // 当前仿真时刻，用于扫描调度检查 (s)

    // --- 几何关系 ---
    double range_m{0.0};       // 距离 (m)
    double azimuth_rad{0.0};   // 接收机→发射源的方位角 (rad, 北=0 顺时针+)
    double elevation_rad{0.0}; // 接收机→发射源的仰角 (rad)

    // --- 链路预算 ---
    double path_loss_dB{0.0};       // 自由空间路径损耗 (dB)
    double antenna_gain_dBi{0.0};   // 接收天线在信号到达方向的增益 (dBi)
    double received_power_dBm{0.0}; // 接收机输入端功率 (dBm)
    double noise_floor_dBm{0.0};    // 接收机噪声底 (dBm)
    double snr_dB{0.0};             // 信噪比，脉冲惩罚前 (dB)

    // --- 传播与环境（接口预留，当前返回默认值） ---
    double propagation_factor{1.0};   // 方向图传播因子 F40（多径效应）；1.0 = 自由空间
    double absorption_factor{1.0};    // 大气吸收 / 透射率 [0..1]；1.0 = 无损耗
    bool   terrain_masked{false};     // 是否被地形遮蔽
    bool   horizon_masked{false};     // 是否被地球曲率 / 地平线遮蔽
    double zone_attenuation_dB{0.0};  // 区域衰减附加损耗 (dB)

    // --- 干扰 ---
    double interference_power_dBm{0.0}; // 接收机输入端的总干扰功率 (dBm)
    double interference_factor{0.0};    // 干扰效应因子 [0..1]；0 = 无干扰

    // --- 多普勒 ---
    double doppler_frequency_hz{0.0}; // 多普勒频移 (Hz)；正 = 接近
    double doppler_speed_mps{0.0};    // 多普勒速度 (m/s)；正 = 接近

    // --- 检测门限 ---
    double effective_threshold_dB{0.0}; // 实际使用的 SNR 门限（来自频率查表或标量）
    double sensitivity_dBm{0.0};        // 最小可检测功率 (dBm)，来自灵敏度查表
    double snr_margin_dB{0.0};          // snr_dB - effective_threshold_dB (> 0 表示可探)

    // --- 状态标记 ---
    bool scan_available{false}; // 接收机在 sim_time 是否处于该频段驻留窗口
    bool detected{false};       // 是否满足全部检测条件

    // 使用 Beam 1（receiver->antenna + frequency_bands）。
    bool compute();

    // 使用指定波束的天线和频段（多波束支持）。
    bool compute(const ESM_Beam& beam);

    // 指向当前使用的波束（compute 后有效）
    const ESM_Beam* active_beam{nullptr};

private:
    void compute_geometry();
    void compute_link_budget(const Antenna& use_antenna,
                             const std::vector<FrequencyBand>& use_bands);

    // --- 传播 / 环境桩函数（当前返回默认值） ---
    void compute_propagation_factor();
    void compute_atmospheric_absorption();
    void compute_terrain_masking();
    void compute_horizon_masking();
    void compute_zone_attenuation();
    void compute_interference();
    void compute_doppler();
};

#endif // ESM_INTERACTION_HPP