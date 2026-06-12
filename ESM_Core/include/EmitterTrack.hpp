#ifndef EMITTER_TRACK_HPP
#define EMITTER_TRACK_HPP

#include <string>
#include <vector>

// 每次检测时捕获的信号参数
struct SignalData
{
    double      frequency_lower_hz{0.0};
    double      frequency_upper_hz{0.0};
    double      pulse_width_s{0.0};
    double      pulse_repetition_interval_s{0.0};
    std::string emitter_truth_type;
};

// ESM 传感器维护的已探测辐射源航迹记录。
struct EmitterTrack
{
    std::string emitter_id;      // 已探测发射源的 ID

    double frequency_hz{0.0};    // 探测到的载波频率 (Hz)
    double azimuth_rad{0.0};     // 接收机到发射源的方位角 (rad)
    double elevation_rad{0.0};   // 到发射源的仰角 (rad)
    double range_m{0.0};         // 距离估计（0 = 未知）

    double received_power_dBm{0.0}; // 最近一次测量的接收功率 (dBm)
    double antenna_gain_dBi{0.0};   // 最近一次探测使用的接收天线增益 (dBi)
    double snr_dB{0.0};             // 最近一次测量的 SNR (dB)

    double first_detect_time{0.0};  // 首次探测的仿真时刻 (s)
    double last_detect_time{0.0};   // 最近一次成功探测的仿真时刻 (s)
    double last_update_time{0.0};   // 最近一次带内观测的仿真时刻 (s)
    int    detect_count{0};         // 累计成功探测次数

    // ------------------------------------------------------------------
    // 信号数据捕获（每次成功探测时记录）
    // ------------------------------------------------------------------
    std::vector<SignalData> signal_history;

    // ------------------------------------------------------------------
    // 报告状态（发射器类型识别 + 目标平台分类）
    // ------------------------------------------------------------------
    std::string declared_emitter_type;   // 声明的发射器类型
    double      declared_confidence{0.0}; // 声明置信度 [0, 1]
    bool        type_declared{false};     // 是否已首次声明类型
    std::string declared_target_type;     // 声明的目标平台类型

    // ------------------------------------------------------------------
    // PSOS（概率扫描累积探测）状态
    // 当 psos_enabled 为 true 时由 ESM_Sensor 维护。
    // 使用扫描重叠模型（PA × PF），见 ESM_Sensor::compute_psos()。
    // ------------------------------------------------------------------
    double psos_cumulative_pd{0.0}; // 累积检测概率 P_det = 1 - ∏(1 - PSS_frame)
    bool   psos_confirmed{false};   // 累积 Pd >= 确认阈值后置 true
};

#endif // EMITTER_TRACK_HPP
