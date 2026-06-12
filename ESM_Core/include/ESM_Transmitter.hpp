#ifndef ESM_TRANSMITTER_HPP
#define ESM_TRANSMITTER_HPP

#include <string>
#include <Eigen/Core>

// 信号发射类型。
enum class SignalType
{
    CW,     // 连续波：始终发射
    PULSED  // 脉冲：仅部分时间发射
};

// 极化类型。
enum class Polarization
{
    NONE,       // 未指定 / 不适用
    HORIZONTAL, // 水平极化
    VERTICAL,   // 垂直极化
    LHCP,       // 左旋圆极化
    RHCP        // 右旋圆极化
};

// 表示一个射频辐射源（雷达、干扰机、通信发射机等）。
struct ESM_Transmitter
{
    std::string id;

    Eigen::Vector3d position{Eigen::Vector3d::Zero()}; // 位置，ENU 平地坐标系 (m)

    double frequency_hz{1e9};      // 载波频率 (Hz)
    double power_dBm{30.0};        // 天线端口峰值发射功率 (dBm)
    double antenna_gain_dBi{0.0};  // 发射天线增益 (dBi, 全向 = 0)
    double signal_bandwidth_hz{0.0}; // 发射信号带宽 (Hz)；0 = 使用接收机带宽匹配

    SignalType signal_type{SignalType::CW};
    double duty_cycle{1.0}; // 发射占空比 [0..1]；仅 PULSED 时生效

    // --- 脉冲参数（仅 PULSED 时有效，接口预留） ---
    double pulse_width_s{0.0};                  // 脉冲宽度 (s)；0 = CW
    double pulse_repetition_frequency_hz{0.0};  // 脉冲重复频率 PRF (Hz)；0 = CW
    double pulse_repetition_interval_s{0.0};    // 脉冲重复间隔 PRI (s)；0 = CW
    double pulse_compression_ratio{1.0};        // 脉冲压缩比（绝对比值，非 dB）；1.0 = 无压缩

    // --- 功率设定 ---
    bool use_peak_power{false};  // true = GetPower 返回峰值功率；false = 平均功率（默认）

    // --- 极化 ---
    Polarization polarization{Polarization::NONE};

    // --- 发射天线方向图（用于 PSOS 方位重叠概率 PA 计算） ---
    struct TxAntennaModel
    {
        enum class Pattern { ISOTROPIC, GAUSSIAN, SINC2 };

        Pattern pattern{Pattern::ISOTROPIC};
        double  gain_dBi{0.0};        // 峰值增益 (dBi)，建议设为 antenna_gain_dBi 相同值
        double  beamwidth_az_rad{1.5708}; // 方位 3dB 波束宽度 (rad)
        double  beamwidth_el_rad{1.5708}; // 仰角 3dB 波束宽度 (rad)
    };
    TxAntennaModel tx_antenna;

    // --- 发射天线扫描模式 ---
    enum class ScanMode { FIXED, AZ_SCAN, EL_SCAN, AZ_EL_SCAN };
    ScanMode scan_mode{ScanMode::FIXED};
    double scan_min_az_rad{ -kPi};
    double scan_max_az_rad{  kPi};
    double scan_min_el_rad{-kPi_2};
    double scan_max_el_rad{ kPi_2};

    bool active{true};

    static constexpr double kPi   = 3.14159265358979323846;
    static constexpr double kPi_2 = 1.57079632679489661923;

    // 等效全向辐射功率 (dBm) — 峰值
    double eirp_dBm() const { return power_dBm + antenna_gain_dBi; }

    // 平均功率 (dBm)，含占空比修正
    double average_power_dBm() const
    {
        if (signal_type == SignalType::PULSED && duty_cycle > 0.0 && duty_cycle < 1.0)
            return power_dBm + 10.0 * std::log10(duty_cycle);
        return power_dBm;
    }

    // 按 use_peak_power 返回峰值或平均功率 (dBm)
    double get_power_dBm() const
    {
        return use_peak_power ? power_dBm : average_power_dBm();
    }
};

#endif // ESM_TRANSMITTER_HPP