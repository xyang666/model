#ifndef ESM_RECEIVER_HPP
#define ESM_RECEIVER_HPP

#include <cmath>
#include <vector>
#include <Eigen/Core>

#include "Antenna.hpp"

// ESM 接收机可监听的一个频段。
// 定义在 ESM_Beam.hpp 之前，因为 ESM_Beam 需要 FrequencyBand。
struct FrequencyBand
{
    FrequencyBand() = default;
    FrequencyBand(double lower, double upper)
        : lower_hz(lower), upper_hz(upper) {}

    double lower_hz{0.0};
    double upper_hz{0.0};
    double dwell_time_s{1.0};   // 每扫描周期在该频段的驻留时长 (s)
    double revisit_time_s{1.0}; // 扫描周期 (s)；须 >= dwell_time_s
};

#include "ESM_Beam.hpp"
#include "ESM_Transmitter.hpp"
#include "FreqTable.hpp"

// 接收机功能分类（对应 AFSIM WsfEM_Rcvr::RcvrFunction）。
enum class RcvrFunction
{
    UNDEFINED,
    COMM,            // 通信接收机
    SENSOR,          // 主动/半主动传感（窄带/匹配频率）
    PASSIVE_SENSOR,  // 被动传感（宽带）
    INTERFERER       // 干扰系统
};

// ESM 接收机：被动、宽带、多频段能力。
struct ESM_Receiver
{
    Eigen::Vector3d position{Eigen::Vector3d::Zero()}; // 位置，ENU 平地坐标系 (m)

    std::vector<FrequencyBand> frequency_bands; // 监听频段列表

    Antenna antenna;                     // 接收天线（方向图 + 波束指向），对应 Beam 1

    // 多波束支持：每个波束独立天线 + 独立频段，对应 AFSIM PassiveBeam
    std::vector<ESM_Beam> beams;

    double noise_figure_dB{5.0};         // 接收机噪声系数 (dB)
    double bandwidth_hz{10e6};           // 瞬时接收带宽 (Hz)
    double detection_threshold_dB{10.0}; // 默认最小 SNR 检测门限 (dB)

    // 频率相关检测门限（频率 → 最小 SNR dB）。
    // 非空时覆盖 detection_threshold_dB 标量值。
    FreqTable detection_threshold_table;

    // 频率相关接收灵敏度（频率 → 最小可检测功率 dBm）。
    // 非空时附加判决条件：received_power_dBm >= sensitivity_dBm(freq)。
    FreqTable sensitivity_table;

    // --- 按信号类型分离的检测门限/灵敏度表（AFSIM 对齐）---
    // CW 检测门限表（频率 → 最小 SNR dB）。
    // 非空时优先于 detection_threshold_table。
    FreqTable cw_detection_threshold_table;

    // PULSED 检测门限表（频率 → 最小 SNR dB）。
    // 非空时优先于 detection_threshold_table。
    FreqTable pulsed_detection_threshold_table;

    // CW 灵敏度表（频率 → 最小可检测功率 dBm）。
    FreqTable cw_sensitivity_table;

    // PULSED 灵敏度表（频率 → 最小可检测功率 dBm）。
    FreqTable pulsed_sensitivity_table;

    // --- 天线 / 馈线损耗 ---
    double antenna_ohmic_loss_dB{0.0};  // 天线欧姆损耗 (dB)，0 = 无损耗
    double receive_line_loss_dB{0.0};   // 接收馈线损耗 (dB)，0 = 无损耗

    // --- 噪声乘数 ---
    double noise_multiplier{1.0};       // 噪声乘数（绝对比值）；1.0 = 标称噪声

    // --------------------------------------------------------------------------
    // PSOS（概率扫描累积探测）
    // 启用后，传感器跨帧累积单次扫描探测概率（PA × PF 模型）。
    // 当累积概率达到 psos_confirm_threshold 时确认探测。
    // --------------------------------------------------------------------------
    bool psos_enabled{false};
    double psos_confirm_threshold{0.9}; // PSOS 确认所需累积 Pd [0..1]
    double psos_frame_time_s{1.0};      // 传感器更新帧周期 (s)，用于 dwell count 计算

    // PSOS 所需累积探测概率。>= 0 时覆盖 psos_confirm_threshold（AFSIM 对齐）。
    double required_pd{-1.0};           // -1 表示使用 psos_confirm_threshold

    // -----------------------------------------------------------------------
    // 接收机特性（AFSIM 对齐扩展）
    // -----------------------------------------------------------------------
    RcvrFunction rcvr_function{RcvrFunction::PASSIVE_SENSOR}; // 功能分类

    // 接收机极化（用于极化匹配损耗计算）
    Polarization polarization{Polarization::NONE};

    // 显式噪声功率 (dBm)，> -1e20 时覆盖公式计算，使用此值作为噪声底。
    // 默认 -1e30 表示"未设置，使用公式计算"。
    double explicit_noise_power_dBm{-1e30};

    // -----------------------------------------------------------------------
    // 查询方法
    // -----------------------------------------------------------------------

    // 频率是否落在任意监听频段内（仅频率检查）。
    bool can_receive(double frequency_hz) const
    {
        for (const auto &band : frequency_bands)
        {
            if (frequency_hz >= band.lower_hz && frequency_hz <= band.upper_hz)
                return true;
        }
        return false;
    }

    // 频率在监听频段内且接收机在 sim_time 正在扫描该频段。
    // 传 sim_time < 0 可跳过扫描调度检查。
    bool is_scanning(double frequency_hz, double sim_time) const
    {
        return is_scanning(frequency_hz, sim_time, frequency_bands);
    }

    // 使用指定频段列表检查扫描状态（多波束支持）。
    static bool is_scanning(double frequency_hz, double sim_time,
                            const std::vector<FrequencyBand>& bands)
    {
        for (const auto &band : bands)
        {
            if (frequency_hz >= band.lower_hz && frequency_hz <= band.upper_hz)
            {
                if (sim_time < 0.0 || band.revisit_time_s <= 0.0)
                    return true;
                double phase = std::fmod(sim_time, band.revisit_time_s);
                return phase < band.dwell_time_s;
            }
        }
        return false;
    }

    // 指定频率和信号类型的检测门限 (dB SNR)（AFSIM 对齐）。
    // 路由逻辑：信号类型对应新表非空 → 新表插值
    //         旧 detection_threshold_table 非空 → 旧表插值
    //         否则 → 标量 detection_threshold_dB
    double get_detection_threshold_dB(double frequency_hz,
                                      SignalType signal_type = SignalType::CW) const
    {
        const FreqTable& sig_table = (signal_type == SignalType::PULSED)
                                         ? pulsed_detection_threshold_table
                                         : cw_detection_threshold_table;
        if (!sig_table.empty())
            return sig_table.lookup(frequency_hz);
        if (!detection_threshold_table.empty())
            return detection_threshold_table.lookup(frequency_hz);
        return detection_threshold_dB;
    }

    // 指定频率的最小可检测功率 (dBm)（AFSIM 对齐）。
    // 路由逻辑同 get_detection_threshold_dB。
    // 所有相关表均为空时返回 -1e30（禁用灵敏度检查）。
    double get_sensitivity_dBm(double frequency_hz,
                                SignalType signal_type = SignalType::CW) const
    {
        const FreqTable& sig_table = (signal_type == SignalType::PULSED)
                                         ? pulsed_sensitivity_table
                                         : cw_sensitivity_table;
        if (!sig_table.empty())
            return sig_table.lookup(frequency_hz);
        if (!sensitivity_table.empty())
            return sensitivity_table.lookup(frequency_hz);
        return -1e30;
    }

    // 接收机输入端噪声底 (dBm)。
    //
    // 优先级：
    //   1. 若 explicit_noise_power_dBm > -1e20（用户显式设定），返回该值。
    //   2. 均无损耗（ohmic_loss=0, line_loss=0）时使用经典公式：
    //        P_noise = -174 + 10*log10(B) + NF
    //   3. 有损耗时使用系统噪声温度模型：
    //        P_noise = 10*log10(k * T_sys * B) + 30
    double noise_floor_dBm(double elevation_rad = 0.0) const
    {
        // 显式噪声功率覆盖
        if (explicit_noise_power_dBm > -1e20)
            return explicit_noise_power_dBm;

        if (antenna_ohmic_loss_dB == 0.0 && receive_line_loss_dB == 0.0)
        {
            // 经典热噪声公式（向后兼容）
            return -174.0 + 10.0 * std::log10(bandwidth_hz) + noise_figure_dB;
        }
        // 系统噪声温度模型
        double t_sys = ComputeSystemNoiseTemperature(elevation_rad,
                                                     antenna_ohmic_loss_dB,
                                                     receive_line_loss_dB,
                                                     noise_figure_dB, 0.0);
        constexpr double k_Boltzmann = 1.380649e-23;
        double noise_watts = k_Boltzmann * t_sys * bandwidth_hz;
        return 10.0 * std::log10(noise_watts) + 30.0; // W → dBm
    }

    // 含噪声乘数的有效噪声底 (dBm)。
    double effective_noise_floor_dBm(double elevation_rad = 0.0) const
    {
        double nf = noise_floor_dBm(elevation_rad);
        if (noise_multiplier > 0.0)
            nf += 10.0 * std::log10(noise_multiplier);
        return nf;
    }

    // 综合系统噪声温度 (K)。
    // 模型:
    //   T_ant(el) = 50 + 20*cos(el)  (el > 0), = 290 K (el <= 0)
    //   L_ohmic   = 10^(ohmic_dB/10)
    //   L_line    = 10^(line_dB/10)
    //   NF_lin    = 10^(NF_dB/10)
    //   T_sys     = T_ant * L_line + T0 * (L_line - 1) + T0 * (NF_lin - 1)
    // 其中 T0 = 290 K
    static double ComputeSystemNoiseTemperature(double elevation_rad,
                                                double antenna_ohmic_loss_db,
                                                double receive_line_loss_db,
                                                double noise_figure_db,
                                                double /*frequency_hz*/)
    {
        constexpr double T0 = 290.0;

        // 天线噪声温度：仰角 > 0 时依赖仰角，否则视为对着地面
        double t_ant;
        if (elevation_rad > 0.0)
            t_ant = 50.0 + 20.0 * std::cos(elevation_rad);
        else
            t_ant = 290.0;

        // 考虑欧姆损耗（增加天线温度）
        if (antenna_ohmic_loss_db > 0.0)
        {
            double l_ohmic = std::pow(10.0, antenna_ohmic_loss_db / 10.0);
            t_ant = t_ant * l_ohmic + T0 * (l_ohmic - 1.0);
        }

        // 馈线损耗
        double l_line = (receive_line_loss_db > 0.0)
                            ? std::pow(10.0, receive_line_loss_db / 10.0)
                            : 1.0;

        // 接收机噪声系数
        double nf_lin = (noise_figure_db > 0.0)
                            ? std::pow(10.0, noise_figure_db / 10.0)
                            : 1.0;

        // 综合系统噪声温度
        return t_ant * l_line + T0 * (l_line - 1.0) + T0 * (nf_lin - 1.0);
    }

    // 带宽效应 (dB)：信号带宽与接收带宽不匹配时的功率损耗。
    // signal_bw_hz <= 0 时返回 0（无损耗）。
    // 模型：B_sig > B_rcv 时，有效接收功率下降 = 10*log10(B_rcv / B_sig)
    double GetBandwidthEffect(double signal_bw_hz) const
    {
        if (signal_bw_hz <= 0.0 || bandwidth_hz <= 0.0)
            return 0.0;
        if (signal_bw_hz <= bandwidth_hz)
            return 0.0;
        return 10.0 * std::log10(bandwidth_hz / signal_bw_hz);
    }

    // 极化效应 (dB)：发射与接收极化不匹配时的信号功率损耗。
    // 模型：
    //   同向极化 → 0 dB
    //   正交线极化（H↔V）→ -30 dB
    //   圆↔线 → -3 dB
    //   反向圆极化（LHCP↔RHCP）→ -30 dB
    //   一方为 NONE → 0 dB（未知极化视为匹配）
    double GetPolarizationEffect(Polarization tx_polarization) const
    {
        if (polarization == Polarization::NONE || tx_polarization == Polarization::NONE)
            return 0.0;
        if (polarization == tx_polarization)
            return 0.0;

        // 正交线极化
        if ((polarization == Polarization::HORIZONTAL && tx_polarization == Polarization::VERTICAL) ||
            (polarization == Polarization::VERTICAL && tx_polarization == Polarization::HORIZONTAL))
            return -30.0;

        // 反向圆极化
        if ((polarization == Polarization::LHCP && tx_polarization == Polarization::RHCP) ||
            (polarization == Polarization::RHCP && tx_polarization == Polarization::LHCP))
            return -30.0;

        // 圆 ↔ 线：3 dB 损耗
        return -3.0;
    }

    // 综合 SNR 计算 (dB)。
    // signal_power_dBm：经极化/带宽修正后的接收功率
    // noise_dBm：噪声底（已包含 noise_multiplier 效应）
    // clutter_dBm：杂波功率（dBm），0 = 无杂波
    // interference_dBm：干扰功率（dBm），0 = 无干扰
    //
    // SNR = 10*log10(S_lin / (N_lin + C_lin + I_lin))
    double ComputeSignalToNoise(double signal_power_dBm,
                                double noise_dBm,
                                double clutter_dBm,
                                double interference_dBm) const
    {
        double s_lin = std::pow(10.0, signal_power_dBm / 10.0);
        double n_lin = std::pow(10.0, noise_dBm / 10.0);
        double c_lin = std::pow(10.0, clutter_dBm / 10.0);
        double i_lin = std::pow(10.0, interference_dBm / 10.0);
        double total = n_lin + c_lin + i_lin;
        if (total <= 0.0)
            return 100.0; // 无噪声场景
        return 10.0 * std::log10(s_lin / total);
    }

    // 根据脉冲宽度更新等效噪声带宽并重新计算噪声底。
    // 短脉冲（窄脉宽）展宽频谱，有效噪声带宽 = max(接收带宽, 1/脉宽)。
    // 此方法修改 bandwidth_hz；调用后后续 noise_floor_dBm() 使用新值。
    // pulse_width_s <= 0 时视为连续波，不做修改。
    void UpdateNoisePower(double pulse_width_s)
    {
        if (pulse_width_s > 0.0)
        {
            double pulse_bw = 1.0 / pulse_width_s;
            if (pulse_bw > bandwidth_hz)
                bandwidth_hz = pulse_bw; // 已展宽，更新有效带宽
        }
    }

    // 遮蔽检查（桩函数）。
    // AFSIM 对应 CheckMasking() / CheckXmtrMasking()。
    // 当前始终返回 false（无障碍）。
    bool CheckMasking() const { return false; }

    // -----------------------------------------------------------------------
    // 交互者管理（接口预留）。
    // AFSIM 对应 AddInteractor / RemoveInteractor / EmitterActiveCallback。
    // ESM 采用每帧传入 transmitter 列表的方式，
    // 此接口保留供框架集成场景使用。
    // -----------------------------------------------------------------------
    using InteractorList = std::vector<const ESM_Transmitter*>;

    InteractorList comm_interactors;
    InteractorList sensor_interactors;
    InteractorList interference_interactors;
};

#endif // ESM_RECEIVER_HPP