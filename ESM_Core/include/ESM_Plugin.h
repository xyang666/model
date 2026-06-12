// ****************************************************************************
// ESM_Plugin.h - 电子侦察模型内部实现类（不导出，外部调用者不可见）
//
// 封装 ESM_Sensor 及相关类型，对外仅暴露 C-API。
// ****************************************************************************
#pragma once

#include "stdafx.h"

#include "ESM_Sensor.hpp"
#include <map>
#include <string>
#include <vector>

class ESM_Plugin
{
public:
    bool Initialize();
    bool Finalize();

    // --- 接收机配置 ---
    void SetReceiverPosition(double x, double y, double z);
    void SetReceiverNoiseFigure(double nf_dB);
    void SetReceiverBandwidth(double bw_hz);
    void SetReceiverDetectionThreshold(double thr_dB);
    void SetCoastTime(double coast_s);

    // --- 天线配置 ---
    void SetAntennaPattern(int type, double peak_gain_dBi,
                           double az_bw_rad, double el_bw_rad,
                           double back_lobe_dB);
    void SetAntennaBoresight(double az_rad, double el_rad);
    void SetAntennaFov(double min_az_rad, double max_az_rad,
                       double min_el_rad, double max_el_rad,
                       double min_range_m, double max_range_m);

    // --- 频段管理 ---
    void AddFrequencyBand(double lower_hz, double upper_hz,
                          double dwell_s, double revisit_s);
    void ClearFrequencyBands();

    // --- 辐射源管理 ---
    int  AddTransmitter(const char* id,
                        double pos_x, double pos_y, double pos_z,
                        double freq_hz, double power_dBm, double gain_dBi);
    void SetTransmitterSignal(const char* id, int signal_type,
                              double duty_cycle, double signal_bw_hz);
    void SetTransmitterPolarization(const char* id, int polarization);
    void SetTransmitterPulse(const char* id,
                             double pulse_width_s, double prf_hz,
                             double pri_s, double pcr);
    void SetTransmitterTxAntenna(const char* id,
                                 int pattern, double gain_dBi,
                                 double bw_az_rad, double bw_el_rad,
                                 int scan_mode,
                                 double scan_min_az, double scan_max_az,
                                 double scan_min_el, double scan_max_el);
    void SetTransmitterActive(const char* id, bool active);
    void SetTransmitterUsePeakPower(const char* id, bool use_peak);
    void RemoveTransmitter(const char* id);
    void ClearTransmitters();
    int  GetTransmitterCount() const;
    const char* GetTransmitterId(int index) const;

    // --- 仿真 ---
    void Update(double sim_time);
    int  AttemptDetect(int tx_index, double sim_time,
                       double* out_snr_dB, double* out_range_m,
                       double* out_az_rad, double* out_el_rad,
                       double* out_rx_power_dBm, double* out_ant_gain_dBi,
                       double* out_path_loss_dB);

    // --- 航迹查询 ---
    int         GetTrackCount() const;
    const char* GetTrackId(int index) const;
    int         GetTrackData(int index,
                             double* freq_hz, double* az_rad, double* el_rad,
                             double* range_m, double* rx_power_dBm,
                             double* ant_gain_dBi, double* snr_dB,
                             double* first_detect_time, double* last_detect_time,
                             double* last_update_time, int* detect_count,
                             double* psos_cumulative_pd, int* psos_confirmed) const;

    // --- PSOS 配置 ---
    void SetPsosEnabled(bool enabled);
    void SetPsosConfirmThreshold(double threshold);
    void SetPsosFrameTime(double frame_time_s);
    void SetPsosRequiredPd(double required_pd);

    // --- 情报层（Phase 1） ---
    void AddEmitterReportRule(const char* emitter_type,
                              double time_to_declare, double time_to_reevaluate,
                              bool report_truth);
    void AddEmitterReportEntry(const char* emitter_type,
                               const char* reported_name, double confidence);
    void AddTargetReportRule_Emitters(const char* target_type,
                                      const char** emitter_ids, int emitter_count,
                                      const char* declared_target);
    void SetTargetId(const char* target_id);
    void SetMeasurementError(double az_sigma_rad, double el_sigma_rad,
                             double range_sigma_m);
    void SetReportsFrequency(bool enabled);
    void SetReportsPulsewidth(bool enabled);
    void SetReportsPri(bool enabled);
    const char* GetTrackDeclaredType(int index) const;
    double      GetTrackDeclaredConfidence(int index) const;
    const char* GetTrackDeclaredTargetType(int index) const;

    // --- 多波束管理 ---
    int  AddBeam();
    void RemoveBeam(int beam_index);
    int  GetBeamCount() const;
    void SetBeamAntennaPattern(int beam_index, int type, double peak_gain_dBi,
                               double az_bw_rad, double el_bw_rad, double back_lobe_dB);
    void SetBeamAntennaBoresight(int beam_index, double az_rad, double el_rad);
    void SetBeamAntennaFov(int beam_index,
                           double min_az, double max_az,
                           double min_el, double max_el,
                           double min_range, double max_range);
    void AddBeamFrequencyBand(int beam_index,
                              double lower_hz, double upper_hz,
                              double dwell_s, double revisit_s);

    // --- 航迹管理层（Phase 2） ---
    void SetTrackInitMofN(int m, int n);
    void SetTrackMaintMofN(int m, int n, double coast_s);

    // 检查刚刚声明的发射器/目标（供 ESM_Framework 事件生成用）
    bool EmitterJustDeclared(const char* emitter_id);
    bool TargetJustDeclared();
    // 检查刚刚建立/丢失的航迹（供 ESM_Framework 事件生成用）
    bool TrackJustInitiated(const char* emitter_id);
    bool TrackJustDropped(const char* emitter_id);
    // 获取本帧刚刚丢失的航迹 ID 列表
    std::vector<std::string> GetJustDroppedIds();

private:
    ESM_Sensor                     m_sensor;
    std::vector<ESM_Transmitter>    m_transmitters;
    std::map<std::string, int>     m_id_to_index;
    bool                           m_initialized{false};

    void rebuild_id_map();
    int  find_transmitter_index(const char* id) const;
};

// 全局单例，供框架层直接访问
extern ESM_Plugin g_plugin;
