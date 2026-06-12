// ****************************************************************************
// ESM_Plugin.cpp - 电子侦察模型 DLL 实现
//
// 包含：
//   1. DllMain 入口点
//   2. ESM_Plugin 内部类的具体实现（业务逻辑）
//   3. 全局单例
//   4. C-API 包装函数（桥接对外接口与内部实现）
// ****************************************************************************

#include "stdafx.h"
#include "ESM_Plugin.h"
#include "ESM_Interface.h"

#include <cstring>

// ============================================================================
// DLL 入口点
// ============================================================================
#ifdef WIN32
BOOL APIENTRY DllMain(HANDLE hModule,
    DWORD  ul_reason_for_call,
    LPVOID lpReserved)
{
    return TRUE;
}
#endif

// ============================================================================
// 内部辅助函数：枚举映射
// ============================================================================

static SignalType toSignalType(int v)
{
    return static_cast<SignalType>(v);
}

static Polarization toPolarization(int v)
{
    return static_cast<Polarization>(v);
}

static AntennaPattern::Type toAntennaPatternType(int v)
{
    return static_cast<AntennaPattern::Type>(v);
}

static ESM_Transmitter::TxAntennaModel::Pattern toTxPattern(int v)
{
    return static_cast<ESM_Transmitter::TxAntennaModel::Pattern>(v);
}

static ESM_Transmitter::ScanMode toScanMode(int v)
{
    return static_cast<ESM_Transmitter::ScanMode>(v);
}

// ============================================================================
// ESM_Plugin 实现
// ============================================================================

bool ESM_Plugin::Initialize()
{
    m_sensor = ESM_Sensor();
    m_transmitters.clear();
    m_id_to_index.clear();
    m_initialized = true;
    return true;
}

bool ESM_Plugin::Finalize()
{
    m_transmitters.clear();
    m_id_to_index.clear();
    m_initialized = false;
    return true;
}

// --- 接收机配置 ---

void ESM_Plugin::SetReceiverPosition(double x, double y, double z)
{
    m_sensor.receiver.position = Eigen::Vector3d(x, y, z);
}

void ESM_Plugin::SetReceiverNoiseFigure(double nf_dB)
{
    m_sensor.receiver.noise_figure_dB = nf_dB;
}

void ESM_Plugin::SetReceiverBandwidth(double bw_hz)
{
    m_sensor.receiver.bandwidth_hz = bw_hz;
}

void ESM_Plugin::SetReceiverDetectionThreshold(double thr_dB)
{
    m_sensor.receiver.detection_threshold_dB = thr_dB;
}

void ESM_Plugin::SetCoastTime(double coast_s)
{
    m_sensor.coast_time = coast_s;
}

// --- 天线配置 ---

void ESM_Plugin::SetAntennaPattern(int type, double peak_gain_dBi,
                                   double az_bw_rad, double el_bw_rad,
                                   double back_lobe_dB)
{
    m_sensor.receiver.antenna.pattern.type = toAntennaPatternType(type);
    m_sensor.receiver.antenna.pattern.peak_gain_dBi = peak_gain_dBi;
    m_sensor.receiver.antenna.pattern.az_beamwidth_rad = az_bw_rad;
    m_sensor.receiver.antenna.pattern.el_beamwidth_rad = el_bw_rad;
    m_sensor.receiver.antenna.pattern.back_lobe_floor_dB = back_lobe_dB;
}

void ESM_Plugin::SetAntennaBoresight(double az_rad, double el_rad)
{
    m_sensor.receiver.antenna.boresight_az_rad = az_rad;
    m_sensor.receiver.antenna.boresight_el_rad = el_rad;
}

void ESM_Plugin::SetAntennaFov(double min_az_rad, double max_az_rad,
                               double min_el_rad, double max_el_rad,
                               double min_range_m, double max_range_m)
{
    m_sensor.receiver.antenna.fov_min_az_rad = min_az_rad;
    m_sensor.receiver.antenna.fov_max_az_rad = max_az_rad;
    m_sensor.receiver.antenna.fov_min_el_rad = min_el_rad;
    m_sensor.receiver.antenna.fov_max_el_rad = max_el_rad;
    m_sensor.receiver.antenna.min_range_m = min_range_m;
    m_sensor.receiver.antenna.max_range_m = max_range_m;
}

// --- 频段管理 ---

void ESM_Plugin::AddFrequencyBand(double lower_hz, double upper_hz,
                                  double dwell_s, double revisit_s)
{
    FrequencyBand band(lower_hz, upper_hz);
    band.dwell_time_s = dwell_s;
    band.revisit_time_s = revisit_s;
    m_sensor.receiver.frequency_bands.push_back(band);
}

void ESM_Plugin::ClearFrequencyBands()
{
    m_sensor.receiver.frequency_bands.clear();
}

// --- 辐射源管理 ---

void ESM_Plugin::rebuild_id_map()
{
    m_id_to_index.clear();
    for (int i = 0; i < static_cast<int>(m_transmitters.size()); ++i)
        m_id_to_index[m_transmitters[i].id] = i;
}

int ESM_Plugin::find_transmitter_index(const char* id) const
{
    auto it = m_id_to_index.find(std::string(id));
    if (it != m_id_to_index.end())
        return it->second;
    return -1;
}

int ESM_Plugin::AddTransmitter(const char* id,
                               double pos_x, double pos_y, double pos_z,
                               double freq_hz, double power_dBm, double gain_dBi)
{
    ESM_Transmitter xmtr;
    xmtr.id = id;
    xmtr.position = Eigen::Vector3d(pos_x, pos_y, pos_z);
    xmtr.frequency_hz = freq_hz;
    xmtr.power_dBm = power_dBm;
    xmtr.antenna_gain_dBi = gain_dBi;
    m_transmitters.push_back(xmtr);
    rebuild_id_map();
    return static_cast<int>(m_transmitters.size()) - 1;
}

void ESM_Plugin::SetTransmitterSignal(const char* id, int signal_type,
                                      double duty_cycle, double signal_bw_hz)
{
    int idx = find_transmitter_index(id);
    if (idx < 0) return;
    ESM_Transmitter& xmtr = m_transmitters[idx];
    xmtr.signal_type = toSignalType(signal_type);
    xmtr.duty_cycle = duty_cycle;
    xmtr.signal_bandwidth_hz = signal_bw_hz;
}

void ESM_Plugin::SetTransmitterPolarization(const char* id, int polarization)
{
    int idx = find_transmitter_index(id);
    if (idx < 0) return;
    m_transmitters[idx].polarization = toPolarization(polarization);
}

void ESM_Plugin::SetTransmitterPulse(const char* id,
                                     double pulse_width_s, double prf_hz,
                                     double pri_s, double pcr)
{
    int idx = find_transmitter_index(id);
    if (idx < 0) return;
    ESM_Transmitter& xmtr = m_transmitters[idx];
    xmtr.pulse_width_s = pulse_width_s;
    xmtr.pulse_repetition_frequency_hz = prf_hz;
    xmtr.pulse_repetition_interval_s = pri_s;
    xmtr.pulse_compression_ratio = pcr;
}

void ESM_Plugin::SetTransmitterTxAntenna(const char* id,
                                         int pattern, double gain_dBi,
                                         double bw_az_rad, double bw_el_rad,
                                         int scan_mode,
                                         double scan_min_az, double scan_max_az,
                                         double scan_min_el, double scan_max_el)
{
    int idx = find_transmitter_index(id);
    if (idx < 0) return;
    ESM_Transmitter& xmtr = m_transmitters[idx];
    xmtr.tx_antenna.pattern = toTxPattern(pattern);
    xmtr.tx_antenna.gain_dBi = gain_dBi;
    xmtr.tx_antenna.beamwidth_az_rad = bw_az_rad;
    xmtr.tx_antenna.beamwidth_el_rad = bw_el_rad;
    xmtr.scan_mode = toScanMode(scan_mode);
    xmtr.scan_min_az_rad = scan_min_az;
    xmtr.scan_max_az_rad = scan_max_az;
    xmtr.scan_min_el_rad = scan_min_el;
    xmtr.scan_max_el_rad = scan_max_el;
}

void ESM_Plugin::SetTransmitterActive(const char* id, bool active)
{
    int idx = find_transmitter_index(id);
    if (idx < 0) return;
    m_transmitters[idx].active = active;
}

void ESM_Plugin::SetTransmitterUsePeakPower(const char* id, bool use_peak)
{
    int idx = find_transmitter_index(id);
    if (idx < 0) return;
    m_transmitters[idx].use_peak_power = use_peak;
}

void ESM_Plugin::RemoveTransmitter(const char* id)
{
    int idx = find_transmitter_index(id);
    if (idx < 0) return;
    m_transmitters.erase(m_transmitters.begin() + idx);
    rebuild_id_map();
}

void ESM_Plugin::ClearTransmitters()
{
    m_transmitters.clear();
    m_id_to_index.clear();
}

int ESM_Plugin::GetTransmitterCount() const
{
    return static_cast<int>(m_transmitters.size());
}

const char* ESM_Plugin::GetTransmitterId(int index) const
{
    if (index < 0 || index >= static_cast<int>(m_transmitters.size()))
        return "";
    return m_transmitters[index].id.c_str();
}

// --- 仿真 ---

void ESM_Plugin::Update(double sim_time)
{
    m_sensor.update(sim_time, m_transmitters);
}

int ESM_Plugin::AttemptDetect(int tx_index, double sim_time,
                              double* out_snr_dB, double* out_range_m,
                              double* out_az_rad, double* out_el_rad,
                              double* out_rx_power_dBm, double* out_ant_gain_dBi,
                              double* out_path_loss_dB)
{
    if (tx_index < 0 || tx_index >= static_cast<int>(m_transmitters.size()))
        return -1;

    ESM_Interaction r = m_sensor.attempt_detect(m_transmitters[tx_index], sim_time);
    if (!r.transmitter)
        return -1;

    if (out_snr_dB)        *out_snr_dB        = r.snr_dB;
    if (out_range_m)       *out_range_m       = r.range_m;
    if (out_az_rad)        *out_az_rad        = r.azimuth_rad;
    if (out_el_rad)        *out_el_rad        = r.elevation_rad;
    if (out_rx_power_dBm)  *out_rx_power_dBm  = r.received_power_dBm;
    if (out_ant_gain_dBi)  *out_ant_gain_dBi  = r.antenna_gain_dBi;
    if (out_path_loss_dB)  *out_path_loss_dB  = r.path_loss_dB;

    return r.detected ? 1 : 0;
}

// --- 航迹查询 ---

int ESM_Plugin::GetTrackCount() const
{
    return static_cast<int>(m_sensor.tracks().size());
}

const char* ESM_Plugin::GetTrackId(int index) const
{
    const auto& tracks = m_sensor.tracks();
    if (index < 0 || index >= static_cast<int>(tracks.size()))
        return "";
    return tracks[index].emitter_id.c_str();
}

int ESM_Plugin::GetTrackData(int index,
                             double* freq_hz, double* az_rad, double* el_rad,
                             double* range_m, double* rx_power_dBm,
                             double* ant_gain_dBi, double* snr_dB,
                             double* first_detect_time, double* last_detect_time,
                             double* last_update_time, int* detect_count,
                             double* psos_cumulative_pd, int* psos_confirmed) const
{
    const auto& tracks = m_sensor.tracks();
    if (index < 0 || index >= static_cast<int>(tracks.size()))
        return -1;

    const EmitterTrack& t = tracks[index];
    if (freq_hz)            *freq_hz            = t.frequency_hz;
    if (az_rad)             *az_rad             = t.azimuth_rad;
    if (el_rad)             *el_rad             = t.elevation_rad;
    if (range_m)            *range_m            = t.range_m;
    if (rx_power_dBm)       *rx_power_dBm       = t.received_power_dBm;
    if (ant_gain_dBi)       *ant_gain_dBi       = t.antenna_gain_dBi;
    if (snr_dB)             *snr_dB             = t.snr_dB;
    if (first_detect_time)  *first_detect_time  = t.first_detect_time;
    if (last_detect_time)   *last_detect_time   = t.last_detect_time;
    if (last_update_time)   *last_update_time   = t.last_update_time;
    if (detect_count)       *detect_count       = t.detect_count;
    if (psos_cumulative_pd) *psos_cumulative_pd = t.psos_cumulative_pd;
    if (psos_confirmed)     *psos_confirmed     = t.psos_confirmed ? 1 : 0;
    return 0;
}

// --- PSOS 配置 ---

void ESM_Plugin::SetPsosEnabled(bool enabled)
{
    m_sensor.receiver.psos_enabled = enabled;
}

void ESM_Plugin::SetPsosConfirmThreshold(double threshold)
{
    m_sensor.receiver.psos_confirm_threshold = threshold;
}

void ESM_Plugin::SetPsosFrameTime(double frame_time_s)
{
    m_sensor.receiver.psos_frame_time_s = frame_time_s;
}

void ESM_Plugin::SetPsosRequiredPd(double required_pd)
{
    m_sensor.receiver.required_pd = required_pd;
}

// --- 情报层（Phase 1） ---

void ESM_Plugin::AddEmitterReportRule(const char* emitter_type,
                                      double time_to_declare,
                                      double time_to_reevaluate,
                                      bool report_truth)
{
    EmitterReportRule rule;
    rule.emitter_truth_type = emitter_type;
    rule.time_to_declare    = time_to_declare;
    rule.time_to_reevaluate = time_to_reevaluate;
    rule.report_truth       = report_truth;
    m_sensor.emitter_reporting().AddRule(rule);
}

void ESM_Plugin::AddEmitterReportEntry(const char* emitter_type,
                                       const char* reported_name,
                                       double confidence)
{
    // 查找已存在的规则并添加条目
    auto& reporting = m_sensor.emitter_reporting();
    EmitterReportRule rule;
    rule.emitter_truth_type = emitter_type;
    rule.time_to_declare    = 5.0;
    rule.report_truth       = false;
    rule.report_table.push_back({reported_name, confidence});
    reporting.AddRule(rule);
}

void ESM_Plugin::AddTargetReportRule_Emitters(const char* target_type,
                                              const char** emitter_ids,
                                              int emitter_count,
                                              const char* declared_target)
{
    TargetReportRule rule;
    rule.target_truth_type = target_type;
    std::vector<std::string> ids;
    for (int i = 0; i < emitter_count; ++i)
        ids.push_back(emitter_ids[i]);
    rule.emitter_to_target.push_back({ids, declared_target});
    m_sensor.target_reporting().AddRule(rule);
}

void ESM_Plugin::SetTargetId(const char* target_id)
{
    m_sensor.target_id = target_id;
}

void ESM_Plugin::SetMeasurementError(double az_sigma_rad, double el_sigma_rad,
                                     double range_sigma_m)
{
    MeasurementErrorConfig cfg;
    cfg.azimuth_error_sigma_rad   = az_sigma_rad;
    cfg.elevation_error_sigma_rad = el_sigma_rad;
    cfg.range_error_m             = range_sigma_m;
    m_sensor.error_model().Configure(cfg);
}

void ESM_Plugin::SetReportsFrequency(bool enabled)
{
    m_sensor.reports_frequency = enabled;
}

void ESM_Plugin::SetReportsPulsewidth(bool enabled)
{
    m_sensor.reports_pulsewidth = enabled;
}

void ESM_Plugin::SetReportsPri(bool enabled)
{
    m_sensor.reports_pri = enabled;
}

const char* ESM_Plugin::GetTrackDeclaredType(int index) const
{
    const auto& tracks = m_sensor.tracks();
    if (index < 0 || index >= static_cast<int>(tracks.size()))
        return "";
    return tracks[index].declared_emitter_type.c_str();
}

double ESM_Plugin::GetTrackDeclaredConfidence(int index) const
{
    const auto& tracks = m_sensor.tracks();
    if (index < 0 || index >= static_cast<int>(tracks.size()))
        return 0.0;
    return tracks[index].declared_confidence;
}

const char* ESM_Plugin::GetTrackDeclaredTargetType(int index) const
{
    const auto& tracks = m_sensor.tracks();
    if (index < 0 || index >= static_cast<int>(tracks.size()))
        return "";
    return tracks[index].declared_target_type.c_str();
}

bool ESM_Plugin::EmitterJustDeclared(const char* emitter_id)
{
    return m_sensor.emitter_reporting().JustDeclared(emitter_id);
}

bool ESM_Plugin::TargetJustDeclared()
{
    return m_sensor.target_reporting().JustDeclared(m_sensor.target_id);
}

void ESM_Plugin::SetTrackInitMofN(int m, int n)
{
    m_sensor.SetTrackInitMofN(m, n);
}

void ESM_Plugin::SetTrackMaintMofN(int m, int n, double coast_s)
{
    m_sensor.SetTrackMaintMofN(m, n, coast_s);
}

bool ESM_Plugin::TrackJustInitiated(const char* emitter_id)
{
    return m_sensor.track_manager().JustInitiated(emitter_id);
}

bool ESM_Plugin::TrackJustDropped(const char* emitter_id)
{
    return m_sensor.track_manager().JustDropped(emitter_id);
}

std::vector<std::string> ESM_Plugin::GetJustDroppedIds()
{
    return m_sensor.GetJustDroppedIds();
}

// --- 多波束管理 ---

int ESM_Plugin::AddBeam()
{
    ESM_Beam beam;
    beam.beam_index = static_cast<int>(m_sensor.receiver.beams.size()) + 1;
    m_sensor.receiver.beams.push_back(beam);
    return beam.beam_index;
}

void ESM_Plugin::RemoveBeam(int beam_index)
{
    if (beam_index <= 0 || beam_index > static_cast<int>(m_sensor.receiver.beams.size()))
        return;
    m_sensor.receiver.beams.erase(
        m_sensor.receiver.beams.begin() + (beam_index - 1));
}

int ESM_Plugin::GetBeamCount() const
{
    return static_cast<int>(m_sensor.receiver.beams.size());
}

void ESM_Plugin::SetBeamAntennaPattern(int beam_index, int type,
                                        double peak_gain_dBi,
                                        double az_bw_rad, double el_bw_rad,
                                        double back_lobe_dB)
{
    if (beam_index < 1 || beam_index > static_cast<int>(m_sensor.receiver.beams.size()))
        return;
    AntennaPattern& pat = m_sensor.receiver.beams[beam_index - 1].antenna.pattern;
    pat.type = toAntennaPatternType(type);
    pat.peak_gain_dBi    = peak_gain_dBi;
    pat.az_beamwidth_rad = az_bw_rad;
    pat.el_beamwidth_rad = el_bw_rad;
    pat.back_lobe_floor_dB = back_lobe_dB;
}

void ESM_Plugin::SetBeamAntennaBoresight(int beam_index, double az_rad, double el_rad)
{
    if (beam_index < 1 || beam_index > static_cast<int>(m_sensor.receiver.beams.size()))
        return;
    Antenna& ant = m_sensor.receiver.beams[beam_index - 1].antenna;
    ant.boresight_az_rad = az_rad;
    ant.boresight_el_rad = el_rad;
}

void ESM_Plugin::SetBeamAntennaFov(int beam_index,
                                    double min_az, double max_az,
                                    double min_el, double max_el,
                                    double min_range, double max_range)
{
    if (beam_index < 1 || beam_index > static_cast<int>(m_sensor.receiver.beams.size()))
        return;
    Antenna& ant = m_sensor.receiver.beams[beam_index - 1].antenna;
    ant.fov_min_az_rad = min_az;
    ant.fov_max_az_rad = max_az;
    ant.fov_min_el_rad = min_el;
    ant.fov_max_el_rad = max_el;
    ant.min_range_m    = min_range;
    ant.max_range_m    = max_range;
}

void ESM_Plugin::AddBeamFrequencyBand(int beam_index,
                                       double lower_hz, double upper_hz,
                                       double dwell_s, double revisit_s)
{
    if (beam_index < 1 || beam_index > static_cast<int>(m_sensor.receiver.beams.size()))
        return;
    FrequencyBand band(lower_hz, upper_hz);
    band.dwell_time_s   = dwell_s;
    band.revisit_time_s = revisit_s;
    m_sensor.receiver.beams[beam_index - 1].frequency_bands.push_back(band);
}

// ============================================================================
// 全局单例
// ============================================================================

ESM_Plugin g_plugin;

// ============================================================================
// C-API 包装函数 —— 对外暴露的唯一入口
//
// 注意：此处不需要 DLL_EXPORT，因为头文件中的声明已标注
// ============================================================================

extern "C"
{

// ========================================================================
// 模型初始化接口
// ========================================================================
DLL_EXPORT bool Initialize()
{
    return g_plugin.Initialize();
}

// ========================================================================
// 模型结束释放接口
// ========================================================================
DLL_EXPORT bool Finalize()
{
    return g_plugin.Finalize();
}

// ========================================================================
// 接收机配置
// ========================================================================

DLL_EXPORT void ESM_SetReceiverPosition(double x, double y, double z)
{
    g_plugin.SetReceiverPosition(x, y, z);
}

DLL_EXPORT void ESM_SetReceiverNoiseFigure(double nf_dB)
{
    g_plugin.SetReceiverNoiseFigure(nf_dB);
}

DLL_EXPORT void ESM_SetReceiverBandwidth(double bw_hz)
{
    g_plugin.SetReceiverBandwidth(bw_hz);
}

DLL_EXPORT void ESM_SetReceiverDetectionThreshold(double thr_dB)
{
    g_plugin.SetReceiverDetectionThreshold(thr_dB);
}

DLL_EXPORT void ESM_SetCoastTime(double coast_s)
{
    g_plugin.SetCoastTime(coast_s);
}

// ========================================================================
// 天线配置
// ========================================================================

DLL_EXPORT void ESM_SetAntennaPattern(int type, double peak_gain_dBi,
                                      double az_bw_rad, double el_bw_rad,
                                      double back_lobe_dB)
{
    g_plugin.SetAntennaPattern(type, peak_gain_dBi,
                               az_bw_rad, el_bw_rad, back_lobe_dB);
}

DLL_EXPORT void ESM_SetAntennaBoresight(double az_rad, double el_rad)
{
    g_plugin.SetAntennaBoresight(az_rad, el_rad);
}

DLL_EXPORT void ESM_SetAntennaFov(double min_az_rad, double max_az_rad,
                                  double min_el_rad, double max_el_rad,
                                  double min_range_m, double max_range_m)
{
    g_plugin.SetAntennaFov(min_az_rad, max_az_rad,
                           min_el_rad, max_el_rad,
                           min_range_m, max_range_m);
}

// ========================================================================
// 频段管理
// ========================================================================

DLL_EXPORT void ESM_AddFrequencyBand(double lower_hz, double upper_hz,
                                     double dwell_s, double revisit_s)
{
    g_plugin.AddFrequencyBand(lower_hz, upper_hz, dwell_s, revisit_s);
}

DLL_EXPORT void ESM_ClearFrequencyBands()
{
    g_plugin.ClearFrequencyBands();
}

// ========================================================================
// 辐射源管理
// ========================================================================

DLL_EXPORT int ESM_AddTransmitter(const char* id,
                                  double pos_x, double pos_y, double pos_z,
                                  double freq_hz, double power_dBm, double gain_dBi)
{
    return g_plugin.AddTransmitter(id, pos_x, pos_y, pos_z,
                                   freq_hz, power_dBm, gain_dBi);
}

DLL_EXPORT void ESM_SetTransmitterSignal(const char* id, int signal_type,
                                         double duty_cycle, double signal_bw_hz)
{
    g_plugin.SetTransmitterSignal(id, signal_type, duty_cycle, signal_bw_hz);
}

DLL_EXPORT void ESM_SetTransmitterPolarization(const char* id, int polarization)
{
    g_plugin.SetTransmitterPolarization(id, polarization);
}

DLL_EXPORT void ESM_SetTransmitterPulse(const char* id,
                                        double pulse_width_s, double prf_hz,
                                        double pri_s, double pcr)
{
    g_plugin.SetTransmitterPulse(id, pulse_width_s, prf_hz, pri_s, pcr);
}

DLL_EXPORT void ESM_SetTransmitterTxAntenna(const char* id,
                                            int pattern, double gain_dBi,
                                            double bw_az_rad, double bw_el_rad,
                                            int scan_mode,
                                            double scan_min_az, double scan_max_az,
                                            double scan_min_el, double scan_max_el)
{
    g_plugin.SetTransmitterTxAntenna(id, pattern, gain_dBi,
                                     bw_az_rad, bw_el_rad,
                                     scan_mode,
                                     scan_min_az, scan_max_az,
                                     scan_min_el, scan_max_el);
}

DLL_EXPORT void ESM_SetTransmitterActive(const char* id, int active)
{
    g_plugin.SetTransmitterActive(id, active != 0);
}

DLL_EXPORT void ESM_SetTransmitterUsePeakPower(const char* id, int use_peak)
{
    g_plugin.SetTransmitterUsePeakPower(id, use_peak != 0);
}

DLL_EXPORT void ESM_RemoveTransmitter(const char* id)
{
    g_plugin.RemoveTransmitter(id);
}

DLL_EXPORT void ESM_ClearTransmitters()
{
    g_plugin.ClearTransmitters();
}

DLL_EXPORT int ESM_GetTransmitterCount()
{
    return g_plugin.GetTransmitterCount();
}

DLL_EXPORT const char* ESM_GetTransmitterId(int index)
{
    return g_plugin.GetTransmitterId(index);
}

// ========================================================================
// 仿真更新
// ========================================================================

DLL_EXPORT void ESM_Update(double sim_time)
{
    g_plugin.Update(sim_time);
}

// ========================================================================
// 直接探测查询
// ========================================================================

DLL_EXPORT int ESM_AttemptDetect(int tx_index, double sim_time,
                                 double* out_snr_dB, double* out_range_m,
                                 double* out_az_rad, double* out_el_rad,
                                 double* out_rx_power_dBm, double* out_ant_gain_dBi,
                                 double* out_path_loss_dB)
{
    return g_plugin.AttemptDetect(tx_index, sim_time,
                                  out_snr_dB, out_range_m,
                                  out_az_rad, out_el_rad,
                                  out_rx_power_dBm, out_ant_gain_dBi,
                                  out_path_loss_dB);
}

// ========================================================================
// 航迹查询
// ========================================================================

DLL_EXPORT int ESM_GetTrackCount()
{
    return g_plugin.GetTrackCount();
}

DLL_EXPORT const char* ESM_GetTrackId(int index)
{
    return g_plugin.GetTrackId(index);
}

DLL_EXPORT int ESM_GetTrackData(int index,
                                double* freq_hz, double* az_rad, double* el_rad,
                                double* range_m, double* rx_power_dBm,
                                double* ant_gain_dBi, double* snr_dB,
                                double* first_detect_time, double* last_detect_time,
                                double* last_update_time, int* detect_count,
                                double* psos_cumulative_pd, int* psos_confirmed)
{
    return g_plugin.GetTrackData(index,
                                 freq_hz, az_rad, el_rad,
                                 range_m, rx_power_dBm,
                                 ant_gain_dBi, snr_dB,
                                 first_detect_time, last_detect_time,
                                 last_update_time, detect_count,
                                 psos_cumulative_pd, psos_confirmed);
}

// ========================================================================
// PSOS 配置
// ========================================================================

DLL_EXPORT void ESM_SetPsosEnabled(int enabled)
{
    g_plugin.SetPsosEnabled(enabled != 0);
}

DLL_EXPORT void ESM_SetPsosConfirmThreshold(double threshold)
{
    g_plugin.SetPsosConfirmThreshold(threshold);
}

DLL_EXPORT void ESM_SetPsosFrameTime(double frame_time_s)
{
    g_plugin.SetPsosFrameTime(frame_time_s);
}

DLL_EXPORT void ESM_SetPsosRequiredPd(double required_pd)
{
    g_plugin.SetPsosRequiredPd(required_pd);
}

// ========================================================================
// 多波束管理
// ========================================================================

DLL_EXPORT int ESM_AddBeam()
{
    return g_plugin.AddBeam();
}

DLL_EXPORT void ESM_RemoveBeam(int beam_index)
{
    g_plugin.RemoveBeam(beam_index);
}

DLL_EXPORT int ESM_GetBeamCount()
{
    return g_plugin.GetBeamCount();
}

DLL_EXPORT void ESM_SetBeamAntennaPattern(int beam_index, int type,
                                           double peak_gain_dBi,
                                           double az_bw_rad, double el_bw_rad,
                                           double back_lobe_dB)
{
    g_plugin.SetBeamAntennaPattern(beam_index, type, peak_gain_dBi,
                                   az_bw_rad, el_bw_rad, back_lobe_dB);
}

DLL_EXPORT void ESM_SetBeamAntennaBoresight(int beam_index,
                                             double az_rad, double el_rad)
{
    g_plugin.SetBeamAntennaBoresight(beam_index, az_rad, el_rad);
}

DLL_EXPORT void ESM_SetBeamAntennaFov(int beam_index,
                                       double min_az, double max_az,
                                       double min_el, double max_el,
                                       double min_range, double max_range)
{
    g_plugin.SetBeamAntennaFov(beam_index, min_az, max_az,
                               min_el, max_el, min_range, max_range);
}

DLL_EXPORT void ESM_AddBeamFrequencyBand(int beam_index,
                                          double lower_hz, double upper_hz,
                                          double dwell_s, double revisit_s)
{
    g_plugin.AddBeamFrequencyBand(beam_index, lower_hz, upper_hz,
                                  dwell_s, revisit_s);
}

// ========================================================================
// 航迹管理（Phase 2）
// ========================================================================

DLL_EXPORT void ESM_SetTrackInitMofN(int m, int n)
{
    g_plugin.SetTrackInitMofN(m, n);
}

DLL_EXPORT void ESM_SetTrackMaintMofN(int m, int n, double coast_s)
{
    g_plugin.SetTrackMaintMofN(m, n, coast_s);
}

// ========================================================================
// 情报层（Phase 1）
// ========================================================================

DLL_EXPORT void ESM_AddEmitterReportRule(const char* emitter_type,
                                          double time_to_declare,
                                          double time_to_reevaluate,
                                          int report_truth)
{
    g_plugin.AddEmitterReportRule(emitter_type, time_to_declare,
                                  time_to_reevaluate, report_truth != 0);
}

DLL_EXPORT void ESM_AddEmitterReportEntry(const char* emitter_type,
                                           const char* reported_name,
                                           double confidence)
{
    g_plugin.AddEmitterReportEntry(emitter_type, reported_name, confidence);
}

DLL_EXPORT void ESM_AddTargetReportRule_Emitters(const char* target_type,
                                                  const char** emitter_ids,
                                                  int emitter_count,
                                                  const char* declared_target)
{
    g_plugin.AddTargetReportRule_Emitters(target_type, emitter_ids,
                                          emitter_count, declared_target);
}

DLL_EXPORT void ESM_SetTargetId(const char* target_id)
{
    g_plugin.SetTargetId(target_id);
}

DLL_EXPORT void ESM_SetMeasurementError(double az_sigma_rad,
                                         double el_sigma_rad,
                                         double range_sigma_m)
{
    g_plugin.SetMeasurementError(az_sigma_rad, el_sigma_rad, range_sigma_m);
}

DLL_EXPORT void ESM_SetReportsFrequency(int enabled)
{
    g_plugin.SetReportsFrequency(enabled != 0);
}

DLL_EXPORT void ESM_SetReportsPulsewidth(int enabled)
{
    g_plugin.SetReportsPulsewidth(enabled != 0);
}

DLL_EXPORT void ESM_SetReportsPri(int enabled)
{
    g_plugin.SetReportsPri(enabled != 0);
}

DLL_EXPORT const char* ESM_GetTrackDeclaredType(int index)
{
    return g_plugin.GetTrackDeclaredType(index);
}

DLL_EXPORT double ESM_GetTrackDeclaredConfidence(int index)
{
    return g_plugin.GetTrackDeclaredConfidence(index);
}

DLL_EXPORT const char* ESM_GetTrackDeclaredTargetType(int index)
{
    return g_plugin.GetTrackDeclaredTargetType(index);
}

} // extern "C"
