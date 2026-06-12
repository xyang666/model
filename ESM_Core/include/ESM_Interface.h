// ****************************************************************************
// ESM_Interface.h - 电子侦察模型对外 C-API 接口
//
// 外部调用者只需要此头文件 + .lib + .dll，完全看不到内部实现。
//
// 使用方式：
//   1. 调用 Initialize() 初始化模型
//   2. 通过 ESM_Set* 系列函数配置接收机、天线、频段
//   3. 通过 ESM_AddTransmitter / ESM_SetTransmitter* 添加和配置辐射源
//   4. 每帧调用 ESM_Update(sim_time) 执行仿真
//   5. 通过 ESM_GetTrackCount / ESM_GetTrackData 查询航迹
//   6. 结束时调用 Finalize()
// ****************************************************************************
#pragma once

#include "stdafx.h"

// ============================================================================
// 枚举常量定义（与 C++ enum class 默认值一一对应）
// ============================================================================

// AntennaPattern::Type
#define ESM_ANTENNA_ISOTROPIC  0
#define ESM_ANTENNA_GAUSSIAN   1
#define ESM_ANTENNA_SINC2      2

// SignalType
#define ESM_SIGNAL_CW          0
#define ESM_SIGNAL_PULSED      1

// Polarization
#define ESM_POLARIZATION_NONE        0
#define ESM_POLARIZATION_HORIZONTAL  1
#define ESM_POLARIZATION_VERTICAL    2
#define ESM_POLARIZATION_LHCP        3
#define ESM_POLARIZATION_RHCP        4

// ESM_Transmitter::TxAntennaModel::Pattern
#define ESM_TX_ANTENNA_ISOTROPIC  0
#define ESM_TX_ANTENNA_GAUSSIAN   1
#define ESM_TX_ANTENNA_SINC2      2

// ESM_Transmitter::ScanMode
#define ESM_SCAN_FIXED      0
#define ESM_SCAN_AZ         1
#define ESM_SCAN_EL         2
#define ESM_SCAN_AZ_EL      3

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 模型生命周期
// ============================================================================

DLL_EXPORT bool Initialize();
DLL_EXPORT bool Finalize();

// ============================================================================
// 接收机配置
// ============================================================================

DLL_EXPORT void ESM_SetReceiverPosition(double x, double y, double z);
DLL_EXPORT void ESM_SetReceiverNoiseFigure(double nf_dB);
DLL_EXPORT void ESM_SetReceiverBandwidth(double bw_hz);
DLL_EXPORT void ESM_SetReceiverDetectionThreshold(double thr_dB);
DLL_EXPORT void ESM_SetCoastTime(double coast_s);

// ============================================================================
// 天线配置
// ============================================================================

DLL_EXPORT void ESM_SetAntennaPattern(int type, double peak_gain_dBi,
                                      double az_bw_rad, double el_bw_rad,
                                      double back_lobe_dB);
DLL_EXPORT void ESM_SetAntennaBoresight(double az_rad, double el_rad);
DLL_EXPORT void ESM_SetAntennaFov(double min_az_rad, double max_az_rad,
                                  double min_el_rad, double max_el_rad,
                                  double min_range_m, double max_range_m);

// ============================================================================
// 频段管理
// ============================================================================

DLL_EXPORT void ESM_AddFrequencyBand(double lower_hz, double upper_hz,
                                     double dwell_s, double revisit_s);
DLL_EXPORT void ESM_ClearFrequencyBands();

// ============================================================================
// 辐射源管理
// ============================================================================

DLL_EXPORT int  ESM_AddTransmitter(const char* id,
                                   double pos_x, double pos_y, double pos_z,
                                   double freq_hz, double power_dBm, double gain_dBi);
DLL_EXPORT void ESM_SetTransmitterSignal(const char* id, int signal_type,
                                         double duty_cycle, double signal_bw_hz);
DLL_EXPORT void ESM_SetTransmitterPolarization(const char* id, int polarization);
DLL_EXPORT void ESM_SetTransmitterPulse(const char* id,
                                        double pulse_width_s, double prf_hz,
                                        double pri_s, double pcr);
DLL_EXPORT void ESM_SetTransmitterTxAntenna(const char* id,
                                            int pattern, double gain_dBi,
                                            double bw_az_rad, double bw_el_rad,
                                            int scan_mode,
                                            double scan_min_az, double scan_max_az,
                                            double scan_min_el, double scan_max_el);
DLL_EXPORT void ESM_SetTransmitterActive(const char* id, int active);
DLL_EXPORT void ESM_SetTransmitterUsePeakPower(const char* id, int use_peak);
DLL_EXPORT void ESM_RemoveTransmitter(const char* id);
DLL_EXPORT void ESM_ClearTransmitters();
DLL_EXPORT int         ESM_GetTransmitterCount();
DLL_EXPORT const char* ESM_GetTransmitterId(int index);

// ============================================================================
// 仿真更新
// ============================================================================

DLL_EXPORT void ESM_Update(double sim_time);

// ============================================================================
// 直接探测查询（单次链路预算，不更新航迹）
// 返回值：1=探测成功，0=未探测到，-1=index越界或无有效交互
// ============================================================================

DLL_EXPORT int ESM_AttemptDetect(int tx_index, double sim_time,
                                 double* out_snr_dB, double* out_range_m,
                                 double* out_az_rad, double* out_el_rad,
                                 double* out_rx_power_dBm, double* out_ant_gain_dBi,
                                 double* out_path_loss_dB);

// ============================================================================
// 航迹查询
// ============================================================================

DLL_EXPORT int         ESM_GetTrackCount();
DLL_EXPORT const char* ESM_GetTrackId(int index);
DLL_EXPORT int         ESM_GetTrackData(int index,
                                        double* freq_hz, double* az_rad, double* el_rad,
                                        double* range_m, double* rx_power_dBm,
                                        double* ant_gain_dBi, double* snr_dB,
                                        double* first_detect_time, double* last_detect_time,
                                        double* last_update_time, int* detect_count,
                                        double* psos_cumulative_pd, int* psos_confirmed);

// ============================================================================
// PSOS 配置
// ============================================================================

DLL_EXPORT void ESM_SetPsosEnabled(int enabled);
DLL_EXPORT void ESM_SetPsosConfirmThreshold(double threshold);
DLL_EXPORT void ESM_SetPsosFrameTime(double frame_time_s);
DLL_EXPORT void ESM_SetPsosRequiredPd(double required_pd);

// ============================================================================
// 发射器报告规则（Phase 1）
// ============================================================================

// 添加发射器报告规则：emitter_type 匹配真实类型，report_truth=1 始终报告真实类型
DLL_EXPORT void ESM_AddEmitterReportRule(const char* emitter_type,
                                          double time_to_declare,
                                          double time_to_reevaluate,
                                          int report_truth);

// 为指定发射器类型添加概率报告条目（confidence=0 表示 remainder）
DLL_EXPORT void ESM_AddEmitterReportEntry(const char* emitter_type,
                                           const char* reported_name,
                                           double confidence);

// 添加 cEMITTERS 目标平台报告规则
DLL_EXPORT void ESM_AddTargetReportRule_Emitters(
    const char* target_type,
    const char** emitter_ids, int emitter_count,
    const char* declared_target);

// 设置传感器所属的目标平台 ID（用于平台分类）
DLL_EXPORT void ESM_SetTargetId(const char* target_id);

// ============================================================================
// 多波束管理
// ============================================================================

DLL_EXPORT int  ESM_AddBeam();
DLL_EXPORT void ESM_RemoveBeam(int beam_index);
DLL_EXPORT int  ESM_GetBeamCount();
DLL_EXPORT void ESM_SetBeamAntennaPattern(int beam_index, int type,
                                           double peak_gain_dBi,
                                           double az_bw_rad, double el_bw_rad,
                                           double back_lobe_dB);
DLL_EXPORT void ESM_SetBeamAntennaBoresight(int beam_index,
                                             double az_rad, double el_rad);
DLL_EXPORT void ESM_SetBeamAntennaFov(int beam_index,
                                       double min_az, double max_az,
                                       double min_el, double max_el,
                                       double min_range, double max_range);
DLL_EXPORT void ESM_AddBeamFrequencyBand(int beam_index,
                                          double lower_hz, double upper_hz,
                                          double dwell_s, double revisit_s);

// ============================================================================
// M/N 航迹管理配置（Phase 2）
// ============================================================================

DLL_EXPORT void ESM_SetTrackInitMofN(int m, int n);
DLL_EXPORT void ESM_SetTrackMaintMofN(int m, int n, double coast_s);

// ============================================================================
// DOA 测量误差配置（Phase 1）
// ============================================================================

DLL_EXPORT void ESM_SetMeasurementError(double az_sigma_rad,
                                         double el_sigma_rad,
                                         double range_sigma_m);

// ============================================================================
// 信号数据捕获开关（Phase 1）
// ============================================================================

DLL_EXPORT void ESM_SetReportsFrequency(int enabled);
DLL_EXPORT void ESM_SetReportsPulsewidth(int enabled);
DLL_EXPORT void ESM_SetReportsPri(int enabled);

// ============================================================================
// 航迹声明类型查询（Phase 1）
// ============================================================================

DLL_EXPORT const char* ESM_GetTrackDeclaredType(int index);
DLL_EXPORT double      ESM_GetTrackDeclaredConfidence(int index);
DLL_EXPORT const char* ESM_GetTrackDeclaredTargetType(int index);

#ifdef __cplusplus
}
#endif
