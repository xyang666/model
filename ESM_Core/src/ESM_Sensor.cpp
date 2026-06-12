#include "ESM_Sensor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

// ---------------------------------------------------------------------------
void ESM_Sensor::update(double sim_time, const std::vector<ESM_Transmitter> &transmitters)
{
    for (const auto &xmtr : transmitters)
    {
        ESM_Interaction r = attempt_detect(xmtr, sim_time);

        // compute() 返回 false：发射源未激活、频率在频段外或超出 FOV。
        if (!r.transmitter)
            continue;

        // 对每个带内发射源始终更新 last_update_time，
        // 确保 PSOS 状态和 coast-time 老化由观测而非探测驱动。
        EmitterTrack &track = get_or_create_track(xmtr.id, sim_time);
        track.last_update_time = sim_time;

        bool detected_this_frame = false;

        if (!receiver.psos_enabled)
        {
            // --- 直接检测路径 ---
            if (r.detected)
            {
                fill_track_data(sim_time, track, r);
                detected_this_frame = true;
            }
        }
        else
        {
            // --- PSOS 累积路径（扫描重叠模型） ---
            PSOS_Result psos = compute_psos(r, track);
            track.psos_cumulative_pd = psos.pd_cum;

            if (psos.pss > 0.0)
            {
                double required = (receiver.required_pd >= 0.0)
                                      ? receiver.required_pd
                                      : receiver.psos_confirm_threshold;
                if (psos.pd_cum >= required)
                {
                    track.psos_confirmed = true;
                    fill_track_data(sim_time, track, r);
                    detected_this_frame = true;
                }
            }
            else
            {
                track.psos_cumulative_pd *= 0.8;
                if (track.psos_cumulative_pd < 0.5)
                    track.psos_cumulative_pd = 0.0;
            }
        }

        // 情报层：发射器识别 + 目标平台分类
        apply_reporting(sim_time, track, xmtr);

        // M/N 航迹管理：记录每次检测机会
        track_manager_.OnDetectionOpportunity(xmtr.id, detected_this_frame, sim_time);

        // 若 TrackManager 判定航迹已删除，从 tracks_ 中移除
        if (track_manager_.GetTrackState(xmtr.id) == TrackManager::DROPPED)
        {
            auto it = std::find_if(tracks_.begin(), tracks_.end(),
                                   [&](const EmitterTrack& t) { return t.emitter_id == xmtr.id; });
            if (it != tracks_.end())
                tracks_.erase(it);
        }
    }

    // 每帧评估报告引擎
    emitter_reporting_.Evaluate(sim_time);
    target_reporting_.Evaluate(sim_time);

    // 将声明结果写回航迹
    for (auto& track : tracks_)
    {
        std::string declared_type;
        double confidence = 0.0;
        if (emitter_reporting_.GetDeclaration(track.emitter_id, declared_type, confidence))
        {
            track.declared_emitter_type = declared_type;
            track.declared_confidence   = confidence;
            track.type_declared         = true;
        }
        if (!target_id.empty())
        {
            std::string target_type;
            double target_conf = 0.0;
            if (target_reporting_.GetDeclaration(target_id, target_type, target_conf))
                track.declared_target_type = target_type;
        }
    }

    remove_dropped_tracks();
}

// ---------------------------------------------------------------------------
ESM_Interaction ESM_Sensor::attempt_detect(const ESM_Transmitter &xmtr,
                                          double sim_time) const
{
    // 同步 beams[0] 与 receiver.antenna + frequency_bands
    const_cast<ESM_Receiver&>(receiver).beams.resize(1);
    const_cast<ESM_Receiver&>(receiver).beams[0].antenna = receiver.antenna;
    const_cast<ESM_Receiver&>(receiver).beams[0].frequency_bands = receiver.frequency_bands;
    const_cast<ESM_Receiver&>(receiver).beams[0].beam_index = 1;

    ESM_Interaction best;
    best.transmitter = &xmtr;
    best.receiver = &receiver;
    best.sim_time = sim_time;

    // 遍历所有波束，取 SNR 最优
    for (auto& beam : receiver.beams)
    {
        ESM_Interaction r;
        r.transmitter = &xmtr;
        r.receiver = &receiver;
        r.sim_time = sim_time;

        if (!r.compute(beam))
            continue;

        if (!best.transmitter || r.snr_dB > best.snr_dB)
            best = r;
    }

    if (!best.transmitter)
        best.transmitter = nullptr;

    return best;
}

// ---------------------------------------------------------------------------
// PSOS 计算：PA（方位重叠概率）× PF（频率重叠概率）
//
// 步骤：
//   1. PA = 1.0；发射天线扫描时基于方向图计算方位重叠比例。
//   2. PF = dwell_time / revisit_time（由频率重叠概率决定）。
//   3. PSS = PA × PF（单次扫描检测概率）。
//   4. dwell_count = frame_time / revisit_time（帧内驻留次数）。
//   5. PSS_frame = 1 - (1 - PSS)^dwell_count（帧级检测概率）。
//   6. pd_cum = 1 - (1 - prev_pd_cum) × (1 - PSS_frame)（更新累积 Pd）。
ESM_Sensor::PSOS_Result ESM_Sensor::compute_psos(const ESM_Interaction &r,
                                                   const EmitterTrack &track) const
{
    PSOS_Result result;

    const auto &xmtr = *r.transmitter;

    // ==================================================================
    // 1. PA — 方位重叠概率
    // ==================================================================
    double pa = 1.0;

    // 发射天线是否在方位方向扫描？
    bool tx_scans_az = (xmtr.scan_mode == ESM_Transmitter::ScanMode::AZ_SCAN ||
                        xmtr.scan_mode == ESM_Transmitter::ScanMode::AZ_EL_SCAN);

    if (tx_scans_az && xmtr.tx_antenna.gain_dBi > 0.0)
    {
        // S_req：达到检测门限所需的最小接收功率（线性 mW）
        double threshold_linear = std::pow(10.0, r.effective_threshold_dB / 10.0);
        double noise_linear     = std::pow(10.0, r.noise_floor_dBm / 10.0);
        double s_req_linear     = threshold_linear * noise_linear;

        // S_iso：若发射天线增益为 1（0 dBi）时的接收功率（线性 mW）
        double g_t_linear = std::pow(10.0, r.transmitter->antenna_gain_dBi / 10.0);
        double p_r_linear = std::pow(10.0, r.received_power_dBm / 10.0);
        double s_iso_linear = p_r_linear / g_t_linear;

        // G_req_norm：所需发射增益相对峰值的归一化比值 [0, 1]
        double g_req_norm = (s_iso_linear > 0.0)
                                ? (s_req_linear / s_iso_linear) / g_t_linear
                                : 0.0;

        // 通过发射天线方向图计算方位重叠比例
        if (g_req_norm > 0.0 && g_req_norm <= 1.0)
        {
            // 使用 AntennaPattern 的 GetGainThresholdFraction
            // 但 ESM_Transmitter 目前没有直接使用 AntennaPattern 类，
            // 而是用 TxAntennaModel 的轻量字段。这里直接计算。
            double az_span = xmtr.scan_max_az_rad - xmtr.scan_min_az_rad;
            if (az_span > 0.0)
            {
                if (xmtr.tx_antenna.pattern == ESM_Transmitter::TxAntennaModel::Pattern::ISOTROPIC)
                {
                    pa = 1.0;
                }
                else if (xmtr.tx_antenna.pattern == ESM_Transmitter::TxAntennaModel::Pattern::GAUSSIAN)
                {
                    // 高斯功率模式：G_norm(az) = exp(-a * az²)
                    // a = 4*ln(2) / beamwidth²
                    double bw = xmtr.tx_antenna.beamwidth_az_rad;
                    if (bw > 0.0 && g_req_norm < 1.0)
                    {
                        double a = 2.772588722239781; // 4*ln(2)
                        double az_half = bw * std::sqrt(-std::log(g_req_norm) / a);
                        pa = std::min(1.0, (2.0 * az_half) / az_span);
                    }
                }
                else // SINC2 — 数值采样
                {
                    constexpr int kSamples = 1000;
                    int count_above = 0;
                    for (int i = 0; i < kSamples; ++i)
                    {
                        double t = static_cast<double>(i) / static_cast<double>(kSamples - 1);
                        double az = xmtr.scan_min_az_rad + t * az_span;
                        double abs_az = std::abs(az);
                        // SINC2 功率模式
                        double bw_avg = 0.5 * (xmtr.tx_antenna.beamwidth_az_rad +
                                                xmtr.tx_antenna.beamwidth_el_rad);
                        double x = 3.14159265358979323846 * abs_az / bw_avg;
                        double g_norm = (x < 1e-9) ? 1.0
                                      : std::sin(x) * std::sin(x) / (x * x);
                        if (g_norm >= g_req_norm)
                            ++count_above;
                    }
                    pa = static_cast<double>(count_above) / static_cast<double>(kSamples);
                }
            }
        }
    }

    result.pa = pa;

    // ==================================================================
    // 2. PF — 频率重叠概率
    // ==================================================================
    double pf = 1.0;
    double revisit_time = 1.0;

    // 寻找发射频率所在的频段
    for (const auto &band : receiver.frequency_bands)
    {
        if (xmtr.frequency_hz >= band.lower_hz && xmtr.frequency_hz <= band.upper_hz)
        {
            if (band.revisit_time_s > 0.0 && band.dwell_time_s > 0.0)
            {
                pf = std::min(band.dwell_time_s / band.revisit_time_s, 1.0);
                revisit_time = band.revisit_time_s;
            }
            break;
        }
    }

    result.pf = pf;

    // ==================================================================
    // 3. PSS — 单次扫描检测概率
    // ==================================================================
    double pss = pa * pf;
    result.pss = pss;

    // ==================================================================
    // 4. 帧内多次驻留
    // ==================================================================
    double dwell_count = (revisit_time > 0.0)
                             ? (receiver.psos_frame_time_s / revisit_time)
                             : 1.0;
    if (dwell_count < 1.0) dwell_count = 1.0;

    double pss_frame = 1.0 - std::pow(1.0 - pss, dwell_count);
    result.pss_frame = pss_frame;

    // ==================================================================
    // 5. 更新累积检测概率
    //    pd_cum_new = 1 - (1 - pd_cum_old) * (1 - pss_frame)
    // ==================================================================
    double prev_pd = track.psos_cumulative_pd;
    result.pd_cum = 1.0 - (1.0 - prev_pd) * (1.0 - pss_frame);

    return result;
}

// ---------------------------------------------------------------------------
EmitterTrack &ESM_Sensor::get_or_create_track(const std::string &id, double sim_time)
{
    EmitterTrack *t = find_track(id);
    if (!t)
    {
        tracks_.push_back(EmitterTrack{});
        t = &tracks_.back();
        t->emitter_id = id;
        t->first_detect_time = sim_time;
        t->last_update_time = sim_time;
    }
    return *t;
}

// ---------------------------------------------------------------------------
void ESM_Sensor::fill_track_data(double sim_time, EmitterTrack &track,
                                 const ESM_Interaction &r)
{
    // 应用 DOA 测量误差
    double noisy_az, noisy_el;
    error_model_.ApplyAzElError(r.azimuth_rad, r.elevation_rad,
                                r.range_m, r.transmitter->frequency_hz,
                                noisy_az, noisy_el);

    track.frequency_hz = r.transmitter->frequency_hz;
    track.azimuth_rad   = noisy_az;
    track.elevation_rad = noisy_el;

    // 应用距离误差
    double track_age = sim_time - track.first_detect_time;
    double noisy_range;
    bool range_valid = false;
    error_model_.ApplyRangeError(r.range_m, track_age, noisy_range, range_valid);
    track.range_m = range_valid ? noisy_range : 0.0;

    track.received_power_dBm = r.received_power_dBm;
    track.antenna_gain_dBi   = r.antenna_gain_dBi;
    track.snr_dB             = r.snr_dB;
    track.last_detect_time   = sim_time;
    track.detect_count      += 1;

    // 捕获信号数据
    capture_signal_data(track, r);
}

// ---------------------------------------------------------------------------
void ESM_Sensor::capture_signal_data(EmitterTrack &track,
                                     const ESM_Interaction &r)
{
    SignalData sig;
    const auto& xmtr = *r.transmitter;

    if (reports_frequency)
    {
        double bandwidth = (xmtr.signal_bandwidth_hz > 0.0)
                               ? xmtr.signal_bandwidth_hz
                               : receiver.bandwidth_hz;
        sig.frequency_lower_hz = xmtr.frequency_hz - bandwidth * 0.5;
        sig.frequency_upper_hz = xmtr.frequency_hz + bandwidth * 0.5;
    }

    if (reports_pulsewidth)
        sig.pulse_width_s = xmtr.pulse_width_s;

    if (reports_pri)
        sig.pulse_repetition_interval_s = xmtr.pulse_repetition_interval_s;

    sig.emitter_truth_type = xmtr.id; // truth type = ID for now
    track.signal_history.push_back(sig);
}

// ---------------------------------------------------------------------------
void ESM_Sensor::apply_reporting(double sim_time, EmitterTrack &track,
                                  const ESM_Transmitter &xmtr)
{
    // 发射器识别：检测到则通知报告引擎
    const std::string& truth_type = xmtr.id;
    emitter_reporting_.OnDetection(track.emitter_id, truth_type, sim_time);

    // 目标平台分类
    if (!target_id.empty())
        target_reporting_.OnDetection(target_id, track.emitter_id, sim_time);
}

// ---------------------------------------------------------------------------
void ESM_Sensor::remove_dropped_tracks()
{
    tracks_.erase(
        std::remove_if(tracks_.begin(), tracks_.end(),
                       [&](const EmitterTrack& t)
                       {
                           return track_manager_.GetTrackState(t.emitter_id)
                                  == TrackManager::DROPPED;
                       }),
        tracks_.end());
}

// ---------------------------------------------------------------------------
void ESM_Sensor::SetTrackInitMofN(int m, int n)
{
    TrackInitConfig init;
    init.m_required = m;
    init.n_opportunities = n;
    TrackMaintConfig maint;
    maint.coast_time_s = coast_time;
    track_manager_.Configure(init, maint);
}

void ESM_Sensor::SetTrackMaintMofN(int m, int n, double coast_s)
{
    TrackInitConfig init;
    TrackMaintConfig maint;
    maint.m_required = m;
    maint.n_opportunities = n;
    maint.coast_time_s = coast_s;
    coast_time = coast_s;
    track_manager_.Configure(init, maint);
}

TrackManager::TrackState ESM_Sensor::GetEmitterTrackState(const std::string& emitter_id) const
{
    return track_manager_.GetTrackState(emitter_id);
}

// ---------------------------------------------------------------------------
EmitterTrack *ESM_Sensor::find_track(const std::string &id)
{
    for (auto &t : tracks_)
        if (t.emitter_id == id)
            return &t;
    return nullptr;
}
