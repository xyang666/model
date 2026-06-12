#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#ifdef _WIN32
#include <windows.h>
#endif

#include "ESM_Sensor.hpp"

static constexpr double kPi = 3.14159265358979323846;

static void print_tracks(double sim_time, const std::vector<EmitterTrack> &tracks)
{
    std::cout << "\n[t=" << std::fixed << std::setprecision(1) << sim_time << "s]"
              << " 活跃航迹: " << tracks.size() << "\n";

    for (const auto &t : tracks)
    {
        std::cout << "  " << std::left << std::setw(16) << t.emitter_id
                  << "  freq=" << std::right << std::setprecision(2) << t.frequency_hz / 1e9 << " GHz"
                  << "  az=" << std::setprecision(1) << t.azimuth_rad * 180.0 / kPi << " deg"
                  << "  el=" << t.elevation_rad * 180.0 / kPi << " deg"
                  << "  range=" << std::setprecision(0) << t.range_m / 1000.0 << " km"
                  << "  G_r=" << std::setprecision(1) << t.antenna_gain_dBi << " dBi"
                  << "  SNR=" << t.snr_dB << " dB"
                  << "  hits=" << t.detect_count
                  << "\n";
    }
}

int main()
{
    // 确保 Windows 控制台使用 UTF-8 代码页显示中文
#ifdef _WIN32
    SetConsoleOutputCP(CP_UTF8);
#endif
    // -------------------------------------------------------------------------
    // 构建 ESM 传感器（接收机位于原点，高度 10 m）
    // -------------------------------------------------------------------------
    ESM_Sensor esm;
    esm.receiver.position = Eigen::Vector3d(0, 0, 10);
    esm.receiver.noise_figure_dB = 6.0;
    esm.receiver.bandwidth_hz = 20e6;
    esm.receiver.detection_threshold_dB = 8.0;
    esm.coast_time = 4.0;

    // --- 定向接收天线：高斯波束，60° 方位 × 40° 仰角 ---
    // 波束指向东北（az=45°），仰角 0°
    esm.receiver.antenna.pattern.type = AntennaPattern::Type::GAUSSIAN;
    esm.receiver.antenna.pattern.peak_gain_dBi = 8.0;
    esm.receiver.antenna.pattern.az_beamwidth_rad = 60.0 * kPi / 180.0;
    esm.receiver.antenna.pattern.el_beamwidth_rad = 40.0 * kPi / 180.0;
    esm.receiver.antenna.pattern.back_lobe_floor_dB = -20.0;
    esm.receiver.antenna.boresight_az_rad = 45.0 * kPi / 180.0; // 东北
    esm.receiver.antenna.boresight_el_rad = 0.0;

    // 监听两个频段
    esm.receiver.frequency_bands.push_back({2e9, 4e9});  // S 波段
    esm.receiver.frequency_bands.push_back({8e9, 12e9}); // X 波段

    // -------------------------------------------------------------------------
    // 发射源
    // -------------------------------------------------------------------------
    std::vector<ESM_Transmitter> transmitters;

    // S 波段雷达，50 km 东北（az≈45°）— 靠近天线波束轴，高增益
    ESM_Transmitter sband_radar;
    sband_radar.id = "S-band-radar";
    sband_radar.position = Eigen::Vector3d(35355, 35355, 0); // ≈50 km 东北
    sband_radar.frequency_hz = 3e9;
    sband_radar.power_dBm = 80.0; // 100 kW
    sband_radar.antenna_gain_dBi = 30.0;
    transmitters.push_back(sband_radar);

    // X 波段干扰机，20 km 正北（az=0°）— 偏离波束轴 45°
    ESM_Transmitter xband_jammer;
    xband_jammer.id = "X-band-jammer";
    xband_jammer.position = Eigen::Vector3d(0, 20000, 500);
    xband_jammer.frequency_hz = 9.5e9;
    xband_jammer.power_dBm = 50.0; // 100 W
    xband_jammer.antenna_gain_dBi = 5.0;
    transmitters.push_back(xband_jammer);

    // S 波段发射源，50 km 正南（az=180°）— 天线背面，仅后瓣
    ESM_Transmitter south_radar;
    south_radar.id = "S-band-south";
    south_radar.position = Eigen::Vector3d(0, -50000, 0);
    south_radar.frequency_hz = 3.5e9;
    south_radar.power_dBm = 80.0;
    south_radar.antenna_gain_dBi = 30.0;
    transmitters.push_back(south_radar);

    // VHF 电台 — 频段外，永不探测
    ESM_Transmitter vhf_radio;
    vhf_radio.id = "VHF-radio";
    vhf_radio.position = Eigen::Vector3d(5000, 5000, 0);
    vhf_radio.frequency_hz = 150e6;
    vhf_radio.power_dBm = 40.0;
    vhf_radio.antenna_gain_dBi = 0.0;
    transmitters.push_back(vhf_radio);

    // -------------------------------------------------------------------------
    // 在仿真循环前展示每个发射源的链路预算
    // -------------------------------------------------------------------------
    std::cout << "=== ESM 传感器 - 天线模型演示 ===\n";
    std::cout << "天线: 高斯, peak=" << esm.receiver.antenna.pattern.peak_gain_dBi
              << " dBi, BW_az=60 deg, BW_el=40 deg, 波束指向=NE(45 deg)\n";
    std::cout << "检测门限: " << esm.receiver.detection_threshold_dB << " dB SNR\n\n";

    std::cout << std::left << std::setw(16) << "发射源"
              << std::right << std::setw(8) << "Az(deg)"
              << std::setw(10) << "偏置角"
              << std::setw(10) << "G_r(dBi)"
              << std::setw(12) << "Pr(dBm)"
              << std::setw(10) << "SNR(dB)"
              << std::setw(10) << "探测"
              << "\n";
    std::cout << std::string(76, '-') << "\n";

    for (const auto &xmtr : transmitters)
    {
        ESM_Interaction r = esm.attempt_detect(xmtr, 0.0);
        if (!r.transmitter)
        {
            continue;
        } // 未激活或频段外

        double off_bore_deg = esm.receiver.antenna.off_boresight_angle_rad(
                                  r.azimuth_rad, r.elevation_rad) *
                              180.0 / kPi;

        std::cout << std::left << std::setw(16) << xmtr.id
                  << std::right << std::fixed
                  << std::setw(8) << std::setprecision(1) << r.azimuth_rad * 180.0 / kPi
                  << std::setw(10) << std::setprecision(1) << off_bore_deg
                  << std::setw(10) << std::setprecision(1) << r.antenna_gain_dBi
                  << std::setw(12) << std::setprecision(1) << r.received_power_dBm
                  << std::setw(10) << std::setprecision(1) << r.snr_dB
                  << std::setw(10) << (r.detected ? "YES" : "no");
        std::cout << "\n";
    }

    // -------------------------------------------------------------------------
    // 仿真若干帧
    // -------------------------------------------------------------------------
    std::cout << "\n--- 仿真帧 (1 Hz) ---\n";
    for (int frame = 0; frame <= 10; ++frame)
    {
        double sim_time = frame * 1.0;

        if (frame == 6)
        {
            transmitters[0].active = false;
            std::cout << "\n  *** S-band-radar 关闭 ***\n";
        }

        esm.update(sim_time, transmitters);
        print_tracks(sim_time, esm.tracks());
    }

    // -------------------------------------------------------------------------
    // 第二部分：PSOS 扫描重叠模型演示
    // 展示方位扫描发射器和 PSOS 累积探测。
    // -------------------------------------------------------------------------
    std::cout << "\n\n=== PSOS 扫描重叠模型演示 ===\n";

    ESM_Sensor psos_esm;
    psos_esm.receiver.position = Eigen::Vector3d(0, 0, 10);
    psos_esm.receiver.noise_figure_dB = 6.0;
    psos_esm.receiver.bandwidth_hz = 20e6;
    psos_esm.receiver.detection_threshold_dB = 10.0;
    psos_esm.coast_time = 10.0;

    // 接收天线：宽波束全向（简化 PSOS 演示，聚焦发射扫描）
    psos_esm.receiver.antenna.pattern.type = AntennaPattern::Type::ISOTROPIC;
    psos_esm.receiver.antenna.pattern.peak_gain_dBi = 0.0;

    // 监听 S 波段，每 1s 扫 0.1s
    {
        FrequencyBand band(2e9, 4e9);
        band.dwell_time_s = 0.1;
        band.revisit_time_s = 1.0;
        psos_esm.receiver.frequency_bands.push_back(band);
    }

    // 启用 PSOS
    psos_esm.receiver.psos_enabled = true;
    psos_esm.receiver.psos_confirm_threshold = 0.9;
    psos_esm.receiver.psos_frame_time_s = 1.0;

    // 发射源：脉冲扫描雷达，波束窄，方位扫描
    ESM_Transmitter scan_radar;
    scan_radar.id = "Scan-Radar";
    scan_radar.position = Eigen::Vector3d(50000, 0, 1000); // 正东 50 km
    scan_radar.frequency_hz = 3e9;
    scan_radar.power_dBm = 80.0;
    scan_radar.antenna_gain_dBi = 30.0;
    scan_radar.signal_type = SignalType::PULSED;
    scan_radar.duty_cycle = 0.001;
    scan_radar.tx_antenna.pattern = ESM_Transmitter::TxAntennaModel::Pattern::GAUSSIAN;
    scan_radar.tx_antenna.gain_dBi = 30.0;
    scan_radar.tx_antenna.beamwidth_az_rad = 5.0 * kPi / 180.0;  // 5° 窄波束
    scan_radar.scan_mode = ESM_Transmitter::ScanMode::AZ_SCAN;
    scan_radar.scan_min_az_rad = -80.0 * kPi / 180.0;  // 扫描范围 -80° ~ +80°
    scan_radar.scan_max_az_rad =  80.0 * kPi / 180.0;

    std::vector<ESM_Transmitter> psos_tx = {scan_radar};

    // 打印初始链路预算
    std::cout << "\n初始链路预算 (PSOS 场景):\n";
    {
        ESM_Interaction r = psos_esm.attempt_detect(scan_radar, 0.0);
        if (r.transmitter)
        {
            std::cout << "  SNR = " << r.snr_dB << " dB"
                      << "  P_r = " << r.received_power_dBm << " dBm"
                      << "  门限 = " << r.effective_threshold_dB << " dB"
                      << "  直接探测: " << (r.detected ? "YES" : "no") << "\n";
        }
    }

    // 仿真 30 帧，展示 PSOS 累积
    std::cout << "\nPSOS 累积仿真 (1 Hz, 30 帧):\n";
    auto &psos_track_list = const_cast<std::vector<EmitterTrack>&>(
        static_cast<const ESM_Sensor&>(psos_esm).tracks());

    for (int frame = 0; frame < 30; ++frame)
    {
        double t = frame * 1.0;
        psos_esm.update(t, psos_tx);

        if (psos_esm.tracks().size() > 0)
        {
            const auto &trk = psos_esm.tracks()[0];
            std::cout << "  t=" << std::setw(3) << t << "s"
                      << "  Pd_cum=" << std::setw(6) << std::setprecision(4) << trk.psos_cumulative_pd
                      << "  confirmed=" << (trk.psos_confirmed ? "YES" : "no ")
                      << "  detect_count=" << trk.detect_count;
            if (trk.psos_confirmed)
                std::cout << "  SNR=" << trk.snr_dB << " dB";
            std::cout << "\n";
        }
    }

    return 0;
}
