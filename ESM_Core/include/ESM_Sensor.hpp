#ifndef ESM_SENSOR_HPP
#define ESM_SENSOR_HPP

#include <cmath>
#include <string>
#include <vector>

#include "ESM_Interaction.hpp"
#include "ESM_Receiver.hpp"
#include "ESM_Transmitter.hpp"
#include "EmitterTrack.hpp"
#include "ESM_ErrorModel.hpp"
#include "ESM_Reporting.hpp"
#include "ESM_TrackManager.hpp"

// ESM（电子支援措施）传感器。
//
// 每次调用 update() 扫描所有发射源，执行单向链路预算，并维护辐射源航迹列表。
//
// 支持两种检测路径：
//
//   直接模式（psos_enabled == false，默认）：
//     当 SNR >= 门限且接收机正在扫描对应频段时，立即判定探测成功。
//
//   PSOS 模式（psos_enabled == true）：
//     使用扫描重叠概率模型（PA × PF）跨帧累积。
//     PA（方位重叠概率）基于发射天线方向图及扫描参数计算；
//     PF（频率重叠概率）基于接收机频段扫描调度计算。
//     仅当累积概率达到 psos_confirm_threshold 时才确认航迹。
//     适用于扫描雷达波束周期性扫过 ESM 的场景。
//
// 情报层（Phase 1）：
//   EmitterTypeReporting — 基于置信度的发射器类型识别
//   TargetTypeReporting  — 基于发射器组合的目标平台分类
//   MeasurementErrorModel — DOA/距离测量误差
class ESM_Sensor
{
public:
    ESM_Receiver receiver;

    // 航迹保持时间：自上次带内观测起超过此时长则删除航迹 (s)。
    double coast_time{5.0};

    // 在 sim_time 时刻扫描所有发射源，更新航迹列表。
    void update(double sim_time, const std::vector<ESM_Transmitter>& transmitters);

    // 对单个发射源在 sim_time 执行链路预算。
    // 返回完整填充的 ESM_Interaction（通过检查 .transmitter != nullptr 判断是否有效）。
    ESM_Interaction attempt_detect(const ESM_Transmitter& xmtr, double sim_time) const;

    const std::vector<EmitterTrack>& tracks() const { return tracks_; }

    // --- 情报层访问（Phase 1） ---
    EmitterTypeReporting& emitter_reporting() { return emitter_reporting_; }
    TargetTypeReporting&  target_reporting()  { return target_reporting_; }
    MeasurementErrorModel& error_model()      { return error_model_; }

    // --- 航迹管理层（Phase 2） ---
    TrackManager& track_manager() { return track_manager_; }

    // 配置 M/N 航迹建立参数
    void SetTrackInitMofN(int m, int n);
    // 配置 M/N 航迹维持参数
    void SetTrackMaintMofN(int m, int n, double coast_s);

    // 获取航迹状态
    TrackManager::TrackState GetEmitterTrackState(const std::string& emitter_id) const;
    // 获取本帧刚刚丢失的航迹 ID
    std::vector<std::string> GetJustDroppedIds() const
        { return track_manager_.GetJustDroppedIds(); }

    // 信号数据捕获开关
    bool reports_frequency  = false;
    bool reports_pulsewidth = false;
    bool reports_pri        = false;

    // 用于平台分类的目标 ID（由外部设置，如平台名称）
    std::string target_id;

private:
    std::vector<EmitterTrack> tracks_;

    // --- 情报层成员 ---
    EmitterTypeReporting emitter_reporting_;
    TargetTypeReporting  target_reporting_;
    MeasurementErrorModel error_model_;
    TrackManager        track_manager_;

    // --- 直接检测路径 ---
    void update_track(double sim_time, const ESM_Interaction& interaction);

    // --- PSOS 路径 ---
    struct PSOS_Result
    {
        double pa{0.0};
        double pf{0.0};
        double pss{0.0};
        double pss_frame{0.0};
        double pd_cum{0.0};
    };

    PSOS_Result compute_psos(const ESM_Interaction& r,
                             const EmitterTrack& track) const;

    EmitterTrack& get_or_create_track(const std::string& id, double sim_time);

    void fill_track_data(double sim_time, EmitterTrack& track,
                         const ESM_Interaction& interaction);

    void capture_signal_data(EmitterTrack& track,
                             const ESM_Interaction& r);

    void apply_reporting(double sim_time, EmitterTrack& track,
                         const ESM_Transmitter& xmtr);

    void remove_dropped_tracks();

    EmitterTrack* find_track(const std::string& id);
};

#endif // ESM_SENSOR_HPP
