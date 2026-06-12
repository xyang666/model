#include "tracker/Tracker.h"
#include "filter/KalmanFilterND.h"
#include "filter/ExtendedKalmanFilter.h"
#include "filter/UnscentedKalmanFilter.h"
#include "filter/FilterTypes.h"

namespace TargetMeasurement
{

    static std::unique_ptr<Filter::FilterBase> createFilter(int type)
    {
        switch (type)
        {
        case TM_FILTER_EKF:
            return std::unique_ptr<Filter::ExtendedKalmanFilter>(new Filter::ExtendedKalmanFilter(6, 3));
        case TM_FILTER_UKF:
            return std::unique_ptr<Filter::UnscentedKalmanFilter>(new Filter::UnscentedKalmanFilter(6, 3));
        case TM_FILTER_KALMAN:
        default:
            return std::unique_ptr<Filter::KalmanFilterND>(new Filter::KalmanFilterND(6, 3));
        }
    }

    // 默认匀速运动模型：x' = [I, dt*I; 0, I] * x
    // 状态向量 x = [px, py, pz, vx, vy, vz]ᵀ，前一半位置，后一半速度
    static void setupFilter(Filter::FilterBase* f, double qVal)
    {
        f->setStateFunc(
            [](const Eigen::VectorXd& x, double dt) {
                int h = static_cast<int>(x.size()) / 2;
                Eigen::VectorXd nx(x);
                nx.head(h) += nx.tail(h) * dt;
                return nx;
            });
        // f 对 x 的 Jacobian：A = ∂f/∂x = [I, dt*I; 0, I]
        f->setStateJacobian(
            [](const Eigen::VectorXd&, double dt) {
                Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
                F.topRightCorner(3, 3) *= dt;
                return F;
            });
        f->setMeasureFunc([](const Eigen::VectorXd& x) { return x.head(3); });
        f->setMeasureJacobian([](const Eigen::VectorXd&) {
            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, 6);
            H.leftCols(3).setIdentity();
            return H;
        });
        f->setQ(Eigen::MatrixXd::Identity(6, 6) * qVal);
    }

    int Tracker::ProcessMeasurement(const void* measurement,
                                    TM_LocationOutput* output)
    {
        if (!measurement || !m_initialized_)
            return -1;

        const auto* input = static_cast<const TM_MeasurementInput*>(measurement);
        int tid = input->targetId;

        // 获取或创建航迹
        TrackData* track = getTrack(tid);
        if (!track && tid >= 0)
        {
            TrackData td;
            td.targetId = tid;
            td.detectCount = 0;
            td.totalCount = 0;
            td.lastUpdateTime = input->simTime;
            td.lastMeasTime = input->simTime;
            td.filter = createFilter(m_config_.filterType);
            setupFilter(td.filter.get(), this->defaultProcessNoise());
            auto result = m_tracks_.insert({tid, std::move(td)});
            track = &result.first->second;
        }
        if (!track)
            return -1;

        // 预测
        double dt = input->simTime - track->lastUpdateTime;
        if (dt > 0.0 && track->filter->isInitialized())
            track->filter->predict(dt);
        track->lastUpdateTime = input->simTime;

        // 观测
        int sidx = (input->sensorType >= 0 && input->sensorType <= 2) ? input->sensorType : 0;
        auto obs = obsFuncs_[sidx]
            ? obsFuncs_[sidx](measurement, track->filter->state(), track->filter->covariance())
            : ObservationData{Eigen::VectorXd::Zero(0), Eigen::MatrixXd::Zero(0, 0)};
        if (obs.measurement.size() > 0)
        {
            track->filter->setR(obs.R);
            track->filter->update(obs.measurement);
            track->quality.recordUpdate(track->filter->innovation(), track->filter->innovationCovariance());

            track->detectCount++;
            track->lastMeasTime = input->simTime;
        }
        track->totalCount++;

        // M/N 窗口
        {
            int n = std::max(m_config_.track_maint_n, m_config_.track_init_n);
            int window = track->totalCount;
            if (window > n)
            {
                track->detectCount = std::max(0, track->detectCount - 1);
                track->totalCount = n;
            }
        }

        // 输出
        fill_output(input->simTime, *track, &track->lastOutput);
        track->lastOutput.valid = 1;

        if (output)
            *output = track->lastOutput;

        return 0;
    }

    int Tracker::NoDetectUpdate(double simTime, TM_LocationOutput* output)
    {
        if (!m_initialized_)
            return -1;

        for (auto& pair : m_tracks_)
        {
            auto& t = pair.second;
            double dt = simTime - t.lastUpdateTime;
            if (dt > 0.0 && t.filter && t.filter->isInitialized())
                t.filter->predict(dt);
            t.quality.recordPredict(dt);
            t.lastUpdateTime = simTime;

            // M/N miss
            t.totalCount++;
            if (t.totalCount > std::max(m_config_.track_maint_n, m_config_.track_init_n))
            {
                t.totalCount--;
            }

            fill_output(simTime, t, &t.lastOutput);
            t.lastOutput.valid = (t.quality.getFilterState() != TM_STATE_UNINITIALIZED);
        }

        deleteStaleTracks(simTime);
        m_lastSimTime_ = simTime;

        if (output)
        {
            auto it = m_tracks_.begin();
            if (it != m_tracks_.end())
                *output = it->second.lastOutput;
        }
        return 0;
    }

    int Tracker::SetConfig(const TM_Config* cfg)
    {
        if (!cfg)
            return -1;
        m_config_ = *cfg;
        return 0;
    }

} // namespace TargetMeasurement
