#pragma once

#include "TM_Interface.h"
#include "filter/FilterBase.h"
#include "filter/QualityEstimator.h"
#include "utils/CoordinateTransform.h"
#include <Eigen/Dense>
#include <functional>
#include <memory>
#include <map>

namespace TargetMeasurement
{

struct TrackData
{
    int targetId;
    int detectCount;
    int totalCount;
    double lastUpdateTime;
    double lastMeasTime;
    std::unique_ptr<Filter::FilterBase> filter;
    Filter::QualityEstimator quality;
    TM_LocationOutput lastOutput;
};

class Tracker
{
public:
    virtual ~Tracker();

    bool Initialize(const TM_Config* cfg);

    int  ProcessMeasurement(const void* measurement,
                            TM_LocationOutput* output);

    int  NoDetectUpdate(double simTime, TM_LocationOutput* output);
    int  Reset();

    int  GetTrackCount() const;
    int  GetTrack(int index, TM_LocationOutput* output) const;

    int  SetConfig(const TM_Config* cfg);
    int  GetConfig(TM_Config* cfg) const;

protected:
    Tracker();

    struct ObservationData
    {
        Eigen::VectorXd measurement;
        Eigen::MatrixXd R;
    };

    using SensorObsFn = std::function<ObservationData(const void*, const Eigen::VectorXd&, const Eigen::MatrixXd&)>;
    void setSensorFunc(int sensorType, SensorObsFn func);

    virtual double defaultProcessNoise() const { return 5.0; }
    virtual void fill_output(double simTime,
                             TrackData& track,
                             TM_LocationOutput* out) const;

    const TM_Config& getConfig() const { return m_config_; }
    TrackData* getTrack(int targetId);
    void deleteStaleTracks(double simTime);

private:
    TM_Config m_config_;
    bool m_initialized_ = false;
    double m_lastSimTime_ = 0.0;
    std::map<int, TrackData> m_tracks_;
    SensorObsFn obsFuncs_[3];
};

} // namespace TargetMeasurement
