#pragma once

#include "Tracker.h"

namespace TargetMeasurement
{

class RadarTracker : public Tracker
{
public:
    RadarTracker();

protected:
    double defaultProcessNoise() const override { return 10.0; }

private:
    ObservationData computeObservation(
        const void* measurement,
        const Eigen::VectorXd& predictedState,
        const Eigen::MatrixXd& predictedCov) const;
};

} // namespace TargetMeasurement
