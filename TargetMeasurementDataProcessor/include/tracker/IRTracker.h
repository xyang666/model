#pragma once

#include "Tracker.h"

namespace TargetMeasurement
{

class IRTracker : public Tracker
{
public:
    IRTracker(double focalLenPx = 500.0, double cx = 160.0, double cy = 120.0);

protected:
    double defaultProcessNoise() const override { return 3.0; }

private:
    ObservationData computeObservation(
        const void* measurement,
        const Eigen::VectorXd& predictedState,
        const Eigen::MatrixXd& predictedCov) const;

    double fx_, fy_;
    double cx_, cy_;
};

} // namespace TargetMeasurement
