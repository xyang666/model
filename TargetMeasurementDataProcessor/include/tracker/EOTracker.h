#pragma once

#include "Tracker.h"

namespace TargetMeasurement
{

class EOTracker : public Tracker
{
public:
    EOTracker(double focalLenPx = 1000.0, double cx = 320.0, double cy = 240.0);

protected:
    double defaultProcessNoise() const override { return 5.0; }

private:
    ObservationData computeObservation(
        const void* measurement,
        const Eigen::VectorXd& predictedState,
        const Eigen::MatrixXd& predictedCov) const;

    double fx_, fy_;
    double cx_, cy_;
};

} // namespace TargetMeasurement
