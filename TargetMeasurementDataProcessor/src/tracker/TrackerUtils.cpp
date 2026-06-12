#include "tracker/Tracker.h"

namespace TargetMeasurement
{

int Tracker::GetConfig(TM_Config* cfg) const
{
    if (!cfg)
        return -1;
    *cfg = m_config_;
    return 0;
}

void Tracker::fill_output(double simTime,
                          TrackData& track,
                          TM_LocationOutput* out) const
{
    if (!out || !track.filter)
        return;

    Eigen::VectorXd state = track.filter->state();
    Eigen::MatrixXd cov = track.filter->covariance();

    double position[3] = {state(0), state(1), state(2)};
    double velocity[3] = {state(3), state(4), state(5)};

    out->position[0] = position[0]; out->position[1] = position[1]; out->position[2] = position[2];
    out->velocity[0] = velocity[0]; out->velocity[1] = velocity[1]; out->velocity[2] = velocity[2];
    out->referenceFrame = TM_FRAME_WORLD;

    for (int r = 0; r < 6; ++r)
        for (int c = 0; c < 6; ++c)
            out->stateCovariance[r * 6 + c] = cov(r, c);

    track.quality.updateCovariance(cov);

    out->positionRmsError = track.quality.getPositionRmsError();
    out->velocityRmsError = track.quality.getVelocityRmsError();
    out->chiSquared       = track.quality.getChiSquared();
    out->avgChiSquared    = track.quality.getAvgChiSquared();
    out->trackQuality     = track.quality.getTrackQuality();

    double sigmas[3];
    track.quality.getPositionSigmas(sigmas);
    out->sigmaA = sigmas[0]; out->sigmaB = sigmas[1]; out->sigmaC = sigmas[2];

    out->updateTime  = simTime;
    out->filterState = track.quality.getFilterState();
    out->valid       = 1;
    out->updateCount = track.quality.getPass();
}

} // namespace TargetMeasurement
