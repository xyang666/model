///
/// @file      ODEEventObserver.hpp
/// @brief     Event observer wrapping a detector with root-finding for precise event times.

#pragma once

#include "Solver/Config.hpp"
#include "Solver/ODEStateObserver.hpp"
#include "Solver/ODEEventDetector.hpp"
#include <limits>

SOLVER_NS_BEGIN

class ODEEventObserver : public ODEStateObserver
{
public:
    ODEEventObserver() = default;
    explicit ODEEventObserver(ODEEventDetector* detector);
    ~ODEEventObserver() override;
    EODEAction onStateUpdate(double* y, double& x, ODEIntegrator* integrator) final;
    ODEEventDetector* getEventDetector() const { return detector_; }
    bool isEventOccurred(double* y, double& x);
    errc_t findEventTime(double x1, double x2, double& result, ODEIntegrator* integrator);

protected:
    friend class ODEEventDetectorList;
    ODEEventDetector* detector_{nullptr};
    double lastDifference_{std::numeric_limits<double>::quiet_NaN()};
    double lastTime_{std::numeric_limits<double>::quiet_NaN()};
    double eventTime_{std::numeric_limits<double>::quiet_NaN()};
    int    repeatCount_{-1};
};

SOLVER_NS_END
