#include "Solver/StateObserver/ODEEventObserver.hpp"
#include "Solver/NLE.hpp"
#include "Solver/ODEIntegrator.hpp"
#include "Solver/Logger.hpp"
#include <cmath>
#include <cstring>

SOLVER_NS_BEGIN

ODEEventObserver::ODEEventObserver(ODEEventDetector* detector)
    : detector_(detector)
{
}

ODEEventObserver::~ODEEventObserver()
{
    if (detector_)
    {
        delete detector_;
        detector_ = nullptr;
    }
}

EODEAction ODEEventObserver::onStateUpdate(double* y, double& x, ODEIntegrator* integrator)
{
    double lastTime = this->lastTime_;
    bool eventOccurred = isEventOccurred(y, x);
    if (eventOccurred)
    {
        double eventTime = x;
        errc_t err = findEventTime(lastTime, x, eventTime, integrator);
        if (err != eNoError)
        {
            // pass
        }
        else
        {
            memcpy(y, integrator->stateTemp(), integrator->getODE()->getDimension() * sizeof(double));
            x = eventTime;
        }
        return EODEAction::eStop;
    }
    return EODEAction::eContinue;
}

bool ODEEventObserver::isEventOccurred(double* y, double& x)
{
    if (this->repeatCount_ < 0)
    {
        this->repeatCount_ = 0;
        this->lastDifference_ = detector_->getDifference(y, x);
        this->lastTime_ = x;
        return false;
    }
    else
    {
        auto direction = detector_->getDirection();
        double difference = detector_->getDifference(y, x);
        bool lastSign = std::signbit(lastDifference_);
        bool currentSign = std::signbit(difference);
        bool occurred;
        if (direction == ODEEventDetector::eBoth)
        {
            occurred = lastSign ^ currentSign;
        }
        else
        {
            static_assert(ODEEventDetector::eDecrease < 0, "value not correct");
            static_assert(ODEEventDetector::eIncrease > 0, "value not correct");
            bool timeSign = std::signbit((x - lastTime_) * (int)direction);
            occurred = (lastSign ^ timeSign) && !(currentSign ^ timeSign);
        }
        lastDifference_ = difference;
        lastTime_ = x;
        if (occurred)
            this->repeatCount_++;
        return this->repeatCount_ >= detector_->getRepeatCount();
    }
}

errc_t ODEEventObserver::findEventTime(double x1, double x2, double& result, ODEIntegrator* integrator)
{
    auto detector = detector_;
    BrentSolver solver(detector->getThreshold());
    int ndim = integrator->getODE()->getDimension();
    errc_t err = solver.solve(
        [detector, integrator, ndim](double t) -> double
        {
            double t0 = integrator->timeAtStepStart();
            memcpy(integrator->stateTemp(), integrator->stateAtStepStart(), ndim * sizeof(double));
            integrator->singleStep(*integrator->getODE(), integrator->stateTemp(), t0, t - t0);
            return detector->getDifference(integrator->stateTemp(), t);
        },
        x1, x2, this->eventTime_
    );
    if (err != eNoError)
    {
        aError("failed to solve event time, error = ", err);
    }
    else
    {
        result = eventTime_;
    }
    return err;
}

SOLVER_NS_END
