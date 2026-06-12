#include "Solver/StateObserver/ODEEventDetectorList.hpp"
#include "Solver/ODEIntegrator.hpp"
#include <algorithm>
#include <cstring>

SOLVER_NS_BEGIN

ODEEventDetectorList::~ODEEventDetectorList()
{
    for (auto observer : eventObservers_)
    {
        delete observer;
    }
}

EODEAction ODEEventDetectorList::onStateUpdate(double* y, double& x, ODEIntegrator* integrator)
{
    ODEEventObserver* firstObservered = nullptr;
    double firstEventTime = std::numeric_limits<double>::max();
    for (auto observer : eventObservers_)
    {
        double lastTime = observer->lastTime_;
        bool eventOccurred = observer->isEventOccurred(y, x);
        if (eventOccurred)
        {
            errc_t err = observer->findEventTime(lastTime, x, observer->eventTime_, integrator);
            if (err == eNoError && observer->eventTime_ < firstEventTime)
            {
                firstEventTime = observer->eventTime_;
                firstObservered = observer;
            }
        }
    }
    if (firstObservered != nullptr)
    {
        auto ode = integrator->getODE();
        int ndim = ode->getDimension();
        double t0 = integrator->timeAtStepStart();
        double step = firstEventTime - t0;
        memcpy(y, integrator->stateAtStepStart(), ndim * sizeof(double));
        integrator->singleStep(*ode, y, t0, step);
        x = firstEventTime;
        return EODEAction::eStop;
    }
    return EODEAction::eContinue;
}

void ODEEventDetectorList::addEventDetector(ODEEventDetector* detector)
{
    eventObservers_.emplace_back(new ODEEventObserver(detector));
}

void ODEEventDetectorList::removeEventDetector(ODEEventDetector* detector)
{
    auto it = std::find_if(eventObservers_.begin(), eventObservers_.end(),
        [detector](ODEEventObserver* obs) { return obs->getEventDetector() == detector; });
    if (it != eventObservers_.end())
    {
        delete *it;
        eventObservers_.erase(it);
    }
}

SOLVER_NS_END
