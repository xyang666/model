#include "Solver/ODEIntegrator.hpp"
#include "Solver/StateObserver/ODEStateVectorCollector.hpp"

SOLVER_NS_BEGIN

ODEIntegrator::~ODEIntegrator()
{
    if (innerStateObserver_)
    {
        delete innerStateObserver_;
        innerStateObserver_ = nullptr;
    }
}

errc_t ODEIntegrator::initialize(ODE& ode)
{
    this->ode_ = &ode;
    this->initWorkStateObserver();
    return eNoError;
}

errc_t ODEIntegrator::integrate(
    ODE& ode, double* y, double& t, double tf,
    std::vector<double>& xlist, std::vector<std::vector<double>>& ylist)
{
    ODEStateVectorCollector* collector = new ODEStateVectorCollector(ode.getDimension());
    this->addStateObserver(collector);
    errc_t rc = this->integrate(ode, y, t, tf);
    xlist = std::move(collector->x());
    ylist = std::move(collector->y());
    this->removeStateObserver(collector);
    return rc;
}

void ODEIntegrator::addEventDetector(ODEEventDetector* detector)
{
    eventDetectorList_.addEventDetector(detector);
}

void ODEIntegrator::removeEventDetector(ODEEventDetector* detector)
{
    eventDetectorList_.removeEventDetector(detector);
}

void ODEIntegrator::addStateObserver(ODEStateObserver* observer)
{
    stateObserverList_.addStateObserver(observer);
}

void ODEIntegrator::removeStateObserver(ODEStateObserver* observer)
{
    stateObserverList_.removeStateObserver(observer);
}

void ODEIntegrator::initWorkStateObserver()
{
    if (!stateObserverList_.empty())
    {
        if (!eventDetectorList_.empty())
        {
            if (!innerStateObserver_)
            {
                innerStateObserver_ = new ODEInnerStateObserver(this);
            }
            workStateObserver_ = innerStateObserver_;
        }
        else if (stateObserverList_.size() == 1)
        {
            workStateObserver_ = &stateObserverList_[0];
        }
        else
        {
            workStateObserver_ = &stateObserverList_;
        }
    }
    else if (eventDetectorList_.size() == 1)
    {
        workStateObserver_ = &eventDetectorList_[0];
    }
    else
    {
        workStateObserver_ = &eventDetectorList_;
    }
}

SOLVER_NS_END
