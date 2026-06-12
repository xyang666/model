#include "Solver/StateObserver/ODEInnerStateObserver.hpp"
#include "Solver/ODEIntegrator.hpp"

SOLVER_NS_BEGIN

EODEAction ODEInnerStateObserver::onStateUpdate(double* y, double& x, ODEIntegrator* integrator)
{
    EODEAction action = integrator_->eventDetectorList_.onStateUpdate(y, x, integrator);
    if (action == EODEAction::eStop)
    {
        integrator_->stateObserverList_.onStateUpdate(y, x, integrator);
        return action;
    }
    return integrator_->stateObserverList_.onStateUpdate(y, x, integrator);
}

SOLVER_NS_END
