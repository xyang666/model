#include "Solver/StateObserver/ODEStateObserverList.hpp"
#include <algorithm>

SOLVER_NS_BEGIN

ODEStateObserverList::~ODEStateObserverList()
{
    for (auto observer : observers_)
    {
        delete observer;
    }
}

EODEAction ODEStateObserverList::onStateUpdate(double* y, double& x, ODEIntegrator* integrator)
{
    EODEAction action;
    for (auto observer : observers_)
    {
        action = observer->onStateUpdate(y, x, integrator);
        if (action != EODEAction::eContinue)
        {
            break;
        }
    }
    return action;
}

void ODEStateObserverList::removeStateObserver(ODEStateObserver* observer)
{
    auto it = std::find(observers_.begin(), observers_.end(), observer);
    if (it != observers_.end())
    {
        delete *it;
        observers_.erase(it);
    }
}

SOLVER_NS_END
