#include "Solver/StateObserver/ODEStateVectorCollector.hpp"

SOLVER_NS_BEGIN

ODEStateVectorCollector::ODEStateVectorCollector(int ndim)
    : ndim_(ndim)
{
}

EODEAction ODEStateVectorCollector::onStateUpdate(double* y, double& x, ODEIntegrator* integrator)
{
    x_.push_back(x);
    y_.push_back(std::vector<double>(y, y + ndim_));
    return EODEAction::eContinue;
}

SOLVER_NS_END
