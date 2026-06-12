///
/// @file      ODEStateVectorCollector.hpp
/// @brief     Collects time/state history during integration.

#pragma once

#include "Solver/Config.hpp"
#include "Solver/ODEStateObserver.hpp"
#include <vector>

SOLVER_NS_BEGIN

class ODEStateVectorCollector : public ODEStateObserver
{
public:
    ODEStateVectorCollector(int ndim);
    ~ODEStateVectorCollector() = default;

    EODEAction onStateUpdate(double* y, double& x, ODEIntegrator* integrator) override;

    std::vector<double>& x() { return x_; }
    std::vector<std::vector<double>>& y() { return y_; }

protected:
    int ndim_;
    std::vector<double> x_;
    std::vector<std::vector<double>> y_;
};

SOLVER_NS_END
