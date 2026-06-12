///
/// @file      ODEInnerStateObserver.hpp
/// @brief     Internal observer that coordinates event detection and state observation.

#pragma once

#include "Solver/Config.hpp"
#include "Solver/ODEStateObserver.hpp"

SOLVER_NS_BEGIN

class ODEIntegrator;

class ODEInnerStateObserver : public ODEStateObserver
{
public:
    ODEInnerStateObserver(ODEIntegrator* integrator) : integrator_(integrator) {}
    ~ODEInnerStateObserver() = default;
    EODEAction onStateUpdate(double* y, double& x, ODEIntegrator* integrator) final;

protected:
    ODEIntegrator* integrator_ = nullptr;
};

SOLVER_NS_END
