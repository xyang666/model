///
/// @file      RKF56.hpp
/// @brief     Runge-Kutta-Fehlberg 5(6) adaptive method.

#pragma once

#include "Solver/Config.hpp"
#include "Solver/ODEVarStepIntegrator.hpp"

SOLVER_NS_BEGIN

class RKF56 : public ODEVarStepIntegrator
{
public:
    errc_t initialize(ODE& ode) final;
    errc_t singleStep(ODE& ode, double* y, double t0, double step) final;
};

SOLVER_NS_END
