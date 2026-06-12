///
/// @file      RK4.hpp
/// @brief     4th-order Runge-Kutta method (fixed step).

#pragma once

#include "Solver/Config.hpp"
#include "Solver/ODEFixedStepIntegrator.hpp"

SOLVER_NS_BEGIN

class RK4 : public ODEFixedStepIntegrator
{
public:
    errc_t initialize(ODE& ode) final;
    errc_t singleStep(ODE& ode, double* y, double t0, double step) final;
};

SOLVER_NS_END
