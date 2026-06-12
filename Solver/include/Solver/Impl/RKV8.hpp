///
/// @file      RKV8.hpp
/// @brief     Verner's 8th-order Runge-Kutta method (fixed step).

#pragma once

#include "Solver/Config.hpp"
#include "Solver/ODEFixedStepIntegrator.hpp"

SOLVER_NS_BEGIN

class RKV8 : public ODEFixedStepIntegrator
{
public:
    errc_t initialize(ODE& ode) final;
    errc_t singleStep(ODE& ode, double* y, double t0, double step) final;
};

SOLVER_NS_END
