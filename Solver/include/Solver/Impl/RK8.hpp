///
/// @file      RK8.hpp
/// @brief     8th-order 10-stage Runge-Kutta method (Shanks, fixed step).

#pragma once

#include "Solver/Config.hpp"
#include "Solver/ODEFixedStepIntegrator.hpp"

SOLVER_NS_BEGIN

class RK8 : public ODEFixedStepIntegrator
{
public:
    errc_t initialize(ODE& ode) final;
    errc_t singleStep(ODE& ode, double* y, double t0, double step) final;
};

SOLVER_NS_END
