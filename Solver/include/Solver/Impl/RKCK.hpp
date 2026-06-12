///
/// @file      RKCK.hpp
/// @brief     Runge-Kutta Cash-Karp 4(5) adaptive method.

#pragma once

#include "Solver/Config.hpp"
#include "Solver/ODEVarStepIntegrator.hpp"

SOLVER_NS_BEGIN

class RKCK : public ODEVarStepIntegrator
{
public:
    errc_t initialize(ODE& ode) final;
    errc_t singleStep(ODE& ode, double* y, double t0, double step) final;
};

SOLVER_NS_END
