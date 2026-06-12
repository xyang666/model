///
/// @file      ABM.hpp
/// @brief     Adams-Bashforth-Moulton predictor-corrector method (variable-step, order 1–6).
/// @details   PECE mode with variable-coefficient formulas computed from the actual
///            time grid each step.  Bootstrap via RK4.  Error estimated by Milne's device
///            (difference between predictor and corrector).

#pragma once

#include "Solver/Config.hpp"
#include "Solver/ODEVarStepIntegrator.hpp"

SOLVER_NS_BEGIN

class ABM : public ODEVarStepIntegrator
{
public:
    explicit ABM(int order = 4);
    ~ABM() override;

    errc_t initialize(ODE& ode) final;
    errc_t singleStep(ODE& ode, double* y, double t0, double step) final;

private:
    errc_t bootstrapStep(ODE& ode, double* y, double t0, double h);
    errc_t abmStep(ODE& ode, double* y, double t0, double h);
    void setBootstrapError(const double* y, double h);

    // Compute integration weights for polynomial through (tau[j], f[j]),
    // integrated from t0 to t0+h.  Stores result in w[0..k-1].
    static void computeWeights(double* w, const double* tau, int k, double t0, double h);

    int    order_;
    int    histIdx_;
    int    histCount_;
    double initStepSize_;
    double tHist_[6];    // time values aligned with KArr_ history (circular buffer)
};

SOLVER_NS_END
