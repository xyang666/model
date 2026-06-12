///
/// @file      ODEVarStepIntegrator.hpp
/// @brief     Variable-step (adaptive) ODE integrator.

#pragma once

#include "Solver/Config.hpp"
#include "Solver/ODEFixedStepIntegrator.hpp"

SOLVER_NS_BEGIN

class ODEVarStepIntegrator : public ODEFixedStepIntegrator
{
public:
    ODEVarStepIntegrator();
    ~ODEVarStepIntegrator();
    using ODEFixedStepIntegrator::integrate;

    errc_t integrate(ODE& ode, double* y, double& t, double tf) final;

    errc_t integrateStep(ODE& ode, double* y, double& t, double tf) final;

    void setMaxAbsErr(double maxAbsErr) { maxAbsErr_ = maxAbsErr; }
    void setMaxRelErr(double maxRelErr) { maxRelErr_ = maxRelErr; }
    void setInitialStepSize(double initialStepSize) { setStepSize(initialStepSize); }

    double getLargestStepSize() const;
    double getSmallestStepSize() const;

    using ODEFixedStepIntegrator::getNumSteps;

protected:
    bool isErrorMeet(double& absh, const double* y, const double* ynew);

private:
    bool   useMinStep_;          ///< Enable lower bound on step size
    bool   useMaxStep_;          ///< Enable upper bound on step size
    bool   warnOnMinStep_;       ///< Emit warning when step-size floor is hit
    int    maxStepAttempts_;     ///< Max retries per step before giving up / warning
    double minStepSize_;         ///< Absolute minimum step size
    double maxStepSize_;         ///< Absolute maximum step size
    double maxAbsErr_;           ///< Absolute error tolerance (used in error-per-step criterion)
    double maxRelErr_;           ///< Relative error tolerance
    double minStepScaleFactor_;  ///< Minimum factor by which step size may shrink in one go
    double maxStepScaleFactor_;  ///< Maximum factor by which step size may grow in one go
    double safetyCoeffLow_;      ///< Safety coefficient when error exceeds tolerance (step rejected)
    double safetyCoeffHigh_;     ///< Safety coefficient when error is within tolerance (step accepted)
    double errCtrPowthLow_;      ///< Error-controller exponent when step is rejected
    double errCtrPowthHigh_;     ///< Error-controller exponent when step is accepted
};

SOLVER_NS_END
