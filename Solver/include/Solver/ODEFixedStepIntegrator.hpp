///
/// @file      ODEFixedStepIntegrator.hpp
/// @brief     Fixed-step ODE integrator.

#pragma once

#include "Solver/Config.hpp"
#include "Solver/ODEIntegrator.hpp"
#include "Solver/ODEWorkspace.hpp"

SOLVER_NS_BEGIN

class ODEFixedStepIntegrator : public ODEIntegrator
{
public:
    class Workspace;
    ODEFixedStepIntegrator();
    ~ODEFixedStepIntegrator();
    using ODEIntegrator::integrate;

    double getStepSize() const { return stepSize_; }
    void setStepSize(double stepSize) { stepSize_ = stepSize; }

    int getNumSteps() const;

    const Workspace& getWorkspace() const { return workspace_; }
    Workspace& getWorkspace() { return workspace_; }

    errc_t integrate(ODE& ode, double* y, double& t, double tf) override;

    errc_t integrateStep(ODE& ode, double* y, double& t, double tf) override;

protected:
    void resetWorkspace(int dimension, int stage);

public:
    class Workspace : public ODEWorkspace
    {
    public:
        Workspace();
        ~Workspace();
        void reset(int dimension, int stage);
        void clear();

    public:
        int numSteps_;
        int dimension_;
        int stage_;
        double largestStepSize_;
        double smallestStepSize_;
        double** KArr_;
        double* absErrPerLen_;
        double* ymid_;
        double  nextAbsStepSize_;
    };

private:
    Workspace workspace_;
    double stepSize_;
};

SOLVER_NS_END
