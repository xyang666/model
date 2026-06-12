#include "Solver/ODEFixedStepIntegrator.hpp"
#include "Solver/MathOperator.hpp"
#include "Solver/ODEStateObserver.hpp"
#include <limits>
#include <cmath>
#include <algorithm>

SOLVER_NS_BEGIN

using namespace math;

ODEFixedStepIntegrator::Workspace::Workspace()
    : numSteps_(0)
    , dimension_(0)
    , stage_(0)
    , largestStepSize_(0)
    , smallestStepSize_(std::numeric_limits<double>::max())
    , KArr_(nullptr)
    , absErrPerLen_(nullptr)
    , ymid_(nullptr)
    , nextAbsStepSize_(0)
{
}

void ODEFixedStepIntegrator::Workspace::reset(int dimension, int stage)
{
    if (dimension > this->dimension_ || stage > this->stage_)
    {
        clear();
        KArr_ = new double*[stage];
        for (int i = 0; i < stage; i++)
        {
            KArr_[i] = new double[dimension];
        }
        absErrPerLen_ = new double[dimension];
        ymid_ = new double[dimension];
    }
    numSteps_ = 0;
    largestStepSize_ = 0;
    smallestStepSize_ = std::numeric_limits<double>::max();
    dimension_ = dimension;
    stage_ = stage;
}

void ODEFixedStepIntegrator::Workspace::clear()
{
    if (KArr_ != nullptr)
    {
        for (int i = 0; i < stage_; i++)
        {
            delete[] KArr_[i];
        }
        delete[] KArr_;
        KArr_ = nullptr;
    }
    if (absErrPerLen_ != nullptr)
    {
        delete[] absErrPerLen_;
        absErrPerLen_ = nullptr;
    }
    if (ymid_ != nullptr)
    {
        delete[] ymid_;
        ymid_ = nullptr;
    }
}

ODEFixedStepIntegrator::Workspace::~Workspace()
{
    clear();
}

ODEFixedStepIntegrator::ODEFixedStepIntegrator()
    : stepSize_(60)
{
}

ODEFixedStepIntegrator::~ODEFixedStepIntegrator()
{
    if (stateAtStepStart_)
    {
        delete[] stateAtStepStart_;
    }
    if (stateAtStepEnd_)
    {
        delete[] stateAtStepEnd_;
    }
    if (stateTemp_)
    {
        delete[] stateTemp_;
    }
}

int ODEFixedStepIntegrator::getNumSteps() const
{
    return this->getWorkspace().numSteps_;
}

errc_t ODEFixedStepIntegrator::integrate(ODE& ode, double* y, double& t, double tf)
{
    this->initialize(ode);
    auto& wrk = this->getWorkspace();

    errc_t err = eNoError;
    double stepSize = this->stepSize_;
    if (stepSize <= 0)
    {
        stepSize = 60;
    }
    double t0 = t;
    double habs = std::min(fabs(stepSize), fabs(tf - t0));
    int tdir = sign(tf - t0);

    if (workStateObserver_)
    {
        if (workStateObserver_->onStateUpdate(y, t, this) == EODEAction::eStop)
        {
            return eNoError;
        }
    }
    while (tdir * (tf - t) > 0)
    {
        double h = tdir * std::min(habs, std::abs(tf - t));
        err = this->singleStep(ode, y, t, h);
        if (err != eNoError)
        {
            return err;
        }
        t += h;
        wrk.numSteps_++;
        if (workStateObserver_)
        {
            if (workStateObserver_->onStateUpdate(y, t, this) == EODEAction::eStop)
            {
                return eNoError;
            }
        }
    }
    return eNoError;
}

errc_t ODEFixedStepIntegrator::integrateStep(ODE& ode, double* y, double& t, double tf)
{
    double absh = this->stepSize_;
    double step = tf - t;
    int tdir = sign(step);
    double stepabs = std::abs(step);
    if (stepabs < absh)
    {
        absh = stepabs;
    }
    double h = absh * tdir;
    errc_t err = this->singleStep(ode, y, t, h);
    if (err != eNoError)
    {
        return err;
    }
    t += h;
    return eNoError;
}

void ODEFixedStepIntegrator::resetWorkspace(int dimension, int stage)
{
    if (dimension > this->getWorkspace().dimension_)
    {
        if (stateAtStepStart_)
        {
            delete[] stateAtStepStart_;
        }
        if (stateAtStepEnd_)
        {
            delete[] stateAtStepEnd_;
        }
        if (stateTemp_)
        {
            delete[] stateTemp_;
        }
        stateAtStepStart_ = new double[dimension];
        stateAtStepEnd_ = new double[dimension];
        stateTemp_ = new double[dimension];
    }
    this->getWorkspace().reset(dimension, stage);
}

SOLVER_NS_END
