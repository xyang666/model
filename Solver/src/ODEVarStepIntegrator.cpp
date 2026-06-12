#include "Solver/ODEVarStepIntegrator.hpp"
#include "Solver/ODEStateObserver.hpp"
#include "Solver/MathOperator.hpp"
#include "Solver/Logger.hpp"
#include <cmath>
#include <algorithm>

SOLVER_NS_BEGIN
using namespace math;

ODEVarStepIntegrator::ODEVarStepIntegrator()
    : useMinStep_{false}
    , useMaxStep_{false}
    , warnOnMinStep_{true}
    , maxStepAttempts_{50}
    , minStepSize_{1}
    , maxStepSize_{86400}
    , maxAbsErr_{1e-10}
    , maxRelErr_{1e-13}
    , minStepScaleFactor_{0.5}
    , maxStepScaleFactor_{2.0}
    , safetyCoeffLow_{0.8}
    , safetyCoeffHigh_{0.9}
    , errCtrPowthLow_{0.25}
    , errCtrPowthHigh_{0.1}
{
}

ODEVarStepIntegrator::~ODEVarStepIntegrator()
{
}

double ODEVarStepIntegrator::getLargestStepSize() const
{
    return this->getWorkspace().largestStepSize_;
}

double ODEVarStepIntegrator::getSmallestStepSize() const
{
    return this->getWorkspace().smallestStepSize_;
}

errc_t ODEVarStepIntegrator::integrate(ODE& ode, double* y, double& t, double tf)
{
    this->initialize(ode);
    auto& wrk = this->getWorkspace();
    double absh, h, hmin, hmax;
    double tnew;
    double t0 = t;
    if (this->useMinStep_)
    {
        hmin = this->minStepSize_;
    }
    else
    {
        hmin = 16 * eps(t0);
    }
    if (this->useMaxStep_)
    {
        hmax = this->maxStepSize_;
    }
    else
    {
        hmax = fabs(tf - t0);
    }

    absh = std::abs(this->getStepSize());
    int tdir = sign(tf - t);
    bool final = false;
    int numAttempts = 0;
    const double* y0 = y;
    double* yf = y;
    std::copy_n(y0, wrk.dimension_, this->stateAtStepStart_);
    this->timeAtStepStart_ = t0;
    if (workStateObserver_)
    {
        if (workStateObserver_->onStateUpdate(this->stateAtStepStart_, t, this) == EODEAction::eStop)
        {
            return eNoError;
        }
    }
    while (1)
    {
        if (!this->useMinStep_)
        {
            hmin = 16 * eps(t);
        }
        absh = clamp(absh, hmin, hmax);
        if (!(1.1 * absh < std::abs(tf - t)))
        {
            h = tf - t;
            absh = std::abs(h);
            tnew = tf;
            final = true;
        }
        else
        {
            h = absh * tdir;
            tnew = t + h;
        }
        std::copy_n(this->stateAtStepStart_, wrk.dimension_, this->stateAtStepEnd_);
        this->timeAtStepEnd_ = tnew;
        if (errc_t rc = this->singleStep(ode, this->stateAtStepEnd_, t, h))
        {
            return rc;
        }
        bool isOK = this->isErrorMeet(absh, this->stateAtStepStart_, this->stateAtStepEnd_);
        if (isOK)
        {
            if (workStateObserver_)
            {
                if (workStateObserver_->onStateUpdate(this->stateAtStepEnd_, tnew, this) == EODEAction::eStop)
                {
                    break;
                }
            }
            wrk.numSteps_++;
            wrk.largestStepSize_ = std::max(wrk.largestStepSize_, absh);
            wrk.smallestStepSize_ = std::min(wrk.smallestStepSize_, absh);
            if (final)
            {
                break;
            }
            numAttempts = 0;
        }
        else
        {
            final = false;
            numAttempts++;
            if (numAttempts >= this->maxStepAttempts_)
            {
                if (warnOnMinStep_)
                {
                    aWarning("Max iteration reached.");
                }
                return eErrorMaxIter;
            }
            continue;
        }
        std::swap(this->stateAtStepStart_, this->stateAtStepEnd_);
        this->timeAtStepStart_ = tnew;
        t = tnew;
    }
    std::copy_n(this->stateAtStepEnd(), wrk.dimension_, yf);
    t = tnew;
    return eNoError;
}

errc_t ODEVarStepIntegrator::integrateStep(ODE& ode, double* y, double& t, double tf)
{
    auto& wrk = this->getWorkspace();
    double& absh = wrk.nextAbsStepSize_;
    double step = tf - t;
    int tdir = sign(step);
    double stepabs = std::abs(step);
    if (stepabs < absh)
    {
        absh = stepabs;
    }
    bool isOK = false;
    int numAttempts = 0;
    const double* y0 = y;
    double h;
    do
    {
        h = absh * tdir;
        if (errc_t rc = this->singleStep(ode, y, t, h))
        {
            return rc;
        }
        isOK = this->isErrorMeet(absh, y0, y);
        if (this->useMinStep_)
        {
            absh = std::max(absh, this->minStepSize_);
        }
        if (this->useMaxStep_)
        {
            absh = std::min(absh, this->maxStepSize_);
        }
        if (numAttempts++ >= this->maxStepAttempts_)
        {
            aWarning("Max iteration reached.");
            return eErrorMaxIter;
        }
    } while (!isOK);
    t += h;
    return eNoError;
}

bool ODEVarStepIntegrator::isErrorMeet(double& absh, const double* y, const double* ynew)
{
    using std::max;
    using std::abs;

    double maxErrRatio;
    double err;
    auto& wrk = this->getWorkspace();
    int dim = wrk.dimension_;
    double threshold = this->maxAbsErr_ / this->maxRelErr_;
    double rtol = this->maxRelErr_;

    // L-infinity norm error control
    {
        double maxTemp = 0;
        for (int i = 0; i < dim; i++)
        {
            double temp = std::abs(wrk.absErrPerLen_[i]) / std::max(std::max(std::abs(ynew[i]), std::abs(y[i])), threshold);
            if (temp > maxTemp)
            {
                maxTemp = temp;
            }
        }
        err = absh * maxTemp;
        maxErrRatio = err / rtol;
    }

    bool isOK;
    {
        if (maxErrRatio > 1)
        {
            double scale = this->safetyCoeffLow_ * pow(maxErrRatio, -this->errCtrPowthLow_);
            scale = std::max(scale, this->minStepScaleFactor_);
            absh *= scale;
            isOK = false;
        }
        else
        {
            double scale = this->safetyCoeffHigh_ * pow(maxErrRatio, -this->errCtrPowthHigh_);
            scale = std::min(scale, this->maxStepScaleFactor_);
            absh *= scale;
            isOK = true;
        }
    }
    return isOK;
}

SOLVER_NS_END
