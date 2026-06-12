#include "Solver/Impl/RK4.hpp"

SOLVER_NS_BEGIN

errc_t RK4::initialize(ODE& ode)
{
    this->ODEIntegrator::initialize(ode);
    this->resetWorkspace(ode.getDimension(), 4);
    return eNoError;
}

errc_t RK4::singleStep(ODE& ode, double* y, double t0, double step)
{
    int err;
    auto& wrk = this->getWorkspace();
    int ndim = wrk.dimension_;
    double t = t0;
    double* k1 = wrk.KArr_[0];
    double* k2 = wrk.KArr_[1];
    double* k3 = wrk.KArr_[2];
    double* k4 = wrk.KArr_[3];
    double* ymid = wrk.ymid_;

    double hh = step * 0.5;
    const double* y0 = y;
    double* yf = y;

    // k1
    err = ode.evaluate(t, y0, k1);

    // k2
    for (int i = 0; i < ndim; i++)
        ymid[i] = y0[i] + hh * k1[i];
    err |= ode.evaluate(t0 + hh, ymid, k2);

    // k3
    for (int i = 0; i < ndim; i++)
        ymid[i] = y0[i] + hh * k2[i];
    err |= ode.evaluate(t0 + hh, ymid, k3);

    // k4
    for (int i = 0; i < ndim; i++)
        ymid[i] = y0[i] + step * k3[i];
    err |= ode.evaluate(t0 + step, ymid, k4);

    // yf
    for (int i = 0; i < ndim; i++)
        yf[i] = y0[i] + step * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]) / 6.0;

    return err;
}

SOLVER_NS_END
