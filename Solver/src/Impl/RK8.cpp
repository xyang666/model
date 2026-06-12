#include "Solver/Impl/RK8.hpp"
#include <cmath>

SOLVER_NS_BEGIN

errc_t RK8::initialize(ODE& ode)
{
    this->ODEIntegrator::initialize(ode);
    this->resetWorkspace(ode.getDimension(), 10);
    return eNoError;
}

errc_t RK8::singleStep(ODE& ode, double* y, double t0, double h)
{
    const double c2 = 4.0 / 27.0;
    const double c3 = 2.0 / 9.0;
    const double c4 = 1.0 / 3.0;
    const double c5 = 1.0 / 2.0;
    const double c6 = 2.0 / 3.0;
    const double c7 = 1.0 / 6.0;
    const double c9 = 5.0 / 6.0;

    const double a21 = 4.0 / 27.0;
    const double a31 = 1.0 / 18.0;
    const double a32 = 3.0 / 18.0;
    const double a41 = 1.0 / 12.0;
    const double a43 = 3.0 / 12.0;
    const double a51 = 1.0 / 8.0;
    const double a54 = 3.0 / 8.0;
    const double a61 = 13.0  / 54.0;
    const double a63 = -27.0 / 54.0;
    const double a64 = 42.0  / 54.0;
    const double a65 = 8.0   / 54.0;
    const double a71 = 389.0  / 4320.0;
    const double a73 = -54.0  / 4320.0;
    const double a74 = 966.0  / 4320.0;
    const double a75 = -824.0 / 4320.0;
    const double a76 = 243.0  / 4320.0;
    const double a81 = -231.0  / 20.0;
    const double a83 = 81.0    / 20.0;
    const double a84 = -1164.0 / 20.0;
    const double a85 = 656.0   / 20.0;
    const double a86 = -122.0  / 20.0;
    const double a87 = 800.0   / 20.0;
    const double a91 = -127.0 / 288.0;
    const double a93 = 18.0 / 288.0;
    const double a94 = -678.0 / 288.0;
    const double a95 = 456.0 / 288.0;
    const double a96 = -9.0 / 288.0;
    const double a97 = 576.0 / 288.0;
    const double a98 = 4.0 / 288.0;
    const double a10_1 = 1481.0 / 820.0;
    const double a10_3 = -81.0 / 820.0;
    const double a10_4 = 7104.0 / 820.0;
    const double a10_5 = -3376.0 / 820.0;
    const double a10_6 = 72.0 / 820.0;
    const double a10_7 = -5040.0 / 820.0;
    const double a10_8 = -60.0 / 820.0;
    const double a10_9 = 720.0 / 820.0;

    const double b1 = 41.0 / 840.0;
    const double b4 = 27.0 / 840.0;
    const double b5 = 272.0 / 840.0;
    const double b6 = b4;
    const double b7 = 216.0 / 840.0;
    const double b9 = b7;
    const double b10 = 41.0 / 840.0;

    const double* y0 = y;
    double* yf = y;

    auto& wrk = this->getWorkspace();
    int n = wrk.dimension_;
    double
        *k1   = wrk.KArr_[0],
        *k2   = wrk.KArr_[1],
        *k3   = wrk.KArr_[2],
        *k4   = wrk.KArr_[3],
        *k5   = wrk.KArr_[4],
        *k6   = wrk.KArr_[5],
        *k7   = wrk.KArr_[6],
        *k8   = wrk.KArr_[7],
        *k9   = wrk.KArr_[8],
        *k10  = wrk.KArr_[9],
        *ymid = wrk.ymid_;

    errc_t err;

    err = ode.evaluate(t0, y0, k1);

    for (int i = 0; i < n; i++)
        ymid[i] = y0[i] + h * (a21 * k1[i]);
    err |= ode.evaluate(t0 + c2 * h, ymid, k2);

    for (int i = 0; i < n; i++)
        ymid[i] = y0[i] + h * (a31 * k1[i] + a32 * k2[i]);
    err |= ode.evaluate(t0 + c3 * h, ymid, k3);

    for (int i = 0; i < n; i++)
        ymid[i] = y0[i] + h * (a41 * k1[i] + a43 * k3[i]);
    err |= ode.evaluate(t0 + c4 * h, ymid, k4);

    for (int i = 0; i < n; i++)
        ymid[i] = y0[i] + h * (a51 * k1[i] + a54 * k4[i]);
    err |= ode.evaluate(t0 + c5 * h, ymid, k5);

    for (int i = 0; i < n; i++)
        ymid[i] = y0[i] + h * (a61 * k1[i] + a63 * k3[i] + a64 * k4[i] + a65 * k5[i]);
    err |= ode.evaluate(t0 + c6 * h, ymid, k6);

    for (int i = 0; i < n; i++)
        ymid[i] = y0[i] + h * (a71 * k1[i] + a73 * k3[i] + a74 * k4[i] + a75 * k5[i] + a76 * k6[i]);
    err |= ode.evaluate(t0 + c7 * h, ymid, k7);

    for (int i = 0; i < n; i++)
        ymid[i] = y0[i] + h * (a81 * k1[i] + a83 * k3[i] + a84 * k4[i] + a85 * k5[i] + a86 * k6[i] + a87 * k7[i]);
    err |= ode.evaluate(t0 + h, ymid, k8);

    for (int i = 0; i < n; i++)
        ymid[i] = y0[i] + h * (a91 * k1[i] + a93 * k3[i] + a94 * k4[i] + a95 * k5[i] + a96 * k6[i] + a97 * k7[i] + a98 * k8[i]);
    err |= ode.evaluate(t0 + c9 * h, ymid, k9);

    for (int i = 0; i < n; i++)
        ymid[i] = y0[i] + h * (a10_1 * k1[i] + a10_3 * k3[i] + a10_4 * k4[i] + a10_5 * k5[i] + a10_6 * k6[i] + a10_7 * k7[i] + a10_8 * k8[i] + a10_9 * k9[i]);
    err |= ode.evaluate(t0 + h, ymid, k10);

    for (int i = 0; i < n; i++)
        yf[i] = y0[i] + (b1 * k1[i] + b4 * k4[i] + b5 * k5[i] + b6 * k6[i] + b7 * k7[i] + b9 * k9[i] + b10 * k10[i]) * h;

    return err;
}

SOLVER_NS_END
