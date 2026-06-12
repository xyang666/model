#include "Solver/Impl/RKF56.hpp"

SOLVER_NS_BEGIN

static const double
    ch56_6[8]    { 7./1408., 0.   , 1125./2816., 9./32., 125./768., 0.    , 5./66., 5./66. },
    alph56[8]    {0.       , 1./6., 4./15.     , 2./3. , 4./5.    , 1.    , 0.    , 1.     },
    beta56[8][7]{
        {},
        {1./6.     , 0.      ,0.       , 0.       , 0.      , 0. , 0.},
        {4./75     ,16./75.  ,0.       , 0.       , 0.      , 0. , 0.},
        {5./6.     ,-8./3.   ,5./2     , 0.       , 0.      , 0. , 0.},
        {-8./5.    ,144./25. ,-4       , 16./25.  , 0.      , 0. , 0.},
        {361./320. ,-18./5   ,407./128 ,-11./80.  , 55./128., 0. , 0.},
        {-11./640. , 0.      ,11./256. ,-11./160. , 11./256., 0. , 0.},
        {93./640.  , -18./5. ,803./256.,-11./160. , 99./256., 0. , 1.}};

errc_t RKF56::initialize(ODE& ode)
{
    this->ODEIntegrator::initialize(ode);
    this->resetWorkspace(ode.getDimension(), 8);
    return eNoError;
}

errc_t RKF56::singleStep(ODE& ode, double* y, double t0, double h)
{
    const double err_factor = -5.0 / 66.0;

    auto& wrk = this->getWorkspace();
    int ndim = wrk.dimension_;
    auto KArr = wrk.KArr_;
    double* ymid = wrk.ymid_;
    const double* y0 = y;
    double* yf = y;

    for (int k = 0; k < 8; k++)
    {
        for (int i = 0; i < ndim; i++)
        {
            double sumtemp = 0.0;
            for (int j = 0; j < k; j++)
            {
                sumtemp += beta56[k][j] * KArr[j][i];
            }
            ymid[i] = y0[i] + sumtemp * h;
        }
        if (errc_t err = ode.evaluate(t0 + alph56[k] * h, ymid, KArr[k]))
        {
            return err;
        }
    }

    for (int i = 0; i < ndim; i++)
    {
        double sumtemp = 0.0;
        for (int k = 0; k < 8; k++)
        {
            sumtemp += ch56_6[k] * KArr[k][i];
        }
        yf[i] = y0[i] + sumtemp * h;
        wrk.absErrPerLen_[i] = err_factor * (
            + KArr[0][i]
            + KArr[5][i]
            - KArr[6][i]
            - KArr[7][i]
        );
    }

    return eNoError;
}

SOLVER_NS_END
