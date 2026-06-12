#include "Solver/Impl/RKF45.hpp"

SOLVER_NS_BEGIN

static const double
    ch45_4[6]    { 25./216., 0.   , 1408./2565. , 2197./4104.  ,  -1./5. , 0.    },
    alph45[6]    {0.       , 1./4., 3./8.       , 12./13.      ,  1.     , 1./2. },
    beta45[6][5] {
        {},
        {1./4.      , 0.          ,0.         , 0.         , 0.      },
        {3./32.     ,9./32.       ,0.         , 0.         , 0.      },
        {1932./2197.,-7200./2197. ,7296./2197 , 0.         , 0.      },
        {439./216.  ,-8.          ,3680./513. , -845./4104 , 0.      },
        {-8./27.    ,2.           ,-3544./2565, 1859./4104., -11/40.}};

errc_t RKF45::initialize(ODE& ode)
{
    this->ODEIntegrator::initialize(ode);
    this->resetWorkspace(ode.getDimension(), 6);
    return eNoError;
}

errc_t RKF45::singleStep(ODE& ode, double* y, double t0, double h)
{
    auto& wrk = this->getWorkspace();
    int ndim = wrk.dimension_;
    auto KArr = wrk.KArr_;
    double* ymid = wrk.ymid_;
    const double* y0 = y;
    double* yf = y;

    for (int k = 0; k < 6; k++)
    {
        for (int i = 0; i < ndim; i++)
        {
            double sumtemp = 0.0;
            for (int j = 0; j < k; j++)
            {
                sumtemp += beta45[k][j] * KArr[j][i];
            }
            ymid[i] = y0[i] + sumtemp * h;
        }
        if (errc_t err = ode.evaluate(t0 + alph45[k] * h, ymid, KArr[k]))
        {
            return err;
        }
    }

    for (int i = 0; i < ndim; i++)
    {
        double sumtemp = 0.0;
        for (int k = 0; k < 5; k++)
        {
            sumtemp += ch45_4[k] * KArr[k][i];
        }
        yf[i] = y0[i] + sumtemp * h;
        wrk.absErrPerLen_[i] = (
            - 1./360.      * KArr[0][i]
            + 128./4275.   * KArr[2][i]
            + 2197./75240. * KArr[3][i]
            - 1./50.       * KArr[4][i]
            - 2./55.       * KArr[5][i]
        );
    }
    return eNoError;
}

SOLVER_NS_END
