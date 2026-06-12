#include "Solver/Impl/ABM.hpp"
#include <algorithm>
#include <cmath>
#include <cstring>

SOLVER_NS_BEGIN

ABM::ABM(int order)
    : order_(order)
    , histIdx_(0)
    , histCount_(0)
    , initStepSize_(0.0)
{
    if (order_ < 1) order_ = 1;
    if (order_ > 6) order_ = 6;
    for (int i = 0; i < 6; i++) tHist_[i] = 0.0;
}

ABM::~ABM() = default;

errc_t ABM::initialize(ODE& ode)
{
    this->ODEIntegrator::initialize(ode);
    // KArr_[0     .. order_-1]: derivative history (order_ slots)
    // KArr_[order_ .. order_+3]: RK4 bootstrap k1..k4 (4 slots)
    // KArr_[order_+4]         : y_n save
    // KArr_[order_+5]         : f^P / temp
    // wrk.ymid_               : y^P save (error estimation)
    this->resetWorkspace(ode.getDimension(), order_ + 6);
    histIdx_ = 0;
    histCount_ = 0;
    initStepSize_ = 0.0;
    for (int i = 0; i < 6; i++) tHist_[i] = 0.0;
    return eNoError;
}

// ---------------------------------------------------------------------------
// Compute integration weights for a polynomial interpolant through
// (tau[0], f[0]), …, (tau[k-1], f[k-1]) integrated from t0 to t0+h.
//
// Uses shifted times  s_j = (tau[j] - t0) / h  so that the Vandermonde
// system is well-conditioned (all |s_j| = O(1)).  The quadrature condition
//    Σ w_j · s_j^m  =  1/(m+1)          m = 0,…,k-1
// is independent of t0 and h.
// ---------------------------------------------------------------------------
void ABM::computeWeights(double* w, const double* tau, int k, double t0, double h)
{
    double A[36];  // 6×6 max

    for (int j = 0; j < k; j++)
    {
        double sj = (tau[j] - t0) / h;
        A[j] = 1.0;                          // sj^0
        for (int m = 1; m < k; m++)
            A[m * k + j] = A[(m - 1) * k + j] * sj;
    }

    // RHS: ∫_0^1 s^m ds = 1/(m+1)
    for (int m = 0; m < k; m++)
        w[m] = 1.0 / (m + 1);

    // Gaussian elimination with partial pivoting
    for (int col = 0; col < k; col++)
    {
        int pivot = col;
        double best = std::abs(A[col * k + col]);
        for (int row = col + 1; row < k; row++)
        {
            double v = std::abs(A[row * k + col]);
            if (v > best) { best = v; pivot = row; }
        }
        if (pivot != col)
        {
            for (int j = col; j < k; j++)
                std::swap(A[col * k + j], A[pivot * k + j]);
            std::swap(w[col], w[pivot]);
        }
        double piv = A[col * k + col];
        for (int row = col + 1; row < k; row++)
        {
            double fac = A[row * k + col] / piv;
            for (int j = col; j < k; j++)
                A[row * k + j] -= fac * A[col * k + j];
            w[row] -= fac * w[col];
        }
    }

    // Back-substitution
    for (int col = k - 1; col >= 0; col--)
    {
        double s = w[col];
        for (int j = col + 1; j < k; j++)
            s -= A[col * k + j] * w[j];
        w[col] = s / A[col * k + col];
    }
}

// ---------------------------------------------------------------------------
// Bootstrap  —  classic RK4
// ---------------------------------------------------------------------------
errc_t ABM::bootstrapStep(ODE& ode, double* y, double t0, double h)
{
    auto& wrk = this->getWorkspace();
    int ndim = wrk.dimension_;
    double* k1 = wrk.KArr_[order_ + 0];
    double* k2 = wrk.KArr_[order_ + 1];
    double* k3 = wrk.KArr_[order_ + 2];
    double* k4 = wrk.KArr_[order_ + 3];
    double* ymid = wrk.ymid_;
    errc_t err = eNoError;

    double hh = h * 0.5;
    double h6 = h / 6.0;

    if (histCount_ > 0)
    {
        int idx = (histIdx_ - 1 + order_) % order_;
        for (int i = 0; i < ndim; i++)
            k1[i] = wrk.KArr_[idx][i];
    }
    else
    {
        err = ode.evaluate(t0, y, k1);
    }

    for (int i = 0; i < ndim; i++)
        ymid[i] = y[i] + hh * k1[i];
    err |= ode.evaluate(t0 + hh, ymid, k2);

    for (int i = 0; i < ndim; i++)
        ymid[i] = y[i] + hh * k2[i];
    err |= ode.evaluate(t0 + hh, ymid, k3);

    for (int i = 0; i < ndim; i++)
        ymid[i] = y[i] + h * k3[i];
    err |= ode.evaluate(t0 + h, ymid, k4);

    for (int i = 0; i < ndim; i++)
        y[i] = y[i] + h6 * (k1[i] + 2.0 * k2[i] + 2.0 * k3[i] + k4[i]);

    return err;
}

void ABM::setBootstrapError(const double* y, double h)
{
    auto& wrk = this->getWorkspace();
    int ndim = wrk.dimension_;
    double hAbs = std::abs(h);

    double yScale = 1.0;
    for (int i = 0; i < ndim; i++)
        yScale = std::max(yScale, std::abs(y[i]));

    double errEst = yScale * hAbs * 1e-7;
    for (int i = 0; i < ndim; i++)
        wrk.absErrPerLen_[i] = errEst;
}

// ---------------------------------------------------------------------------
// Variable-coefficient ABM  PECE  step
// ---------------------------------------------------------------------------
errc_t ABM::abmStep(ODE& ode, double* y, double t0, double h)
{
    auto& wrk = this->getWorkspace();
    int ndim = wrk.dimension_;
    int p = order_;
    double* ySave = wrk.KArr_[p + 4];
    double* fP    = wrk.KArr_[p + 5];
    double* yPred = wrk.ymid_;
    errc_t err = eNoError;

    // ---- Build time grids ----
    // History is a circular buffer.  The most recent entry (f_n) sits at
    // index  (histIdx_ - 1 + p) % p,  the one before at  (histIdx_ - 2 + p) % p, etc.
    // tHist_ uses the same indexing.
    //
    // Predictor: points  t_n, t_{n-1}, …, t_{n-p+1}
    double tauAB[6];
    for (int j = 0; j < p; j++)
        tauAB[j] = tHist_[(histIdx_ - 1 - j + p) % p];

    // Corrector: points  t_{n+1} (predicted), t_n, …, t_{n-p+2}
    double tauAM[6];
    tauAM[0] = t0 + h;                                   // predicted point
    for (int j = 1; j < p; j++)
        tauAM[j] = tauAB[j - 1];                          // t_n, t_{n-1}, …, t_{n-p+2}

    // ---- Compute variable-step integration weights ----
    double wAB[6], wAM[6];
    computeWeights(wAB, tauAB, p, t0, h);
    computeWeights(wAM, tauAM, p, t0, h);

    // ---- Save y_n ----
    for (int i = 0; i < ndim; i++)
        ySave[i] = y[i];

    // ---- Predictor ----
    for (int i = 0; i < ndim; i++)
    {
        double sum = wAB[0] * wrk.KArr_[(histIdx_ - 1 + p) % p][i];  // f_n
        for (int j = 1; j < p; j++)
        {
            int idx = (histIdx_ - 1 - j + p) % p;
            sum += wAB[j] * wrk.KArr_[idx][i];                       // f_{n-j}
        }
        y[i] = ySave[i] + h * sum;
    }

    // Save y^P for error estimation
    for (int i = 0; i < ndim; i++)
        yPred[i] = y[i];

    // Evaluate f^P
    err = ode.evaluate(t0 + h, y, fP);

    // ---- Corrector ----
    for (int i = 0; i < ndim; i++)
    {
        double sum = wAM[0] * fP[i];                                  // f^P
        sum += wAM[1] * wrk.KArr_[(histIdx_ - 1 + p) % p][i];        // f_n
        for (int j = 2; j < p; j++)
        {
            int idx = (histIdx_ - j + p) % p;                         // f_{n-j+1}
            sum += wAM[j] * wrk.KArr_[idx][i];
        }
        y[i] = ySave[i] + h * sum;
    }

    // ---- Error estimate (Milne's device) ----
    for (int i = 0; i < ndim; i++)
        wrk.absErrPerLen_[i] = y[i] - yPred[i];

    return err;
}

// ---------------------------------------------------------------------------
// singleStep  —  dispatches between bootstrap and ABM
// ---------------------------------------------------------------------------
errc_t ABM::singleStep(ODE& ode, double* y, double t0, double h)
{
    auto& wrk = this->getWorkspace();
    errc_t err = eNoError;

    if (histCount_ < order_)
    {
        if (histCount_ == 0)
            initStepSize_ = std::abs(h);

        double hAbs = std::abs(h);
        double maxH = 2.0 * initStepSize_;
        if (hAbs > maxH * 1.01)
        {
            for (int i = 0; i < wrk.dimension_; i++)
                wrk.absErrPerLen_[i] = 1e100;
            return eNoError;
        }

        err = bootstrapStep(ode, y, t0, h);
        setBootstrapError(y, h);
    }
    else
    {
        err = abmStep(ode, y, t0, h);
    }

    // Store derivative and time in history
    double tnew = t0 + h;
    err |= ode.evaluate(tnew, y, wrk.KArr_[histIdx_]);
    tHist_[histIdx_] = tnew;
    histIdx_ = (histIdx_ + 1) % order_;
    if (histCount_ < order_)
        histCount_++;

    return err;
}

SOLVER_NS_END
