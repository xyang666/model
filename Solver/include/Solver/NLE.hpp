///
/// @file      NLE.hpp
/// @brief     Brent's method for solving nonlinear equations (root-finding).
/// @details   Used by ODEEventObserver to precisely locate event times.

#pragma once

#include "Config.hpp"

#include <cmath>
#include <limits>
#include <functional>

SOLVER_NS_BEGIN

/// @brief Brent's method for root-finding (inverse quadratic interpolation / secant / bisection).
class BrentSolver
{
public:
    explicit BrentSolver(double tol = 1e-10)
        : tol_(tol)
    {}

    /// @brief Solve f(x) = 0 in the interval [a, b].
    /// @param f The function whose root is sought.
    /// @param a Lower bound.
    /// @param b Upper bound.
    /// @param root Output — the found root.
    /// @return eNoError on success.
    errc_t solve(std::function<double(double)> f, double a, double b, double& root)
    {
        double fa = f(a);
        double fb = f(b);

        if (std::abs(fa) < tol_) { root = a; return eNoError; }
        if (std::abs(fb) < tol_) { root = b; return eNoError; }

        if (fa * fb > 0)
            return eErrorInvalidArgument;

        double c = a;
        double fc = fa;
        double d = b - a;
        double e = d;

        for (int iter = 0; iter < 100; ++iter)
        {
            if (std::abs(fc) < std::abs(fb))
            {
                a = b;  b = c;  c = a;
                fa = fb; fb = fc; fc = fa;
            }

            double tol = 2.0 * std::numeric_limits<double>::epsilon() * std::abs(b) + 0.5 * tol_;
            double xm = 0.5 * (c - b);

            if (std::abs(xm) <= tol || std::abs(fb) < tol_)
            {
                root = b;
                return eNoError;
            }

            if (std::abs(e) >= tol && std::abs(fa) > std::abs(fb))
            {
                double s = fb / fa;
                double p, q;
                if (a == c)
                {
                    p = 2.0 * xm * s;
                    q = 1.0 - s;
                }
                else
                {
                    q = fa / fc;
                    double r = fb / fc;
                    p = s * (2.0 * xm * q * (q - r) - (b - a) * (r - 1.0));
                    q = (q - 1.0) * (r - 1.0) * (s - 1.0);
                }
                if (p > 0) q = -q; else p = -p;
                double min1 = 3.0 * xm * q - std::abs(tol * q);
                double min2 = std::abs(e * q);
                if (2.0 * p < (min1 < min2 ? min1 : min2))
                {
                    e = d;  d = p / q;
                }
                else
                {
                    d = xm;  e = d;
                }
            }
            else
            {
                d = xm;  e = d;
            }

            a = b;  fa = fb;
            if (std::abs(d) > tol)
                b += d;
            else
                b += (xm > 0 ? tol : -tol);
            fb = f(b);

            // Maintain bracket: ensure c is contra-point (opposite sign from b)
            if (fb * fc > 0.0)
            {
                c = a;  fc = fa;  d = b - a;  e = d;
            }
        }

        root = b;
        return eNoError;
    }

private:
    double tol_;
};

SOLVER_NS_END
