///
/// @file      MathOperator.hpp
/// @brief     Math utility functions (sign, clamp, eps, norm).

#pragma once

#include <cmath>
#include <algorithm>
#include <limits>

SOLVER_NS_BEGIN

namespace math
{

/// @brief Signum function: returns 1 for positive, -1 for negative, 0 for zero.
inline int sign(double x)
{
    return (x > 0) ? 1 : ((x < 0) ? -1 : 0);
}

/// @brief Clamp value between low and high.
inline double clamp(double value, double low, double high)
{
    return std::max(low, std::min(high, value));
}

/// @brief Machine epsilon scaled for the magnitude of x.
inline double eps(double x)
{
    return std::numeric_limits<double>::epsilon() * std::max(std::abs(x), 1.0);
}

/// @brief L2 (Euclidean) norm.
inline double norm(const double* v, int n)
{
    double sum = 0.0;
    for (int i = 0; i < n; ++i)
        sum += v[i] * v[i];
    return std::sqrt(sum);
}

} // namespace math

SOLVER_NS_END
