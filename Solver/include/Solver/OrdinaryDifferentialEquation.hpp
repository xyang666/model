///
/// @file      OrdinaryDifferentialEquation.hpp
/// @brief     ODE base interface.
/// @details   Defines the interface for ordinary differential equations. Inspired by hipparchus-ode.
/// @ingroup   ODE

#pragma once

#include "Solver/Config.hpp"
#include <utility>

SOLVER_NS_BEGIN

class OrdinaryDifferentialEquation
{
public:
    virtual ~OrdinaryDifferentialEquation() = default;

    virtual int getDimension() const = 0;

    /// @brief Evaluate the ODE: dy/dt = f(y, t).
    /// @param y    State vector (input).
    /// @param dy   Derivative output.
    /// @param t    Current time.
    virtual errc_t evaluate(const double* y, double* dy, double t) = 0;

    SOLVER_INLINE
    errc_t evaluate(double t, const double* y, double* dy) { return evaluate(y, dy, t); }
};

using ODE = OrdinaryDifferentialEquation;

/// @brief Generic ODE wrapping any callable (lambda, function pointer, functor).
template <typename Func>
class ODEGeneric : public ODE
{
public:
    ODEGeneric(Func func, int dim)
        : func_(std::move(func)), dim_(dim) {}

    int getDimension() const override { return dim_; }

    errc_t evaluate(const double* y, double* dy, double t) override
    {
        return func_(y, dy, t);
    }

private:
    Func func_;
    int dim_;
};

/// @brief Create a generic ODE from a callable.
template <typename Func>
ODEGeneric<Func> make_ode(Func func, int dim)
{
    return ODEGeneric<Func>(func, dim);
}

template <typename Func>
ODEGeneric<Func> make_ode(int dim, Func func)
{
    return ODEGeneric<Func>(func, dim);
}

SOLVER_NS_END
