///
/// @file      ODEStateObserver.hpp
/// @brief     State observer pattern for ODE integration.

#pragma once

#include "Solver/Config.hpp"
#include "Solver/TypeTraits.hpp"
#include <utility>

SOLVER_NS_BEGIN

class ODEIntegrator;

enum EODEAction
{
    eContinue = 0,
    eStop = 1,
};

class ODEStateObserver
{
public:
    virtual ~ODEStateObserver() = default;

    virtual EODEAction onStateUpdate(double* y, double& x, ODEIntegrator* integrator) = 0;
};

class ODEStateObserverGenericHelper
{
public:
    template <typename F>
    static SOLVER_INLINE auto call_func(F& func, double* y, double& x, ODEIntegrator* integrator)
        -> typename std::enable_if<is_callable<F, double*, double&, ODEIntegrator*>::value, decltype(std::declval<F>()(y, x, integrator))>::type
    {
        return func(y, x, integrator);
    }

    template <typename F>
    static SOLVER_INLINE auto call_func(F& func, double* y, double& x, ODEIntegrator* integrator)
        -> typename std::enable_if<is_callable<F, double*, double&>::value, decltype(std::declval<F>()(y, x))>::type
    {
        return func(y, x);
    }

    template <typename F>
    static SOLVER_INLINE auto call_func(F& func, double* y, double& x, ODEIntegrator* integrator)
        -> typename std::enable_if<is_callable<F, double*>::value, decltype(std::declval<F>()(y))>::type
    {
        return func(y);
    }
};

template <typename Func>
class ODEStateObserverGeneric : public ODEStateObserver
{
public:
    using Self = ODEStateObserverGeneric<Func>;
    using FuncType = Func;
    explicit ODEStateObserverGeneric(Func func) : func_(std::move(func)) {}

    EODEAction onStateUpdate(double* y, double& x, ODEIntegrator* integrator) override
    {
        return this->operator()(y, x, integrator);
    }

private:
    Func func_;

    template <typename F = FuncType>
    SOLVER_INLINE
    typename std::enable_if<!std::is_void<decltype(
        ODEStateObserverGenericHelper::call_func
        (std::declval<F&>(), std::declval<double*>(), std::declval<double&>(), std::declval<ODEIntegrator*>()))>::value, EODEAction>::type
    operator()(double* y, double& x, ODEIntegrator* integrator) {
        return ODEStateObserverGenericHelper::call_func(func_, y, x, integrator);
    }

    template <typename F = FuncType>
    SOLVER_INLINE
    typename std::enable_if<std::is_void<decltype(
        ODEStateObserverGenericHelper::call_func
        (std::declval<F&>(), std::declval<double*>(), std::declval<double&>(), std::declval<ODEIntegrator*>()))>::value, EODEAction>::type
    operator()(double* y, double& x, ODEIntegrator* integrator) {
        ODEStateObserverGenericHelper::call_func(func_, y, x, integrator);
        return EODEAction::eContinue;
    }
};

SOLVER_NS_END
