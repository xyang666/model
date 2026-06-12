///
/// @file      ODEIntegrator.hpp
/// @brief     ODE integrator base classes.

#pragma once

#include "Solver/Config.hpp"
#include "Solver/StateObserver/ODEEventDetectorList.hpp"
#include "Solver/StateObserver/ODEStateObserverList.hpp"
#include "Solver/StateObserver/ODEInnerStateObserver.hpp"
#include "Solver/OrdinaryDifferentialEquation.hpp"
#include <vector>
#include <type_traits>

SOLVER_NS_BEGIN

class ODEEventDetector;

class IODEIntegrator
{
public:
    virtual ~IODEIntegrator() {};

    virtual errc_t initialize(ODE& ode) = 0;

    virtual errc_t integrate(ODE& ode, double* y, double& t, double tf) = 0;

    virtual errc_t integrateStep(ODE& ode, double* y, double& t, double tf) = 0;

    virtual errc_t singleStep(ODE& ode, double* y, double t0, double step) = 0;
};

class ODEIntegrator : public IODEIntegrator
{
public:
    ODEIntegrator() = default;
    ~ODEIntegrator() override;

    using IODEIntegrator::integrate;

    errc_t initialize(ODE& ode) override;

    errc_t integrate(
        ODE& ode, double* y, double& t, double tf,
        std::vector<double>& xlist, std::vector<std::vector<double>>& ylist
    );

    template <typename Func>
    errc_t integrate(int ndim, Func func, double* y, double& t, double tf)
    {
        auto ode = make_ode(ndim, func);
        return integrate(ode, y, t, tf);
    }

    void addEventDetector(ODEEventDetector* detector);

    template <typename Func>
    typename std::enable_if<!std::is_base_of<ODEEventDetector, typename std::remove_pointer<Func>::type>::value, ODEEventDetector*>::type
    addEventDetector(Func func) {
        ODEEventDetector* detector = new ODEEventDetectorGeneric<Func>(std::move(func));
        addEventDetector(detector);
        return detector;
    }

    void removeEventDetector(ODEEventDetector* detector);

    void addStateObserver(ODEStateObserver* observer);

    template <typename Func>
    typename std::enable_if<!std::is_base_of<ODEStateObserver, typename std::remove_pointer<Func>::type>::value, ODEStateObserver*>::type
    addStateObserver(Func func) {
        auto observer = new ODEStateObserverGeneric<Func>(func);
        addStateObserver(observer);
        return observer;
    }

    void removeStateObserver(ODEStateObserver* observer);

    ODE* getODE() { return ode_; }

    double* stateAtStepStart() { return stateAtStepStart_; }
    double* stateAtStepEnd() { return stateAtStepEnd_; }
    double& timeAtStepStart() { return timeAtStepStart_; }
    double& timeAtStepEnd() { return timeAtStepEnd_; }
    double* stateTemp() { return stateTemp_; }

protected:
    friend class ODEInnerStateObserver;
    void initWorkStateObserver();
    SOLVER_NO_COPY(ODEIntegrator);

protected:
    ODE* ode_{nullptr};
    ODEStateObserver* workStateObserver_{nullptr};
    ODEEventDetectorList eventDetectorList_;
    ODEStateObserverList stateObserverList_;
    ODEInnerStateObserver* innerStateObserver_{nullptr};
    double* stateAtStepStart_{nullptr};
    double* stateAtStepEnd_{nullptr};
    double* stateTemp_{nullptr};
    double timeAtStepStart_{0.0};
    double timeAtStepEnd_{0.0};
};

SOLVER_NS_END
