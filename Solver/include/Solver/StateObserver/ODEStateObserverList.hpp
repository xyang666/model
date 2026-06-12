///
/// @file      ODEStateObserverList.hpp
/// @brief     Composite of state observers.

#pragma once

#include "Solver/Config.hpp"
#include "Solver/ODEStateObserver.hpp"
#include <vector>

SOLVER_NS_BEGIN

class ODEStateObserverList : public ODEStateObserver
{
public:
    ODEStateObserverList() = default;
    ~ODEStateObserverList() override;

    EODEAction onStateUpdate(double* y, double& x, ODEIntegrator* integrator) override;

    void addStateObserver(ODEStateObserver* observer) { observers_.push_back(observer); }
    void removeStateObserver(ODEStateObserver* observer);

    bool empty() const { return observers_.empty(); }
    size_t size() const { return observers_.size(); }

    ODEStateObserver& operator[](size_t index) { return *observers_[index]; }
    const ODEStateObserver& operator[](size_t index) const { return *observers_[index]; }

protected:
    std::vector<ODEStateObserver*> observers_;
};

SOLVER_NS_END
