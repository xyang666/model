///
/// @file      ODEEventDetectorList.hpp
/// @brief     Composite of event observers.

#pragma once

#include "Solver/Config.hpp"
#include "Solver/StateObserver/ODEEventObserver.hpp"
#include <vector>

SOLVER_NS_BEGIN

class ODEEventObserver;

class ODEEventDetectorList : public ODEStateObserver
{
public:
    ~ODEEventDetectorList() override;
    ODEEventDetectorList() = default;

    EODEAction onStateUpdate(double* y, double& x, ODEIntegrator* integrator) final;

    void addEventDetector(ODEEventDetector* detector);
    void removeEventDetector(ODEEventDetector* detector);

    bool empty() const { return eventObservers_.empty(); }
    size_t size() const { return eventObservers_.size(); }

    ODEEventObserver& operator[](size_t index) { return *eventObservers_[index]; }
    const ODEEventObserver& operator[](size_t index) const { return *eventObservers_[index]; }

protected:
    std::vector<ODEEventObserver*> eventObservers_;
};

SOLVER_NS_END
