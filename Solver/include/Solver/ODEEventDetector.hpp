///
/// @file      ODEEventDetector.hpp
/// @brief     Event detector for ODE integration.

#pragma once

#include "Solver/Config.hpp"
#include <utility>

SOLVER_NS_BEGIN

class ODEEventDetector
{
public:
    enum EDirection
    {
        eDecrease = -1,
        eBoth = 0,
        eIncrease = 1,
    };

    ODEEventDetector() = default;
    virtual ~ODEEventDetector() = default;

    virtual double getDifference(const double* y, double x) const { return getValue(y, x) - goal_; }

    virtual double getValue(const double* y, double x) const = 0;

    int getRepeatCount() const { return repeatCount_; }
    void setRepeatCount(int repeatCount) { repeatCount_ = repeatCount; }

    EDirection getDirection() const { return direction_; }
    void setDirection(EDirection direction) { direction_ = direction; }

    double getThreshold() const { return threshold_; }
    void setThreshold(double threshold) { threshold_ = threshold; }

    double getGoal() const { return goal_; }
    void setGoal(double goal) { goal_ = goal; }

private:
    int repeatCount_{1};
    EDirection direction_{eBoth};
    double threshold_{1e-10};
    double goal_{0.0};
};

template <typename Func>
class ODEEventDetectorGeneric : public ODEEventDetector
{
public:
    explicit ODEEventDetectorGeneric(Func func) : func_(std::move(func)) {}

    double getValue(const double* y, double x) const override { return func_(y, x); }

private:
    Func func_;
};

SOLVER_NS_END
