///
/// @example   03-event-detection.cpp
/// @brief     Demonstrate ODE integration with event detection.
/// @details   Solves a pendulum and uses event detectors to catch precise
///            zero-crossings (theta = 0), plus a state observer for monitoring.

#include <Solver/Solver.hpp>
#include <cstdio>
#include <cmath>

using namespace Solver;

int main()
{
    // Simple pendulum: theta'' = -(g/L) * sin(theta)
    const double g_over_L = 9.81;

    auto pendulum = [g_over_L](const double* y, double* dy, double t) -> errc_t {
        dy[0] = y[1];
        dy[1] = -g_over_L * std::sin(y[0]);
        return eNoError;
    };

    // ---- Step observer ----
    int stepCount = 0;
    auto stepMonitor = [&stepCount](double* y, double& x, ODEIntegrator* integrator) -> EODEAction {
        stepCount++;
        std::printf("  Step %3d: t=%.6f, theta=%.6f, omega=%.3f\n",
            stepCount, x, y[0], y[1]);
        return EODEAction::eContinue;
    };

    // ---- Event detector: theta crosses zero in increasing direction ----
    auto thetaCrossing = [](const double* y, double x) -> double {
        return y[0];
    };

    // ---- State observer that reports detected events ----
    // Each crossing from negative to positive marks a full period.
    int eventCount = 0;
    double lastTheta = 0.1;
    double prevCrossTime = 0.0;
    auto eventReporter = [&eventCount, &lastTheta, &prevCrossTime]
                         (double* y, double& x, ODEIntegrator* integrator) -> EODEAction {
        bool droppedToZero = (std::abs(y[0]) < 1e-12 && std::abs(lastTheta) > 1e-6);
        bool signFlip = (lastTheta > 0.0 && y[0] < 0.0)
                     || (lastTheta < 0.0 && y[0] > 0.0);
        if (droppedToZero || signFlip) {
            eventCount++;
            const char* direction = (y[1] > 0) ? "(-) -> (+)" : "(+) -> (-)";
            double dt = prevCrossTime > 0 ? x - prevCrossTime : 0;
            std::printf("  >>> Event %d: theta crossed zero at t=%.10f, omega=%.6f  %s",
                eventCount, x, y[1], direction);
            if (dt > 0)
                std::printf("  dt=%.6f", dt);
            std::printf("\n");
            prevCrossTime = x;
        }
        lastTheta = y[0];
        return EODEAction::eContinue;
    };

    double y[2] = { 0.1, 0.0 };  // small amplitude so sin(θ) ≈ θ
    double t = 0.0;
    double tf = 2.0;

    RKF78 integrator;
    integrator.setInitialStepSize(0.01);
    integrator.setMaxAbsErr(1e-12);
    integrator.setMaxRelErr(1e-12);

    integrator.addStateObserver(stepMonitor);

    ODEEventDetector* detector = integrator.addEventDetector(thetaCrossing);
    detector->setDirection(ODEEventDetector::eIncrease);
    detector->setRepeatCount(10);  // allow multiple events, don't stop early
    integrator.addStateObserver(eventReporter);

    auto ode = make_ode(pendulum, 2);
    integrator.integrate(ode, y, t, tf);

    double T_small = 2.0 * M_PI / std::sqrt(g_over_L);
    std::printf("\nPendulum with Event Detection\n");
    std::printf("  Expected period ~%.4f s (small-angle approximation)\n", T_small);
    std::printf("  Total events detected: %d\n", eventCount);
    std::printf("  Final state at t=%.6f: theta=%.10f, omega=%.6f\n", t, y[0], y[1]);
    std::printf("  Total steps: %d\n", integrator.getNumSteps());

    return 0;
}
