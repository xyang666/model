///
/// @example   01-harmonic-oscillator.cpp
/// @brief     Solve a harmonic oscillator (y'' = -k*y) using RK4 fixed-step integrator.
/// @details   This example demonstrates:
///            1. Defining an ODE system via a lambda
///            2. Configuring a fixed-step integrator (RK4)
///            3. Collecting state history with ODEStateVectorCollector
///            4. Printing the solution

#include <Solver/Solver.hpp>
#include <cstdio>
#include <cmath>

using namespace Solver;

int main()
{
    // Harmonic oscillator: y'' = -k * y
    // Converted to first-order system:
    //   y[0]' = y[1]   (velocity)
    //   y[1]' = -k * y[0]  (acceleration)
    const double k = 1.0;  // spring constant

    auto harmonic = [k](const double* y, double* dy, double t) -> errc_t {
        dy[0] = y[1];
        dy[1] = -k * y[0];
        return eNoError;
    };

    // Initial conditions
    double y[2] = { 1.0, 0.0 };  // position = 1, velocity = 0
    double t = 0.0;
    double tf = 10.0;

    // Create RK4 integrator with step size
    RK4 integrator;
    integrator.setStepSize(0.01);

    // Collect state history
    auto ode = make_ode(harmonic, 2);
    std::vector<double> tHistory;
    std::vector<std::vector<double>> yHistory;
    integrator.integrate(ode, y, t, tf, tHistory, yHistory);

    // Print results
    std::printf("Harmonic Oscillator (k=%.1f) solved with RK4\n", k);
    std::printf("  Steps: %d\n", integrator.getNumSteps());
    std::printf("  Time | Position | Velocity\n");
    std::printf("  -----|----------|---------\n");

    int stride = tHistory.size() / 10;
    if (stride < 1) stride = 1;
    for (size_t i = 0; i < tHistory.size(); i += stride)
    {
        std::printf("  %5.2f | %8.4f | %8.4f\n",
            tHistory[i], yHistory[i][0], yHistory[i][1]);
    }

    // Verify: analytic solution at t=10 is y[0] = cos(10) ≈ -0.8391
    double analytic = std::cos(10.0);
    std::printf("  -----|----------|---------\n");
    std::printf("  Numeric y(10) = %.6f  |  Analytic cos(10) = %.6f\n", y[0], analytic);
    std::printf("  Error = %.2e\n", std::abs(y[0] - analytic));

    return 0;
}
