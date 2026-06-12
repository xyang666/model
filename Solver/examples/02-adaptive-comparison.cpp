///
/// @example   02-adaptive-comparison.cpp
/// @brief     Compare adaptive integrators (RKF45, RKF56, RKF78, RKCK, ABM) on a stiff-ish ODE.
/// @details   Solves the Van der Pol oscillator and compares step count, accuracy, and elapsed time.
///            ABM uses predictor-corrector error estimation (Milne's device) for step control.

#include <Solver/Solver.hpp>
#include <cstdio>
#include <cmath>
#include <chrono>

using namespace Solver;

int main()
{
    // Van der Pol oscillator: y'' - mu*(1-y^2)*y' + y = 0
    // First-order system:
    //   y[0]' = y[1]
    //   y[1]' = mu*(1 - y[0]^2)*y[1] - y[0]
    const double mu = 2.0;

    auto vdp = [mu](const double* y, double* dy, double t) -> errc_t {
        dy[0] = y[1];
        dy[1] = mu * (1.0 - y[0] * y[0]) * y[1] - y[0];
        return eNoError;
    };

    struct Result {
        const char* name;
        int    steps;
        double y0_end;
        double y1_end;
        double elapsed_ms;
    };

    Result results[5];

    auto run = [&](int idx, const char* name, ODEVarStepIntegrator* integrator) {
        double y[2] = { 2.0, 0.0 };
        double t = 0.0;
        double tf = 20.0;

        integrator->setInitialStepSize(0.1);
        integrator->setMaxAbsErr(1e-8);
        integrator->setMaxRelErr(1e-8);

        auto ode = make_ode(vdp, 2);

        auto t1 = std::chrono::high_resolution_clock::now();
        integrator->integrate(ode, y, t, tf);
        auto t2 = std::chrono::high_resolution_clock::now();

        double elapsed = std::chrono::duration<double, std::milli>(t2 - t1).count();
        results[idx] = { name, integrator->getNumSteps(), y[0], y[1], elapsed };
    };

    {
        { RKF45 i; run(0, "RKF45", &i); }
        { RKF56 i; run(1, "RKF56", &i); }
        { RKF78 i; run(2, "RKF78", &i); }
        { RKCK  i; run(3, "RKCK",  &i); }
        { ABM   i(4); run(4, "ABM4", &i); }
    }

    std::printf("Van der Pol oscillator (mu=%.1f) - Integrator Comparison\n", mu);
    std::printf("  Integrator | Steps | y[0](20) | y[1](20) |  Time (ms)\n");
    std::printf("  -----------|-------|-----------|-----------|-----------\n");
    for (int i = 0; i < 5; i++)
    {
        std::printf("  %-10s | %5d | %9.6f | %9.6f | %10.3f\n",
            results[i].name, results[i].steps, results[i].y0_end, results[i].y1_end,
            results[i].elapsed_ms);
    }

    std::printf("\n  ABM4 is a variable-step, variable-coefficient predictor-corrector\n");
    std::printf("  method. Integration weights are computed from the actual time grid\n");
    std::printf("  each step.  Error estimated via Milne's device (|y^C - y^P|).\n");

    return 0;
}
