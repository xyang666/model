#include "aircraft_model.h"
#include <iostream>
#include <iomanip>

int main()
{
    AircraftModel aircraft;

    // Initial state: [x, y, z, V, pitch, yaw]
    Vector6d initial_state;
    initial_state << 0.0, 0.0, 1000.0, 300, 3.14 / 10, 0.0;

    // Control input: [Nx, Nz, roll] (tangential overload, normal overload, roll angle)
    Vector3d control;
    control << 0.0, 0.0, 0.0;

    // Simulation parameters
    double t_end = 10.0; // seconds
    double dt = 0.01;    // time step

    // Run simulation
    auto trajectory = aircraft.simulate(t_end, dt, initial_state, control);

    // Output results (first few points)
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Time\tx\t\ty\t\tz\t\tV\t\tpitch\t\tyaw\n";
    for (size_t i = 0; i < trajectory.size(); i += 100)
    { // Every 1 second
        double t = i * dt;
        auto state = trajectory[i];
        std::cout << t << "\t" << state(0) << "\t" << state(1) << "\t" << state(2) << "\t" << state(3) << "\t" << state(4) << "\t" << state(5) << "\n";
    }

    return 0;
}
