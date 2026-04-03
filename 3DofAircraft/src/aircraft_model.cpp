#include "aircraft_model.h"
#include <cmath>

AircraftModel::AircraftModel()
{
    // Initialize parameters (example values for a small aircraft)
    m = 1000.0; // kg
    g = 9.81;   // m/s^2
}

Vector6d AircraftModel::dynamics(const Vector6d &state, const Vector3d &control)
{
    double x = state(0);
    double y = state(1);
    double z = state(2);
    double V = state(3);
    double pitch = state(4);
    double yaw = state(5);

    double Nx = control(0);
    double Nz = control(1);
    double roll = control(2);

    // kinematic equations
    double dx_dt = V * cos(pitch) * cos(yaw);
    double dy_dt = V * cos(pitch) * sin(yaw);
    double dz_dt = V * sin(pitch);

    // dynamic equations
    double dV_dt = Nx * (g - sin(pitch));
    double dpitch_dt = (Nz * cos(roll) - cos(pitch)) * g / V;
    double dyaw_dt = g * Nz * sin(roll) / (V * cos(pitch));

    Vector6d deriv;
    deriv << dx_dt, dy_dt, dz_dt, dV_dt, dpitch_dt, dyaw_dt;
    return deriv;
}

Vector6d AircraftModel::rk4_step(const Vector6d &state, const Vector3d &control, double dt)
{
    Vector6d k1 = dynamics(state, control);
    Vector6d k2 = dynamics(state + 0.5 * dt * k1, control);
    Vector6d k3 = dynamics(state + 0.5 * dt * k2, control);
    Vector6d k4 = dynamics(state + dt * k3, control);

    Vector6d new_state = state + (dt / 6.0) * (k1 + 2 * k2 + 2 * k3 + k4);
    return new_state;
}

std::vector<Vector6d> AircraftModel::simulate(double t_end, double dt, const Vector6d &initial_state, const Vector3d &control)
{
    std::vector<Vector6d> trajectory;
    trajectory.push_back(initial_state);
    Vector6d current_state = initial_state;
    double t = 0.0;
    while (t < t_end)
    {
        current_state = rk4_step(current_state, control, dt);
        trajectory.push_back(current_state);
        t += dt;
    }
    return trajectory;
}