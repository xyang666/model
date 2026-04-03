#ifndef AIRCRAFT_MODEL_H
#define AIRCRAFT_MODEL_H

#include <vector>
#include <Eigen/Core>

typedef Eigen::Matrix<double, 6, 1> Vector6d;
typedef Eigen::Matrix<double, 3, 1> Vector3d;

// 3DOF Aircraft Model
// State vector: [x, y ,z , V, pitch, yaw] (空间位置/航迹倾角/航迹方位角)
// Control inputs: [Nx， Nz, roll] (切向过载/法向过载/滚转角)

class AircraftModel
{
public:
    AircraftModel();
    ~AircraftModel() = default;

    // Parameters
    double m;    // mass
    double g;    // gravity
    
    // Dynamics function
    Vector6d dynamics(const Vector6d &state, const Vector3d &control);

    // Runge-Kutta 4th order integration
    Vector6d rk4_step(const Vector6d &state, const Vector3d &control, double dt);

    // Simulate for a given time
    std::vector<Vector6d> simulate(double t_end, double dt, const Vector6d &initial_state, const Vector3d &control);
};

#endif // AIRCRAFT_MODEL_H