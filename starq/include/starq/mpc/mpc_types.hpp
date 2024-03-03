#ifndef STARQ_MPC__MPC_TYPES_HPP_
#define STARQ_MPC__MPC_TYPES_HPP_

#include <vector>
#include <memory>
#include <chrono>

#include "eigen3/Eigen/Dense"

namespace starq::mpc
{
    using namespace std::chrono;
    using namespace Eigen;

    struct CenterOfMassState
    {
        Vector3f position;
        Vector3f orientation;
        Vector3f linear_velocity;
        Vector3f angular_velocity;
    };
    using CenterOfMassTrajectory = std::vector<CenterOfMassState>;

    using StanceState = std::vector<bool>;
    using StanceTrajectory = std::vector<StanceState>;

    using FootholdState = std::vector<Vector3f>;
    using FootholdTrajectory = std::vector<FootholdState>;

    struct StanceTiming
    {
        milliseconds swing_duration;
        milliseconds stance_duration;
    };
    using StanceTimingTrajectory = std::vector<StanceTiming>;

    struct MPCConfiguration
    {
        float mass;
        Matrix3f inertia;
        Vector3f gravity;
        float height;

        float friction_coeff;
        float fz_min;
        float fz_max;

        milliseconds time_step;
        size_t window_size;

        CenterOfMassTrajectory com_trajectory;
        StanceTrajectory stance_trajectory;
        FootholdTrajectory foothold_trajectory;
        StanceTimingTrajectory timing_trajectory;

        Vector3f position_weights;
        Vector3f orientation_weights;
        Vector3f linear_velocity_weights;
        Vector3f angular_velocity_weights;
        Vector3f force_weights;

    };

    using FootForceState = std::vector<std::pair<bool, Vector3f>>;
    using FootForceTrajectory = std::vector<FootForceState>;

    struct MPCSolution
    {
        int exit_flag;
        microseconds run_time;
        microseconds setup_time;
        microseconds solve_time;

        CenterOfMassTrajectory x_star;
        FootForceTrajectory u_star;
    };

}

#endif