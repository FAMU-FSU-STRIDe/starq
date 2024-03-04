#include "starq/dynamics/unitree_a1_robot.hpp"

namespace starq::dynamics
{

    UnitreeA1RobotDynamics::UnitreeA1RobotDynamics()
    {
        body_mass_ = UNITREE_A1_MASS;
        body_inertia_ << UNITREE_A1_INERTIA;
        gravity_ << UNITREE_A1_GRAVITY;
        body_height_ = UNITREE_A1_HEIGHT;
        foot_friction_ = UNITREE_A1_FOOT_FRICTION;
        hip_locations_ = {Vector3f(UNITREE_A1_HIP_OFFSET_X, UNITREE_A1_HIP_OFFSET_Y, 0.0),
                          Vector3f(-UNITREE_A1_HIP_OFFSET_X, UNITREE_A1_HIP_OFFSET_Y, 0.0),
                          Vector3f(-UNITREE_A1_HIP_OFFSET_X, -UNITREE_A1_HIP_OFFSET_Y, 0.0),
                          Vector3f(UNITREE_A1_HIP_OFFSET_X, -UNITREE_A1_HIP_OFFSET_Y, 0.0)};
        force_z_min_ = 10;
        force_z_max_ = 250;
    }

    UnitreeA1RobotDynamics::~UnitreeA1RobotDynamics()
    {
    }

    float UnitreeA1RobotDynamics::getBodyMass() const
    {
        return body_mass_;
    }

    Matrix3f UnitreeA1RobotDynamics::getBodyInertia() const
    {
        return body_inertia_;
    }

    Vector3f UnitreeA1RobotDynamics::getGravity() const
    {
        return gravity_;
    }

    float UnitreeA1RobotDynamics::getBodyHeight() const
    {
        return body_height_;
    }

    float UnitreeA1RobotDynamics::getFootFriction() const
    {
        return foot_friction_;
    }

    std::vector<Vector3f> UnitreeA1RobotDynamics::getHipLocations() const
    {
        return hip_locations_;
    }

    float UnitreeA1RobotDynamics::getForceZMin() const
    {
        return force_z_min_;
    }

    float UnitreeA1RobotDynamics::getForceZMax() const
    {
        return force_z_max_;
    }

}