#include "starq/starq/starq_robot.hpp"

#include <iostream>

namespace starq
{

    STARQRobot::STARQRobot()
    {
        setup();
    }

    bool STARQRobot::setState(AxisState state)
    {
        for (size_t i = 0; i < motors_.size(); i++)
        {
            if (!motors_[i]->setState(state))
            {
                std::cout << "Failed to set axis state for motor " << i << "." << std::endl;
                return false;
            }
        }
        return true;
    }

    void STARQRobot::setupMotors()
    {

        using namespace can;
        can_socket_0_ = std::make_shared<CANSocket>("can0");
        can_socket_1_ = std::make_shared<CANSocket>("can1");

        if (!can_socket_0_->connect())
        {
            std::cout << "Failed to connect to CAN interface 0." << std::endl;
        }

        if (!can_socket_1_->connect())
        {
            std::cout << "Failed to connect to CAN interface 1." << std::endl;
        }

        using namespace odrive;
        ODriveSocket::Ptr odrive_socket_0 = std::make_shared<ODriveSocket>(can_socket_0_);
        ODriveSocket::Ptr odrive_socket_1 = std::make_shared<ODriveSocket>(can_socket_1_);

        ODriveController::Ptr odrive_0 = std::make_shared<ODriveController>(odrive_socket_0, 0);
        ODriveController::Ptr odrive_1 = std::make_shared<ODriveController>(odrive_socket_0, 1);
        ODriveController::Ptr odrive_2 = std::make_shared<ODriveController>(odrive_socket_0, 2);
        ODriveController::Ptr odrive_3 = std::make_shared<ODriveController>(odrive_socket_0, 3);
        ODriveController::Ptr odrive_4 = std::make_shared<ODriveController>(odrive_socket_1, 4);
        ODriveController::Ptr odrive_5 = std::make_shared<ODriveController>(odrive_socket_1, 5);
        ODriveController::Ptr odrive_6 = std::make_shared<ODriveController>(odrive_socket_1, 6);
        ODriveController::Ptr odrive_7 = std::make_shared<ODriveController>(odrive_socket_1, 7);

        Float GR = 6.0;
        odrive_0->setGearRatio(GR);
        odrive_1->setGearRatio(GR);
        odrive_2->setGearRatio(GR);
        odrive_3->setGearRatio(GR);
        odrive_4->setGearRatio(GR);
        odrive_5->setGearRatio(GR);
        odrive_6->setGearRatio(GR);
        odrive_7->setGearRatio(GR);

        motors_ = {
            odrive_0,
            odrive_1,
            odrive_2,
            odrive_3,
            odrive_4,
            odrive_5,
            odrive_6,
            odrive_7};
    }

    void STARQRobot::setupLegs()
    {
        Float L1 = 0.05;
        Float L2 = 0.150;
        leg_dynamics_ = std::make_shared<STARQFiveBar2DLegDynamics>(L1, L2);

        LegController::Ptr leg_FL = std::make_shared<LegController>(leg_dynamics_,
                                                                    MotorList{motors_[0], motors_[1]});

        LegController::Ptr leg_RL = std::make_shared<LegController>(leg_dynamics_,
                                                                    MotorList{motors_[2], motors_[3]});

        LegController::Ptr leg_RR = std::make_shared<LegController>(leg_dynamics_,
                                                                    MotorList{motors_[4], motors_[5]});

        LegController::Ptr leg_FR = std::make_shared<LegController>(leg_dynamics_,
                                                                    MotorList{motors_[6], motors_[7]});

        legs_ = {leg_FL, leg_RL, leg_RR, leg_FR};
    }

    void STARQRobot::setupLocalization()
    {
        localization_ = std::make_shared<SystemLocalization>();
    }

    void STARQRobot::setupRobotDynamics()
    {
        robot_parameters_ = nullptr;
    }

    void STARQRobot::setupMPCSolver()
    {
        mpc_solver_ = nullptr;
    }

}