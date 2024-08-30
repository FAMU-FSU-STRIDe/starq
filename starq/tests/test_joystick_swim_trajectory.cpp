#include <stdio.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "starq/starq/starq_robot.hpp"

using namespace starq;

class JoystickTrajectoryNode : public rclcpp::Node
{
public:
    JoystickTrajectoryNode() : Node("joystick_trajectory")
    {
        STARQ_ = std::make_shared<STARQRobot>();

        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "joy", 10, std::bind(&JoystickTrajectoryNode::joyCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Joystick Initialized.");
    }

    ~JoystickTrajectoryNode()
    {
        STARQ_->setStates(AxisState::IDLE);
    }

private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (msg->buttons[0] == 1)
        {
            // X
            STARQ_->setStates(AxisState::CLOSED_LOOP_CONTROL);
            RCLCPP_INFO(this->get_logger(), "Switching to Closed Loop Control");
        }
        else if (msg->buttons[1] == 1)
        {
            // A
            STARQ_->setStates(AxisState::IDLE);
            RCLCPP_INFO(this->get_logger(), "Switching to Idle");
        }
        else if (msg->buttons[4] == 1)
        {
            // LB
            phi_ -= phi_res_;
            RCLCPP_INFO(this->get_logger(), "Phi decreased to %.4f", phi_);
        }
        else if (msg->buttons[5] == 1)
        {
            // RB
            phi_ += phi_res_;
            RCLCPP_INFO(this->get_logger(), "Phi increased to %.4f", phi_);
        }

        const int8_t left_axis_x = msg->axes[1] * 128;
        const Float freq = max_freq_ * left_axis_x / 128.0;

        if (!STARQ_->getTrajectoryController()->isRunning() && left_axis_x > 0)
        {
            Trajectory trajectory = generateTrajectory(freq);
            STARQ_->runTrajectory(trajectory);
        }
        else if (!STARQ_->getTrajectoryController()->isRunning())
        {
            const Float x0 = L0_ * std::cos(phi_);
            const Float y0 = L0_ * std::sin(phi_);
            for (std::size_t i = 0; i < STARQ_->getLegs().size(); i++)
                STARQ_->setFootPosition(i, Vector3(x0, 0.0, y0));
        }
    }

    Trajectory generateTrajectory(const Float freq)
    {
        Trajectory trajectory;
        trajectory.reserve(num_points_);

        const Float x0 = L0_ * std::cos(phi_);
        const Float y0 = L0_ * std::sin(phi_);

        const Float theta = phi_ + M_PI / 2;
        Eigen::Matrix2<Float> R;
        R << std::cos(theta), -std::sin(theta), std::sin(theta), std::cos(theta);

        for (std::size_t t = 0; t < num_points_; t++)
        {
            const Float x = xamp_ * std::cos(2 * M_PI * t / num_points_);
            const Float y = yamp_ * std::sin(2 * M_PI * t / num_points_);
            const Eigen::Vector2<Float> p = Eigen::Vector2<Float>(x0, y0) + R * Eigen::Vector2<Float>(x, y);

            if (p.norm() > maxr_)
            {
                RCLCPP_ERROR(this->get_logger(), "Trajectory point %lu is out of bounds (outer)", t);
                return {};
            }
            else if (p.norm() < minr_)
            {
                RCLCPP_ERROR(this->get_logger(), "Trajectory point %lu is out of bounds (inner)", t);
                return {};
            }

            for (std::size_t i = 0; i < STARQ_->getLegs().size(); i++)
            {
                LegCommand::Ptr command = std::make_shared<LegCommand>();
                command->delay = std::chrono::milliseconds(time_t(1000 * t / freq));
                command->leg_id = i;
                command->control_mode = ControlMode::POSITION;
                command->target_position = Vector3(p.x(), 0.0, p.y());
                trajectory.push_back(command);
            }
        }

        return trajectory;
    }

    STARQRobot::Ptr STARQ_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    std::size_t num_points_ = 100;
    Float max_freq_ = 2.5;
    Float L0_ = 0.18;
    Float phi_ = -3 * M_PI / 4;
    Float phi_res_ = M_PI / 16;
    Float xamp_ = 0.02;
    Float yamp_ = 0.005;
    Float maxr_ = 0.2;
    Float minr_ = 0.1;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<JoystickTrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}
