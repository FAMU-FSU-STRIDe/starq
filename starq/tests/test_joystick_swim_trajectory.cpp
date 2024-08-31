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
        if (msg->buttons[0] == 1 && last_msg_.buttons[0] == 0)
        {
            // 1
            STARQ_->setStates(AxisState::CLOSED_LOOP_CONTROL);
            RCLCPP_INFO(this->get_logger(), "Switching to Closed Loop Control");
        }
        else if (msg->buttons[1] == 1 && last_msg_.buttons[1] == 0)
        {
            // 2
            STARQ_->setStates(AxisState::IDLE);
            RCLCPP_INFO(this->get_logger(), "Switching to Idle");
        }
        else if (msg->buttons[2] == 1 && last_msg_.buttons[2] == 0)
        {
            // 3
            static_ = !static_;
            RCLCPP_INFO(this->get_logger(), "Switching to %s", static_ ? "Static" : "Moving");
        }
        else if (msg->buttons[4] == 1 && last_msg_.buttons[4] == 0)
        {
            // LB
            phi_ -= phi_res_;
            RCLCPP_INFO(this->get_logger(), "Phi decreased to %.4f", phi_);
        }
        else if (msg->buttons[5] == 1 && last_msg_.buttons[5] == 0)
        {
            // RB
            phi_ += phi_res_;
            RCLCPP_INFO(this->get_logger(), "Phi increased to %.4f", phi_);
        }
        else if (msg->buttons[6] == 1 && last_msg_.buttons[6] == 0)
        {
            // LT
            freq_ -= freq_res_;
            freq_ = std::max(freq_, freq_res_);
            RCLCPP_INFO(this->get_logger(), "Frequency decreased to %.4f", freq_);
        }
        else if (msg->buttons[7] == 1 && last_msg_.buttons[7] == 0)
        {
            // RT
            freq_ += freq_res_;
            RCLCPP_INFO(this->get_logger(), "Frequency increased to %.4f", freq_);
        }
        else if (msg->axes[4] >= 0.5 && last_msg_.axes[4] < 0.5)
        {
            // Cross left
            yamp_ -= yamp_res_;
            yamp_ = std::max(yamp_, 0.0);
            RCLCPP_INFO(this->get_logger(), "Y Amplitude decreased to %.4f", yamp_);
        }
        else if (msg->axes[4] <= -0.5 && last_msg_.axes[4] > -0.5)
        {
            // Cross right
            yamp_ += yamp_res_;
            RCLCPP_INFO(this->get_logger(), "Y Amplitude increased to %.4f", yamp_);
        }
        else if (msg->axes[5] >= 0.5 && last_msg_.axes[5] < 0.5)
        {
            // Cross up
            xamp_ += xamp_res_;
            RCLCPP_INFO(this->get_logger(), "X Amplitude increased to %.4f", xamp_);
        }
        else if (msg->axes[5] <= -0.5 && last_msg_.axes[5] > -0.5)
        {
            // Cross down
            xamp_ -= xamp_res_;
            xamp_ = std::max(xamp_, 0.0);
            RCLCPP_INFO(this->get_logger(), "X Amplitude decreased to %.4f", xamp_);
        }

        if (!STARQ_->getTrajectoryController()->isRunning() && !static_)
        {
            Trajectory trajectory = generateTrajectory();
            STARQ_->runTrajectory(trajectory);
        }
        else if (!STARQ_->getTrajectoryController()->isRunning())
        {
            const Float x0 = L0_ * std::cos(phi_);
            const Float y0 = L0_ * std::sin(phi_);
            for (std::size_t i = 0; i < STARQ_->getLegs().size(); i++)
                STARQ_->setFootPosition(i, Vector3(x0, 0.0, y0));
        }

        last_msg_ = *msg;
    }

    Trajectory generateTrajectory()
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
                command->delay = std::chrono::milliseconds(time_t(1000 * (float(t) / num_points_) / freq_));
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
    sensor_msgs::msg::Joy last_msg_;

    std::size_t num_points_ = 100;
    bool static_ = true;
    Float freq_ = 1.0;
    Float freq_res_ = 0.25;
    Float L0_ = 0.18;
    Float phi_ = -3 * M_PI / 4;
    Float phi_res_ = M_PI / 64;
    Float xamp_ = 0.02;
    Float xamp_res_ = 0.005;
    Float yamp_ = 0.005;
    Float yamp_res_ = 0.001;
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
