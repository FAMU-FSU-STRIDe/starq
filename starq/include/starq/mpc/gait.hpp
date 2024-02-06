#ifndef STARQ_MPC__GAIT_HPP_
#define STARQ_MPC__GAIT_HPP_

#include <memory>
#include <vector>
#include <string>
#include <map>

#include "starq/mpc/mpc_types.hpp"

#define GAIT_RESOLUTION 1000

namespace starq::mpc
{
    /// @brief Gait class
    class Gait
    {
    public:
        using Ptr = std::shared_ptr<Gait>;

        /// @brief Create a new Gait object
        Gait();

        /// @brief Destroy the Gait object
        ~Gait();

        /// @brief Load the gait pattern from a file
        /// @param file_path The file to load the gait pattern from
        /// @return True if the gait pattern was loaded, false otherwise
        bool load(const std::string &file_path);

        /// @brief Set the frequency of the gait pattern
        /// @param frequency The frequency of the gait pattern
        void setFrequency(const double &frequency);

        /// @brief Set the velocity of the gait pattern
        /// @param linear_velocity The linear velocity of the gait pattern
        void setVelocity(const Vector3f &linear_velocity, const Vector3f &angular_velocity);

        /// @brief Get the duration of the gait pattern
        /// @return The duration of the gait pattern
        milliseconds getDuration() const;

        /// @brief Get the linear velocity of the gait pattern
        /// @return The linear velocity of the gait pattern
        Vector3f getLinearVelocity() const;

        /// @brief Get the angular velocity of the gait pattern
        /// @return The angular velocity of the gait pattern
        Vector3f getAngularVelocity() const;

        /// @brief Get the stance state of the gait pattern at a given time
        /// @param time The time to get the stance state at in milliseconds between 0 and the duration
        /// @return The stance state of the gait pattern at the given time
        StanceState getStanceState(const milliseconds &time) const;

    private:
        Vector3f linear_velocity_;
        Vector3f angular_velocity_;
        milliseconds duration_;
        std::map<milliseconds, StanceState> stance_pattern_;
    };
}

#endif