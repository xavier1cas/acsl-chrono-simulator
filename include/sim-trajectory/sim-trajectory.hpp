/***********************************************************************************************************************
 * Copyright (c) 2025 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto. All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 * 
 * 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following
 *    disclaimer.
 * 
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 *    following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 **********************************************************************************************************************/

 /***********************************************************************************************************************
 * File:        sim-trajectory.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        Sep 16, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for the trajectory baseclass
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_TRAJECTORY_HPP_
#define SIM_TRAJECTORY_HPP_

#include "Eigen/Dense"

namespace _acsl_
{

namespace _trajectory_
{
// =====================================================================================================================
// Shared data structures
// =====================================================================================================================

// ----------------------------------------------------------------------------
// Structure: m_trajectory
//
// Purpose:
//   Stores user-defined trajectory data, including desired states and their derivatives,
//   in both linear (NED frame) and rotational (Euler and quaternion) representations.
//
// Members:
//   time                    - Timestamp of desired trajectory.
//   position                - Desired position in NED (North-East-Down) coordinates (meters).
//   velocity                - Desired linear velocity in NED coordinates (meters/second).
//   acceleration            - Desired linear acceleration in NED coordinates (meters/second^2).
//   euler                   - Desired Euler angles (roll, pitch, yaw) in radians.
//   euler_rate              - Desired Euler angle rates (roll, pitch, yaw) in radians/second.
//   euler_acceleration      - Desired Euler angle accelerations in radians/second^2.
//   quaternion              - Desired orientation as a unit quaternion.
//   quaternion_rate         - Desired quaternion derivative (represents orientation rate).
//   quaternion_acceleration - Desired quaternion second derivative (orientation acceleration).
//   angular_velocity        - Desired angular velocity in body frame (radians/second).
//   angular_acceleration    - Desired angular acceleration in body frame (radians/second^2).
// ----------------------------------------------------------------------------
struct m_trajectory {
    double time;
    Eigen::Vector3d position;
    Eigen::Vector3d velocity;
    Eigen::Vector3d acceleration;
    Eigen::Vector3d euler;
    Eigen::Vector3d euler_rate;
    Eigen::Vector3d euler_acceleration;
    Eigen::Quaterniond quaternion;
    Eigen::Quaterniond quaternion_rate;
    Eigen::Quaterniond quaternion_acceleration;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d angular_acceleration;
};


// =====================================================================================================================
// Polymorphic base interface for trajectory modules
//
// Purpose:
//   Specifies the API contract for all trajectory module types, allowing polymorphic management via trajectorybase*.
//   Provides getter/setter functions for both trajectory state and the associated user-defined data file.
//
// Structure:
//   - Public section : Module API and all public getters (including user data file).
//   - Protected section: State and file setters (for use by derived classes only).
//
// Notes:
//   - All methods are pure virtual (= 0) and must be implemented in derived types.
//   - Includes documented getter/setter for std::string m_file for user trajectory data file association.
// =====================================================================================================================

class trajectorybase {
public:
    virtual ~trajectorybase() = default;

    // ---------------- Public Getter API ----------------

    /**
     * @brief Update the trajectory module for time t, advancing internal state.
     * @param t Current simulation or trajectory time [s].
     */
    virtual void UpdateModule(const double t) = 0;

    /**
     * @brief Set the associated user trajectory data file.
     * @param file String with filename/path.
     */
    virtual void SetFileName(const std::string& file) = 0;

    /**
     * @brief Get the associated user trajectory data file.
     * @return const std::string& Reference to filename/path of data file.
     */
    virtual const std::string& GetFileName() const = 0;

    /**
     * @brief Initiate the trajectory module
     * @note Reads the file, sets up all the needed parameters for the trajectory module
     */
    virtual void InitiateModule() = 0;

    /**
     * @brief Get the maximum time associated with the trajectory.
     * @return double Value of tmax.
     */
    virtual double GetTmax() const = 0;

    // Getters for all trajectory variables (all const for read-only access)

    /// @brief Get desired position (NED, meters)
    virtual const Eigen::Vector3d& GetPosition() const = 0;

    /// @brief Get desired velocity (NED, m/s)
    virtual const Eigen::Vector3d& GetVelocity() const = 0;

    /// @brief Get desired acceleration (NED, m/s^2)
    virtual const Eigen::Vector3d& GetAcceleration() const = 0;

    /// @brief Get desired Euler angles (roll, pitch, yaw, radians)
    virtual const Eigen::Vector3d& GetEulerAngle() const = 0;

    /// @brief Get desired Euler rates (rpy dot, radians/s)
    virtual const Eigen::Vector3d& GetEulerRate() const = 0;

    /// @brief Get desired Euler accelerations (rpy ddot, radians/s^2)
    virtual const Eigen::Vector3d& GetEulerAcc() const = 0;

    /// @brief Get desired orientation as quaternion
    virtual const Eigen::Quaterniond& GetQuat() const = 0;

    /// @brief Get desired quaternion derivative
    virtual const Eigen::Quaterniond& GetQuatRate() const = 0;

    /// @brief Get desired quaternion second derivative
    virtual const Eigen::Quaterniond& GetQuatAcc() const = 0;

    /// @brief Get desired angular velocity (rad/s)
    virtual const Eigen::Vector3d& GetAngVel() const = 0;

    /// @brief Get desired angular acceleration (rad/s^2)
    virtual const Eigen::Vector3d& GetAngAcc() const = 0;

protected:
    // ---------------- Protected Setter API ----------------

    /**
     * @brief Set the maximum time associated with the trajectory.
     * @param tmax Value to set as maximum trajectory time.
     */
    virtual void SetTmax(double tmax) = 0;

    // Setters for all trajectory variables

    /// @brief Set desired position (NED, meters)
    virtual void SetPosition(const Eigen::Vector3d& pos) = 0;

    /// @brief Set linear velocity (NED, m/s)
    virtual void SetVelocity(const Eigen::Vector3d& vel) = 0;

    /// @brief Set linear acceleration (NED, m/s^2)
    virtual void SetAcceleration(const Eigen::Vector3d& acc) = 0;

    /// @brief Set Euler angles (radians)
    virtual void SetEulerAngle(const Eigen::Vector3d& euler) = 0;

    /// @brief Set Euler rates (radians/s)
    virtual void SetEulerRate(const Eigen::Vector3d& euler_rate) = 0;

    /// @brief Set Euler accelerations (radians/s^2)
    virtual void SetEulerAcc(const Eigen::Vector3d& euler_acc) = 0;

    /// @brief Set orientation as quaternion
    virtual void SetQuat(const Eigen::Quaterniond& quat) = 0;

    /// @brief Set quaternion rate
    virtual void SetQuatRate(const Eigen::Quaterniond& quat_rate) = 0;

    /// @brief Set quaternion acceleration
    virtual void SetQuatAcc(const Eigen::Quaterniond& quat_accel) = 0;

    /// @brief Set angular velocity (rad/s)
    virtual void SetAngVel(const Eigen::Vector3d& ang_vel) = 0;

    /// @brief Set angular acceleration (rad/s^2)
    virtual void SetAngAcc(const Eigen::Vector3d& ang_accel) = 0;

    // User-defined trajectory state storage
    m_trajectory m_user;

    // User-defined trajectory data file
    std::string m_file;

    // Maximum time of the trajectory
    double tmax;
};



}   // namespace _trajectory_

}   // namespace _acsl_

#endif  // SIM_TRAJECTORY_HPP_