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
 * File:        vector-interpolation-trajectory.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        Sep 16, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Vector Interpolation trajectory which works with the transform_trajectory module of matlab. Inherits 
 *              the trajectorybase API.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef VECTOR_INTERPOLATION_TRAJECTORY_HPP_
#define VECTOR_INTERPOLATION_TRAJECTORY_HPP_

#include "sim-trajectory.hpp"

namespace _acsl_
{

namespace _trajectory_
{

class interpolation : public trajectorybase {
public:
    interpolation() = default;
    virtual ~interpolation() {}

    // Update trajectory module
    void UpdateModule(const double t) override;

    // Getter and Setter for file name
    void SetFileName(const std::string& file) override { this->m_file = file; }
    const std::string& GetFileName() const override { return this->m_file; }

    // Function to initate the trajectory module
    void InitiateModule() override;

    // Getter for Tmax
    double GetTmax() const override { return this->tmax; }

    // Getters for user trajectory variables
    const Eigen::Vector3d& GetPosition() const override { return this->m_user.position; }
    const Eigen::Vector3d& GetVelocity() const override { return this->m_user.velocity; }
    const Eigen::Vector3d& GetAcceleration() const override { return this->m_user.acceleration; }
    const Eigen::Vector3d& GetEulerAngle() const override { return this->m_user.euler; }
    const Eigen::Vector3d& GetEulerRate() const override { return this->m_user.euler_rate; }
    const Eigen::Vector3d& GetEulerAcc() const override { return this->m_user.euler_acceleration; }
    const Eigen::Quaterniond& GetQuat() const override { return this->m_user.quaternion; }
    const Eigen::Quaterniond& GetQuatRate() const override { return this->m_user.quaternion_rate; }
    const Eigen::Quaterniond& GetQuatAcc() const override { return this->m_user.quaternion_acceleration; }
    const Eigen::Vector3d& GetAngVel() const override { return this->m_user.angular_velocity; }
    const Eigen::Vector3d& GetAngAcc() const override { return this->m_user.angular_acceleration; }

protected:
    // Setter for Tmax
    void SetTmax(double Tmax) override { this->tmax = Tmax; }
    
    // Setters for user trajectory variables
    void SetPosition(const Eigen::Vector3d& pos) override { this->m_user.position = pos; }
    void SetVelocity(const Eigen::Vector3d& vel) override { this->m_user.velocity = vel; }
    void SetAcceleration(const Eigen::Vector3d& acc) override { this->m_user.acceleration = acc; }
    void SetEulerAngle(const Eigen::Vector3d& euler) override { this->m_user.euler = euler; }
    void SetEulerRate(const Eigen::Vector3d& euler_rate) override { this->m_user.euler_rate = euler_rate; }
    void SetEulerAcc(const Eigen::Vector3d& euler_acc) override { this->m_user.euler_acceleration = euler_acc; }
    void SetQuat(const Eigen::Quaterniond& quat) override { this->m_user.quaternion = quat; }
    void SetQuatRate(const Eigen::Quaterniond& quat_rate) override { this->m_user.quaternion_rate = quat_rate; }
    void SetQuatAcc(const Eigen::Quaterniond& quat_accel) override { this->m_user.quaternion_acceleration = quat_accel; }
    void SetAngVel(const Eigen::Vector3d& ang_vel) override { this->m_user.angular_velocity = ang_vel; }
    void SetAngAcc(const Eigen::Vector3d& ang_accel) override { this->m_user.angular_acceleration = ang_accel; }
};


}  // namespace _trajectory_

}  // namespace _acsl_

#endif  // VECTOR_INTERPOLATION_TRAJECTORY_HPP_