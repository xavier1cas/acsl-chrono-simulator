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
 * File:        piecewise-polynomial-trajectory.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        Sep 16, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Minimum jerk piecewise polynomial trajectory which works with the minjerkpoly module of matlab.
 *              Inherits the trajectorybase API.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef PIECEWISE_POLYNOMIAL_TRAJECTORY_HPP_
#define PIECEWISE_POLYNOMIAL_TRAJECTORY_HPP_

#include "sim-trajectory.hpp"

namespace _acsl_
{

namespace _trajectory_
{

class minjerkpoly : public trajectorybase {
public:
    minjerkpoly() = default;
    virtual ~minjerkpoly() {}

    // Update trajectory module
    void UpdateModule(const double t) override;

    // Getter and Setter for file name
    void SetFileName(const std::string& file) override { this->m_file = file; }
    const std::string& GetFileName() const override { return this->m_file; }

    // Function to initate the trajectory module
    void InitiateModule() override;

    // Getter for Tmax
    double GetTmax() const override { return this->tmax; }

    // Getter for the visual shape
    const std::shared_ptr<chrono::ChVisualShapeLine>& GetVisualShape() const override { return this->visualshape; }

    // Getters for user trajectory variables
    double GetTime() const override { return this->c_state.time; }
    const Eigen::Vector3d& GetPosition() const override { return this->c_state.position; }
    const Eigen::Vector3d& GetVelocity() const override { return this->c_state.velocity; }
    const Eigen::Vector3d& GetAcceleration() const override { return this->c_state.acceleration; }
    const Eigen::Vector3d& GetEulerAngle() const override { return this->c_state.euler; }
    const Eigen::Vector3d& GetEulerRate() const override { return this->c_state.euler_rate; }
    const Eigen::Vector3d& GetEulerAcc() const override { return this->c_state.euler_acceleration; }
    const Eigen::Quaterniond& GetQuat() const override { return this->c_state.quaternion; }
    const Eigen::Quaterniond& GetQuatRate() const override { return this->c_state.quaternion_rate; }
    const Eigen::Quaterniond& GetQuatAcc() const override { return this->c_state.quaternion_acceleration; }
    const Eigen::Vector3d& GetAngVel() const override { return this->c_state.angular_velocity; }
    const Eigen::Vector3d& GetAngAcc() const override { return this->c_state.angular_acceleration; }

protected:
    // Setter for Tmax
    void SetTmax(double Tmax) override { this->tmax = Tmax; }

    // Setters for user trajectory variables
    void SetTime(double t) override { this->c_state.time = t; }
    void SetPosition(const Eigen::Vector3d& pos) override { this->c_state.position = pos; }
    void SetVelocity(const Eigen::Vector3d& vel) override { this->c_state.velocity = vel; }
    void SetAcceleration(const Eigen::Vector3d& acc) override { this->c_state.acceleration = acc; }
    void SetEulerAngle(const Eigen::Vector3d& euler) override { this->c_state.euler = euler; }
    void SetEulerRate(const Eigen::Vector3d& euler_rate) override { this->c_state.euler_rate = euler_rate; }
    void SetEulerAcc(const Eigen::Vector3d& euler_acc) override { this->c_state.euler_acceleration = euler_acc; }
    void SetQuat(const Eigen::Quaterniond& quat) override { this->c_state.quaternion = quat; }
    void SetQuatRate(const Eigen::Quaterniond& quat_rate) override { this->c_state.quaternion_rate = quat_rate; }
    void SetQuatAcc(const Eigen::Quaterniond& quat_accel) override { this->c_state.quaternion_acceleration = quat_accel; }
    void SetAngVel(const Eigen::Vector3d& ang_vel) override { this->c_state.angular_velocity = ang_vel; }
    void SetAngAcc(const Eigen::Vector3d& ang_accel) override { this->c_state.angular_acceleration = ang_accel; }

    // Function to compute the time vector given a dt, ouptut is [0.0, dt, 2*dt, ....., ~Tmax]
    // Default time interval is 0.001 - we want precision 
    Eigen::VectorXd GetTimeVector(double Tmax, double dt = 1e-3);

private:
    // Class objects unique to piecewise-polynomial-trajectory
    std::vector<std::vector<double>> waypoints_;                         // The waypoints we actually want to hit
    std::vector<double> waypoint_times_;                                 // Times at which I want to reach the waypoints
    std::vector<std::vector<double>> piecewise_polynomial_coefficients_; // Matrix for piecewise polynomial coeffs
    int num_waypts;                                                      // Number of waypoints
    double hover_time_;                                                  // Hovering time after trajectory is done
    double landing_start_time;                                           // Time at which landing should start
    double landing_end_time;                                             // Time at which we have finally landed
    double LANDING_TRAJECTORY_EXECUTION_TIME = 4.0;                      // Time taken to land [hardcoded to 4.0s]
    double DISARM_AFTER_LANDING_BUFFER_TIME = 2.0;                       // Time taken to quit the simualtion after run
    double internal_time_adjusted;                                       // Current time adjusted w.r.t the segment
    int segment_;                                                        // Segment of the trajectory in which I am
    Eigen::Vector3d user_defined_position_previous_;                     // previous user_defined_position_
    double user_defined_yaw_previous_;                                   // previous user_defined_yaw_ 
    double norm_velocity_XY_;                                            // norm of the X and Y components of velocity
    
    // Polynomial coefficients
    Eigen::MatrixXd position_coef_x_;
    Eigen::MatrixXd position_coef_y_;
    Eigen::MatrixXd position_coef_z_;
    Eigen::MatrixXd velocity_coef_x_;
    Eigen::MatrixXd velocity_coef_y_;
    Eigen::MatrixXd velocity_coef_z_;
    Eigen::MatrixXd acceleration_coef_x_;
    Eigen::MatrixXd acceleration_coef_y_;
    Eigen::MatrixXd acceleration_coef_z_;
    Eigen::MatrixXd jerk_coef_x_;
    Eigen::MatrixXd jerk_coef_y_;
    Eigen::MatrixXd jerk_coef_z_;

    // Position polynomial coefficients for LANDING from (X, Y, -1) to (X, Y, 0) in 4 seconds
    const Eigen::VectorXd landing_position_coef_z_ = (Eigen::VectorXd(8) << 
        -0.001220703125,
        0.01708984375,
        -0.0820312499999999,
        0.13671875,
        0.0,
        0.0,
        0.0,
        -1.0
    ).finished();

    // Velocity polynomial coefficients for LANDING from (X, Y, -1) to (X, Y, 0) in 4 seconds
    const Eigen::VectorXd landing_velocity_coef_z_ = (Eigen::VectorXd(7) << 
        -0.00854492187499999,
        0.1025390625,
        -0.41015625,
        0.546874999999999,
        0.0,
        0.0,
        0.0
    ).finished();

    // Acceleration polynomial coefficients for LANDING from (X, Y, -1) to (X, Y, 0) in 4 seconds
    const Eigen::VectorXd landing_acceleration_coef_z_ = (Eigen::VectorXd(6) << 
        -0.0512695312499999,
        0.5126953125,
        -1.640625,
        1.640625,
        0.0,
        0.0
    ).finished();

    // Class functions for compute
    // Function to extract the polynomial coeffiecients
    std::vector<Eigen::MatrixXd> polyCoefAssigning(const std::vector<std::vector<double>>& poly_coeff_matrix_in);
    // Function to edtract and set the polynomial cofficient matrices
    void setPolynomialCoefficientMatrices();
    // Function to adjust the time to be fed to the various polynomials and identify the segment of the trajectory
    std::pair<double, int> polynomialTimeAdjusted(double time);
    // Compute the user-defined yaw from the trajectory velocity in the XY plane
    double yawComputation(const Eigen::VectorXd& Vx_coef, const Eigen::VectorXd& Vy_coef, double t);
    // Compute the user-defined yaw_dot from the trajectory velocity in the XY plane
    double yawDotComputation(const Eigen::VectorXd& Vx_coef, const Eigen::VectorXd& Vy_coef,
                             const Eigen::VectorXd& Ax_coef, const Eigen::VectorXd& Ay_coef, 
                             double t);
    // Compute the user-defined yaw_dot_dot from the trajectory velocity in the XY plane
    double yawDotDotComputation(const Eigen::VectorXd& Vx_coef, const Eigen::VectorXd& Vy_coef,
                                const Eigen::VectorXd& Ax_coef, const Eigen::VectorXd& Ay_coef,
                                const Eigen::VectorXd& Jx_coef, const Eigen::VectorXd& Jy_coef,
                                double t);
    // Updates yaw, yaw_dot, and yaw_dot_dot based on the requested time
    void updateUserDefinedYaw(double t);
};


}  // namespace _trajectory_

}  // namespace _acsl_

#endif  // PIECEWISE_POLYNOMIAL_TRAJECTORY_HPP_