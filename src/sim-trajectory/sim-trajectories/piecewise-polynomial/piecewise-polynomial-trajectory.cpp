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
 * File:        piecewise-polynomial-trajectory.cpp
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

#include "piecewise-polynomial-trajectory.hpp"

namespace _acsl_
{

namespace _trajectory_
{
// ========================================================================================================
// HELPER FUNCTIONS FOR MINJERKPOLY INITIATION
// ========================================================================================================
// Function to assign polynomial coefficients to designated variables
std::vector<Eigen::MatrixXd> minjerkpoly::polyCoefAssigning
(const std::vector<std::vector<double>>& poly_coeff_matrix_in) 
{
    // Get the size of the input matrix
    int rows = poly_coeff_matrix_in.size();
    int cols = poly_coeff_matrix_in[0].size();
    
    // Calculate the size of the output matrices
    int out_rows = rows / 3;
    
    // Initialize output matrices for x, y, and z variables
    Eigen::MatrixXd poly_coeff_matrix_out_x(out_rows, cols);
    Eigen::MatrixXd poly_coeff_matrix_out_y(out_rows, cols);
    Eigen::MatrixXd poly_coeff_matrix_out_z(out_rows, cols);
    
    // Assign polynomial coefficients to designated variables
    for (int i = 0; i < out_rows; ++i) {
        for (int j = 0; j < cols; ++j) {
        poly_coeff_matrix_out_x(i, j) = poly_coeff_matrix_in[3 * i][j];
        poly_coeff_matrix_out_y(i, j) = poly_coeff_matrix_in[3 * i + 1][j];
        poly_coeff_matrix_out_z(i, j) = poly_coeff_matrix_in[3 * i + 2][j];
        }
    }
    
    // Return the output matrices
    return {poly_coeff_matrix_out_x, poly_coeff_matrix_out_y, poly_coeff_matrix_out_z};
}

// Function to extract and set the polynomial coefficient matrices
void minjerkpoly::setPolynomialCoefficientMatrices()
{
    const auto& pp_coefficients = this->piecewise_polynomial_coefficients_;
    
    // Call polyCoefAssigning to assign polynomial coefficients to designated variables
    auto position_coefs = this->polyCoefAssigning(pp_coefficients);

    // Extract coefficients for x, y, and z components
    this->position_coef_x_ = position_coefs[0];
    this->position_coef_y_ = position_coefs[1];
    this->position_coef_z_ = position_coefs[2];

    // Compute velocity coefficients
    this->velocity_coef_x_ = ::_shared_::_compute_::polyDerMatrix(position_coef_x_);
    this->velocity_coef_y_ = ::_shared_::_compute_::polyDerMatrix(position_coef_y_);
    this->velocity_coef_z_ = ::_shared_::_compute_::polyDerMatrix(position_coef_z_);

    // Compute acceleration coefficients
    this->acceleration_coef_x_ = ::_shared_::_compute_::polyDerMatrix(velocity_coef_x_);
    this->acceleration_coef_y_ = ::_shared_::_compute_::polyDerMatrix(velocity_coef_y_);
    this->acceleration_coef_z_ = ::_shared_::_compute_::polyDerMatrix(velocity_coef_z_);

    // Compute jerk coefficients
    this->jerk_coef_x_ = ::_shared_::_compute_::polyDerMatrix(acceleration_coef_x_);
    this->jerk_coef_y_ = ::_shared_::_compute_::polyDerMatrix(acceleration_coef_y_);
    this->jerk_coef_z_ = ::_shared_::_compute_::polyDerMatrix(acceleration_coef_z_);
}

// ========================================================================================================
// HELPER FUNCTIONS FOR MINJERKPOLY COMPUTATION
// ========================================================================================================
// Adjusts the time to be fed to the various polynomials and identifies the segment of the trajectory
std::pair<double, int> minjerkpoly::polynomialTimeAdjusted(double time)
{
    int segment = 0;
    double time_adjusted = 0.0;

    for (int i = 0; i < this->num_waypts - 1; ++ i)
    {
        if (time >= this->waypoint_times_[i] && time <= this->waypoint_times_[i + 1])
        {
            segment = i;
            time_adjusted = time - this->waypoint_times_[segment];
            return {time_adjusted, segment};        
        }
    }

    return {time_adjusted, segment};
}

// Compute the user-defined yaw from the trajectory velocity in the XY plane
double minjerkpoly::yawComputation(const Eigen::VectorXd& Vx_coef, const Eigen::VectorXd& Vy_coef, double t)
{
    // Compute the velocity components Vx and Vy at time t using polynomial coefficients
    double Vx = ::_shared_::_compute_::evaluatePolynomial(Vx_coef, t);
    double Vy = ::_shared_::_compute_::evaluatePolynomial(Vy_coef, t);

    // Compute the yaw angle using atan2
    double yaw = std::atan2(Vy, Vx);

    return yaw;
}

// Compute the user-defined yaw_dot from the trajectory velocity in the XY plane
double minjerkpoly::yawDotComputation(const Eigen::VectorXd& Vx_coef, const Eigen::VectorXd& Vy_coef,
                                      const Eigen::VectorXd& Ax_coef, const Eigen::VectorXd& Ay_coef, 
                                      double t)
{
    // Compute the acceleration components Ax and Ay at time t using polynomial coefficients
    double Ax = ::_shared_::_compute_::evaluatePolynomial(Ax_coef, t);
    double Ay = ::_shared_::_compute_::evaluatePolynomial(Ay_coef, t);

    // Compute the acceleration angle in the complex plane
    double acceleration_angle = std::atan2(Ay, Ax);

    // Compute the velocity and acceleration norms
    double velocity_norm = ::_shared_::_compute_::norm2D<Eigen::VectorXd, double>(Vx_coef, Vy_coef, t);
    double acceleration_norm = ::_shared_::_compute_::norm2D<Eigen::VectorXd, double>(Ax_coef, Ay_coef, t);

    // Compute the yaw angle
    double yaw = yawComputation(Vx_coef, Vy_coef, t);

    // Compute the yaw rate
    double yaw_dot = (acceleration_norm / velocity_norm) * std::sin(acceleration_angle - yaw);

    return yaw_dot;
}

// Compute the user-defined yaw_dot_dot from the trajectory velocity in the XY plane
double minjerkpoly::yawDotDotComputation(const Eigen::VectorXd& Vx_coef, const Eigen::VectorXd& Vy_coef,
                                         const Eigen::VectorXd& Ax_coef, const Eigen::VectorXd& Ay_coef,
                                         const Eigen::VectorXd& Jx_coef, const Eigen::VectorXd& Jy_coef,
                                         double t)
{
    // Compute the jerk components Jx and Jy at time t using polynomial coefficients
    double Jx = ::_shared_::_compute_::evaluatePolynomial(Jx_coef, t);
    double Jy = ::_shared_::_compute_::evaluatePolynomial(Jy_coef, t);

    // Compute the jerk angle in the complex plane
    double jerk_angle = std::atan2(Jy, Jx);

    // Compute the velocity and jerk norms
    double velocity_norm = ::_shared_::_compute_::norm2D<Eigen::VectorXd, double>(Vx_coef, Vy_coef, t);
    double jerk_norm = ::_shared_::_compute_::norm2D<Eigen::VectorXd, double>(Jx_coef, Jy_coef, t);

    // Compute the derivative of the velocity norm
    double velocity_norm_prime = ::_shared_::_compute_::norm2Dderivative<Eigen::VectorXd, double>(Vx_coef, Vy_coef, 
                                                                                                  Ax_coef, Ay_coef, t);

    // Compute the yaw angle and its derivative
    double yaw = yawComputation(Vx_coef, Vy_coef, t);
    double yaw_dot = yawDotComputation(Vx_coef, Vy_coef, Ax_coef, Ay_coef, t);

    // Compute the second derivative of yaw angle
    double yaw_dot_dot = (jerk_norm * std::sin(jerk_angle - yaw) - 2 * velocity_norm_prime * yaw_dot) / velocity_norm;

    return yaw_dot_dot;
}

// Function that updates the user-defined yaw, yaw_dot, and yaw_dot_dot based on the time at which it is requested 
// to be evaluated
void minjerkpoly::updateUserDefinedYaw(double t)
{
    // Compute the 2D norm of the trajectory velocity in the XY plane
    norm_velocity_XY_ = ::_shared_::_compute_::norm2D<Eigen::VectorXd, double>(velocity_coef_x_.row(segment_),
                                                                               velocity_coef_y_.row(segment_),
                                                                               internal_time_adjusted);

    // If the norm of the velocity in the XY plane is close to zero OR If the current time has passed the
    // last waypoint time, THEN set the trajectory to keep the last yaw angle
    if (this->norm_velocity_XY_ < 1e-3 || (t >= this->waypoint_times_.back()))
    {
        this->SetEulerAngle(
            Eigen::Vector3d(
                0.0,
                0.0,
                user_defined_yaw_previous_
            )
        );
        this->SetEulerRate(Eigen::Vector3d::Zero());
        this->SetEulerAcc(Eigen::Vector3d::Zero());
    }
    else
    {
        // Update the User defined yaw angle
        this->SetEulerAngle(
            Eigen::Vector3d(
                0.0,
                0.0,
                this->yawComputation(velocity_coef_x_.row(segment_),
                                     velocity_coef_y_.row(segment_),
                                     internal_time_adjusted)
            )
        );

        // Update the first derivative of the user-defined yaw angle
        this->SetEulerRate(
            Eigen::Vector3d(
                0.0,
                0.0,
                this->yawDotComputation(velocity_coef_x_.row(segment_),
                                        velocity_coef_y_.row(segment_),
                                        acceleration_coef_x_.row(segment_),
                                        acceleration_coef_y_.row(segment_),
                                        internal_time_adjusted)
            )
        );
                
        // Update the second derivative of the user-defined yaw angle
        this->SetEulerRate(
            Eigen::Vector3d(
                0.0,
                0.0,
                this->yawDotDotComputation(velocity_coef_x_.row(segment_),
                                           velocity_coef_y_.row(segment_),
                                           acceleration_coef_x_.row(segment_),
                                           acceleration_coef_y_.row(segment_),
                                           jerk_coef_x_.row(segment_),
                                           jerk_coef_y_.row(segment_),
                                           internal_time_adjusted)
            )
        );
    }

    // Update user_defined_yaw_previous_
    user_defined_yaw_previous_ = this->GetEulerAngle()(2);

    // Update all the other states in the trajectory modules (orientation) states
    this->SetQuat(Eigen::Quaterniond::Identity());
    this->SetQuatRate(Eigen::Quaterniond::Identity());
    this->SetQuatAcc(Eigen::Quaterniond::Identity());
    this->SetAngVel(Eigen::Vector3d::Zero());
    this->SetAngAcc(Eigen::Vector3d::Zero());
}

// ========================================================================================================
// MAIN FUNCTION OF THE TRAJECTORY MODULE
// ========================================================================================================
// Function to initate the interpolation trajectory module
// NOTE: Should only be run after the SetFileName() command is run.
void minjerkpoly::InitiateModule()
{
    // check if the file exists
    ::_shared_::_deserialize_::FileExists(this->GetFileName());
    
    // Read in the data file
    std::ifstream file(this->GetFileName()); 
    if (!file.is_open()) { ::_acsl_::_message_::SIMULATOR_ERROR("[SIMTRj]: CANNOT OPEN FILE" + this->GetFileName()); }

    // Process the json datafile for the data and store it in the member variables
    nlohmann::json j;
    file >> j;
    this->waypoints_ = j["waypoints"].get<std::vector<std::vector<double>>>();
    this->waypoint_times_ = j["waypoint_times"].get<std::vector<double>>();
    this->piecewise_polynomial_coefficients_ = j["piecewise_polynomial_coefficients"].get<std::vector<std::vector<double>>>();
    this->num_waypts = this->waypoint_times_.size();
    this->hover_time_ = j["hover_time"]["seconds"];
    
    // Assign the maximum time
    // Set the landing start time
    // This will be the sum of the end of the flight_trajectory & the time needed to hover.
    this->landing_start_time = this->waypoint_times_.back() + this->hover_time_;

    // Set the landing end time
    // This will be the landing_start_time + 4.0 seconds.
    this->landing_end_time = this->landing_start_time + this->LANDING_TRAJECTORY_EXECUTION_TIME;

    // Assign the maximum time
    this->SetTmax(this->landing_end_time + this->DISARM_AFTER_LANDING_BUFFER_TIME);

    // Setup the polynomial coefficeient matrices
    this->setPolynomialCoefficientMatrices();

    // Create and assign to nurbsasset
    auto controlpts = ::_shared_::_serialize_::serialize2ChVector3d(this->waypoints_[0],
                                                                    this->waypoints_[1],
                                                                    this->waypoints_[2]);
    this->visualshape = ::_shared_::_visualize_::createNurbsVisual(controlpts);
    
}

// Function to update the mimjerkpoly trajectory module to the current time
// NOTE: Needs to be run every cycle to have the trajectory ready to go.
void minjerkpoly::UpdateModule(const double t)
{
    // Update the time stamp
    this->SetTime(t);

    // Update the trajectory
    // Adjust time and identify segment
    // NOTE: If the current time has passed the last waypoint time, then segment_ = 0;
    std::tie(this->internal_time_adjusted, this->segment_) = this->polynomialTimeAdjusted(t);

    // If the internal time has passed the last waypoint time but it is lesser than the landing start time,
    // Set the trajectory to perform HOVER in the last position of the trajectory
    if(t >= this->waypoint_times_.back() && t < this->landing_start_time)
    {
        this->SetPosition(user_defined_position_previous_);
        this->SetVelocity(Eigen::Vector3d::Zero());
        this->SetAcceleration(Eigen::Vector3d::Zero());
    }
    // If the controller_time has passed the landing start time but not the landing end time,
    // perform the LANDING maneuver
    else if (t >= this->landing_start_time && t < this->landing_end_time)
    {
        // Evaluate and update user_defined_position_
        this->SetPosition(
            Eigen::Vector3d(
                user_defined_position_previous_(0),
                user_defined_position_previous_(1),
                ::_shared_::_compute_::evaluatePolynomial(this->landing_position_coef_z_, t - this->landing_start_time)
            )
        );

        // Evaluate and update user_defined_velocity_
        this->SetVelocity(
            Eigen::Vector3d(
                0.0,
                0.0,
                ::_shared_::_compute_::evaluatePolynomial(this->landing_velocity_coef_z_, t - this->landing_start_time)
            )
        );

        // Evaluate and update user_defined_acceleration_
        this->SetAcceleration(
            Eigen::Vector3d(
                0.0,
                0.0,
                ::_shared_::_compute_::evaluatePolynomial(this->landing_acceleration_coef_z_, t - this->landing_start_time)
            )
        );
    }    
    // If the current time has passed the landing end time, stay at the origin       
    else if (t >= this->landing_end_time)
    {
        this->SetPosition(this->user_defined_position_previous_);
        this->SetVelocity(Eigen::Vector3d::Zero());
        this->SetAcceleration(Eigen::Vector3d::Zero());
    }
    // If any of the above conditions are not met, follow the user defined trajectory
    else 
    {
        // Evaluate and update user_defined_position_
        this->SetPosition(
            Eigen::Vector3d(
                ::_shared_::_compute_::evaluatePolynomial(position_coef_x_.row(segment_), internal_time_adjusted),
                ::_shared_::_compute_::evaluatePolynomial(position_coef_y_.row(segment_), internal_time_adjusted),
                ::_shared_::_compute_::evaluatePolynomial(position_coef_z_.row(segment_), internal_time_adjusted)
            )
        );

        // Evaluate and update user_defined_velocity_
        this->SetVelocity(
            Eigen::Vector3d(
                ::_shared_::_compute_::evaluatePolynomial(velocity_coef_x_.row(segment_), internal_time_adjusted),
                ::_shared_::_compute_::evaluatePolynomial(velocity_coef_y_.row(segment_), internal_time_adjusted),
                ::_shared_::_compute_::evaluatePolynomial(velocity_coef_z_.row(segment_), internal_time_adjusted)
            )
        );

        // Evaluate and update user_defined_acceleration_
        this->SetAcceleration(
            Eigen::Vector3d(
                ::_shared_::_compute_::evaluatePolynomial(acceleration_coef_x_.row(segment_), internal_time_adjusted),
                ::_shared_::_compute_::evaluatePolynomial(acceleration_coef_y_.row(segment_), internal_time_adjusted),
                ::_shared_::_compute_::evaluatePolynomial(acceleration_coef_z_.row(segment_), internal_time_adjusted)
            )
        );
    }

    // Update user_defined_position_previous_
    user_defined_position_previous_ = this->GetPosition();

    // Update the user defined yaw
    this->updateUserDefinedYaw(t);
    
}

}   // namespace _trajectory_

}   // namespace _acsl_