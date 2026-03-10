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
 * File:        pid-quaternion-qrbp.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        March 09, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: PID  with quaternion and angular velocities for the QRBP. Inherts the class controller_base for the basic 
 *              functionality that is to be used for all control algorithms.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "pid-quaternion-qrbp.hpp"

namespace _acsl_
{

namespace _qrbp_
{

namespace _pid_quaternion_
{

// -------------------------------------------------------------------------
// Constructor Implementation:
//   - Calls the base (controller_base) constructor and passes 
//     both logger and trajectory.
// -------------------------------------------------------------------------
pid_quaternion::pid_quaternion(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory)
                    : ::_acsl_::_control_::controller_base(logger, trajectory)
{
    // Initial Conditions
    init();   
}

// -------------------------------------------------------------------------
// read_params Implementation:
// - Takes the hardcoded path for the gains and parameters and reads it in
// -------------------------------------------------------------------------
void pid_quaternion::read_params(const std::string& jsonFile)
{
    // Implementation here
    std::ifstream file(jsonFile);
    nlohmann::json j;
    file >> j;

    // Translational parameters
    cip.Kp_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KP_translational"], 3, 3);
    cip.Ki_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KI_translational"], 3, 3);
    cip.Kd_tran = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KD_translational"], 3, 3);

    // Rotational parameters
    cip.Kp_att = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KP_rotational"], 3, 3);
    cip.Kd_att = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KD_rotational"], 3, 3);

}

// Implementing virutal functions from controller_base
void pid_quaternion::init(){
    // Reading in the parameters
    read_params("../chrono-assets/parameters/qrbp/PID_QUATERNION/gains_PID_QUATERNION.json");

    // Set the inital conditions
    y.fill(0.0);
    dy.fill(0.0);

    // Setup the logging
    InitiateLogging();
    ConfigureHeaders();
}

// Update function for the controller
void pid_quaternion::update(double time, 
                            double x,
                            double y,
                            double z,
                            double vx,
                            double vy,
                            double vz,
                            double q0,
                            double q1,
                            double q2,
                            double q3,
                            double roll,
                            double pitch,
                            double yaw,
                            double w_x,
                            double w_y,
                            double w_z)     
{
    // 1. Assign all the states ----------------------------------------------------
    cim.t = time;

    // Assign the Translational States
    cim.x_tran_pos << x,y,z;
    cim.x_tran_vel << vx, vy, vz;

    // Assign the quaternions and ensure it is normalized
    // q0 = w, q1 = x, q2 = y, q3 = z
    cim.q = Eigen::Quaterniond(q0, q1, q2, q3).normalized();

    // Asign the angular velocity
    cim.omega << w_x, w_y, w_z;

    // 2. Get the reference trajectory ---------------------------------------------
    m_traj.UpdateModule(time);
    cim.r_user = m_traj.GetPosition();
    cim.r_dot_user = m_traj.GetVelocity();
    cim.r_ddot_user = m_traj.GetAcceleration();   
    cim.psi_user = m_traj.GetEulerAngle()(2);

    // 3. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 4. Assign the values from the integrator ------------------------------------
    assign_from_rk4();
}

// Function to assign elements from the rk4 integrator
void pid_quaternion::assign_from_rk4()
{
    //------------ assign after integration  ------------//
    int index = 0;
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_q_d0_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_q_d1_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_q_d2_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_q_d3_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_x_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_y_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_z_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_tran_pos_I, y, index);
}

// Model function for integration
void pid_quaternion::model(const _control_::rk4_array<double, NSI> &y, _control_::rk4_array<double, NSI> &dy, double t)
{
    //------------ Fill up the dy for integration  ------------//
    int index = 0;
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_q_d0_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_q_d1_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_q_d2_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_q_d3_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_x_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_y_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_omega_z_d_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_tran_pos, dy, index);
}


// Function to compute the outerloop control in I
void pid_quaternion::compute_translational_control_in_I()
{
    // Compute the error in the states
    cim.e_tran_pos << cim.x_tran_pos - cim.r_user;
    cim.e_tran_vel << cim.x_tran_vel - cim.r_dot_user;

    // Compute the baseline control input
    cim.mu_tran_baseline << MASS*( - cip.Kp_tran * cim.e_tran_pos 
                                   - cip.Kd_tran * cim.e_tran_vel
                                   - cip.Ki_tran * csm.e_tran_pos_I
                                   + cim.r_ddot_user );

    // Compute with the dynamic inversion without aerodynamics
    cim.mu_tran_I << cim.mu_tran_baseline - MASS * G * e3_basis;
}

// Compute the orientation commands and the desired total thrust
void pid_quaternion::compute_u1_q_d()
{
	// Compute the desired quaternion
    cim.f_d = cim.mu_tran_I.norm();

    // Compute the desired axis for the thrust in a computation safe way
    if (cim.f_d > 1e-6) {
        cim.f_d_hat_I = cim.mu_tran_I / cim.f_d;
    }
    else {
        cim.f_d = 0.0;
        cim.f_d_hat_I << 0.0, 0.0, 1.0;
    }


}


// Compute the rotational control
void pid_quaternion::compute_rotational_control()
{
                                           
}

// Function to compute the normalized thrusts
void pid_quaternion::compute_normalized_thrusts()
{
    // Compute the individual thrusts in Newtons
    cim.Thrust << mixer_matrix_qrbp * cim.u;

    // Saturate each element of the Thrust vector between MIN_THRUST and MAX_THRUST
    cim.Sat_Thrust = (cim.Thrust.cwiseMin(MAX_THRUST).cwiseMax(MIN_THRUST));

    // Compute the final control inputs
    control_input(0) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(0));
    control_input(1) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(1));
    control_input(2) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(2));
    control_input(3) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(3));

}

// Function that is called in sim-bridge.cpp
void pid_quaternion::run(const double time_step_rk4_) {

    // Process the dynamics --------------------------------------------------------
    // 1. Compute the aerodynamics

    // 2. Compute the translational control input
    compute_translational_control_in_I();

    // 3. Compute the thrust needed and the desired angles
    compute_u1_q_d();

    // 4. Compute the rotational control input
    compute_rotational_control();

    // 5. Compute the normalized thrust - Final Step
    compute_normalized_thrusts();

    // 6. Do the integration
    rk4.do_step(boost::bind(&pid_quaternion::model, this, bph::_1, bph::_2, bph::_3),
                y, cim.t, time_step_rk4_);
    
    // Capture the time after the execution of the controller
    cim.alg_end_time = std::chrono::high_resolution_clock::now();
    
    // Calculate the duration of the execution 
    cim.alg_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                            cim.alg_end_time - cim.alg_start_time).count();

    // 7. Log the Data after all the calculataions
    this->LogData();
    
}

// Function that is called during the constructor. 
bool pid_quaternion::InitiateLogging()
{
    auto status = _logger_::_filesystem_::setupControllerLogging(this->m_logger, "qrbp" ,"PID_QUATERNION");
    return status;
}

// Funciton that setups up the headers for the log file
void pid_quaternion::ConfigureHeaders()
{

    // Create the oss object
    std::ostringstream oss;

    oss << ", "
        << "Controller Time [s], "
        << "Algorithm Execution Time [us], "
        << "Position x [m], "
        << "Position y [m], "
        << "Position z [m], "
        << "Velocity x [m/s], "
        << "Velocity y [m/s], "
        << "Velocity z [m/s], "
        ;    

    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();

        _message_::SIMULATOR_INFO("[SIMCTL]: WROTE PID QUATERNION LOG HEADER DATA");
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE PID QUATERNION LOG HEADER DATA", e.what());
    }

}

void pid_quaternion::LogData()
{
    // Log the data
    std::ostringstream oss;

    oss << ", "
        << cim.t << ", "
        << cim.alg_duration << ", "
        << cim.x_tran_pos(0) << ", "
        << cim.x_tran_pos(1) << ", "
        << cim.x_tran_pos(2) << ", "
        << cim.x_tran_vel(0) << ", "
        << cim.x_tran_vel(1) << ", "
        << cim.x_tran_vel(2) << ", "
        ;

    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE PID QUATERNION LOG HEADER DATA", e.what());
    }
}


}   // namespace _pid_quaternion_

}   // namespace _qrbp_
    
}   // namespace _acsl_
