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
 * File:        pid-geometric-qrbp.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        March 25, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: PID with geometric and angular velocities for the QRBP. Inherts the class controller_base for the basic 
 *              functionality that is to be used for all control algorithms.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "pid-geometric-qrbp.hpp"

namespace _acsl_
{

namespace _qrbp_
{

namespace _pid_geometric_
{

// -------------------------------------------------------------------------
// Constructor Implementation:
//   - Calls the base (controller_base) constructor and passes 
//     both logger and trajectory.
// -------------------------------------------------------------------------
pid_geometric::pid_geometric(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory)
                            : ::_acsl_::_control_::controller_base(logger, trajectory)
{
    // Initial Conditions
    init();   
}

// -------------------------------------------------------------------------
// read_params Implementation:
// - Takes the hardcoded path for the gains and parameters and reads it in
// -------------------------------------------------------------------------
void pid_geometric::read_params(const std::string& jsonFile)
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

    // Differentiator matrices
    

}

// Implementing virutal functions from controller_base
void pid_geometric::init(){
    // Reading in the parameters
    read_params("../chrono-assets/parameters/qrbp/PID_GEOMETRIC/gains_PID_GEOMETRIC.json");

    // Set the inital conditions
    y.fill(0.0);
    dy.fill(0.0);

    // Setup the logging
    InitiateLogging();
    ConfigureHeaders();
}

// Update function for the controller
void pid_geometric::update(double time, 
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

    // Assign the rotation matrices
    cim.Rji = cim.q.toRotationMatrix(); // rotation from the body to the inertial frame
    cim.Rij = cim.Rji.transpose();      // rotation from the inertial to the body frame

    // Asign the angular velocity
    cim.omega << w_x, w_y, w_z;

    // 2. Get the reference trajectory ---------------------------------------------
    m_traj.UpdateModule(time);
    cim.r_user = m_traj.GetPosition();
    cim.r_dot_user = m_traj.GetVelocity();
    cim.r_ddot_user = m_traj.GetAcceleration();   
    cim.psi_user = m_traj.GetEulerAngle()(2);
    cim.psi_user_unwrapped = ::_shared_::_compute_::unwrapPsiSimple(cim.psi_user, this->psiState);
    cim.psi_dot_user = m_traj.GetEulerRate()(2);

    // 3. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 4. Assign the values from the integrator ------------------------------------
    assign_from_rk4();
}

// Function to assign elements from the rk4 integrator
void pid_geometric::assign_from_rk4()
{
    //------------ assign after integration  ------------//
    int index = 0;
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_tran_pos_I, y, index);
}

// Model function for integration
void pid_geometric::model(const _control_::rk4_array<double, NSI> &y, _control_::rk4_array<double, NSI> &dy, double t)
{
    //------------ Fill up the dy for integration  ------------//
    int index = 0;
    
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_tran_pos, dy, index);
}


// Function to compute the outerloop control in I
void pid_geometric::compute_translational_control_in_I()
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
void pid_geometric::compute_u1_R_d()
{
	
}


// Compute the rotational control
void pid_geometric::compute_rotational_control()
{
    
                                               
}

// Function to compute the normalized thrusts
void pid_geometric::compute_normalized_thrusts()
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
void pid_geometric::run(const double time_step_rk4_) {

    // Process the dynamics --------------------------------------------------------
    // 1. Compute the aerodynamics

    // 2. Compute the translational control input
    compute_translational_control_in_I();

    // 3. Compute the thrust needed and the desired rotaion matrix
    compute_u1_R_d();

    // 4. Compute the rotational control input
    compute_rotational_control();

    // 5. Compute the normalized thrust - Final Step
    compute_normalized_thrusts();

    // 6. Do the integration
    rk4.do_step(boost::bind(&pid_geometric::model, this, bph::_1, bph::_2, bph::_3),
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
bool pid_geometric::InitiateLogging()
{
    auto status = _logger_::_filesystem_::setupControllerLogging(this->m_logger, "qrbp" ,"PID_GEOMETRIC");
    return status;
}

// Funciton that setups up the headers for the log file
void pid_geometric::ConfigureHeaders()
{

    // Create the oss object
    std::ostringstream oss;

    oss << ", "
        << "Controller Time [s], "
        << "Algorithm Execution Time [us], "
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

void pid_geometric::LogData()
{
    // Log the data
    std::ostringstream oss;

    oss << ", "
        << cim.t << ", "
        << cim.alg_duration << ", "
        ;

    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE PID QUATERNION LOG HEADER DATA", e.what());
    }
}


}   // namespace _pid_geometric_

}   // namespace _qrbp_
    
}   // namespace _acsl_
