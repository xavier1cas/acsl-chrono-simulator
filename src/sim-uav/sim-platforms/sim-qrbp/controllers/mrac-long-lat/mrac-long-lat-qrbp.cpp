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
 * File:        mrac-long-lat-qrbp.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        February 11, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: MRAC with longitudinal and lateral dynamics for the QRBP. Inherts the class controller_base for the basic 
 *              functionality that is to be used for all control algorithms.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "mrac-long-lat-qrbp.hpp"

namespace _acsl_
{

namespace _qrbp_
{

namespace _mrac_long_lat_
{

// -------------------------------------------------------------------------
// Constructor Implementation:
//   - Calls the base (controller_base) constructor and passes 
//     both logger and trajectory.
// -------------------------------------------------------------------------
mrac_long_lat::mrac_long_lat(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory)
                            : ::_acsl_::_control_::controller_base(logger, trajectory),
                              ::_acsl_::_uav_::_aerofoil_::simairfoil(::_acsl_::_uav_::_aerofoil_::AirFoilType::NACA0012)
{
    // Initial Conditions
    init();   
}

// -------------------------------------------------------------------------
// read_params Implementation:
// - Takes the hardcoded path for the gains and parameters and reads it in
// -------------------------------------------------------------------------
void mrac_long_lat::read_params(const std::string& jsonFile)
{
    // Implementation here
	std::ifstream file(jsonFile);
	nlohmann::json j;
    file >> j;

    // Adaptive differentiator gains and parameter matrices
    dip.C_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["C_differentiator"], 3, 6);
    dip.B_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["B_differentiator"], 6, 3);
    dip.L_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["L_differentiator"], 6, 3);

    dip.A_ref_y_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["A_ref_y_differentiator"], 6, 6);
    dip.A_tran_y_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["A_tran_y_differentiator"], 6, 6);

    dip.Gamma_y_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["Gamma_y_differentiator"], 3, 3);
    dip.Gamma_Theta_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["Gamma_Theta_differentiator"], 3, 3);
    dip.Gamma_g_y_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["Gamma_g_y_differentiator"], 3, 3);

    dip.projection_x_max_K_hat_y_diff = j["DIFFERENTIATOR"]["projection_x_max_K_hat_y_differentiator"];
    dip.projection_epsilon_K_hat_y_diff = j["DIFFERENTIATOR"]["projection_epsilon_K_hat_y_differentiator"];
    
    dip.projection_x_max_Theta_hat_diff = j["DIFFERENTIATOR"]["projection_x_max_Theta_hat_differentiator"];
    dip.projection_epsilon_Theta_hat_diff = j["DIFFERENTIATOR"]["projection_epsilon_Theta_hat_differentiator"];
    
    dip.projection_x_max_K_hat_g_y_diff = j["DIFFERENTIATOR"]["projection_x_max_K_hat_g_y_differentiator"];
    dip.projection_epsilon_K_hat_g_y_diff = j["DIFFERENTIATOR"]["projection_epsilon_K_hat_g_y_differentiator"];

    dip.K_ye_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["K_ye_differentiator"], 3, 3);
    dip.Theta_e_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["Theta_e_differentiator"], 3, 3);
    dip.K_gye_diff = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["DIFFERENTIATOR"]["K_gye_differentiator"], 3, 3);


    // Longitudinal Outerloop parameters
    cip.Kp_refmod_lon_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_OL"]["Kp_refmod_lon_ol"], 2, 2);
    cip.Kd_refmod_lon_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_OL"]["Kd_refmod_lon_ol"], 2, 2);
    cip.Kp_cmd_lon_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_OL"]["Kp_cmd_lon_ol"], 2, 2);
    cip.Kd_cmd_lon_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_OL"]["Kd_cmd_lon_ol"], 2, 2);
    cip.Ki_cmd_lon_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_OL"]["Ki_cmd_lon_ol"], 2, 2);
    cip.Kp_lon_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_OL"]["Kp_lon_ol"], 2, 2);
    cip.Kd_lon_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_OL"]["Kd_lon_ol"], 2, 2);
    cip.Ki_lon_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_OL"]["Ki_lon_ol"], 2, 2);
    cip.Q_lon_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_OL"]["Q_lon_ol"], 4, 4);
    cip.Gamma_x_lon_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_OL"]["Gamma_x_lon_ol"], 4, 4);
    cip.Gamma_r_lon_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_OL"]["Gamma_r_lon_ol"], 2, 2);
    cip.Gamma_Theta_lon_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_OL"]["Gamma_Theta_lon_ol"], 5, 5);
    cip.epsilon_x_lon_ol = j["LONGITUDINAL_OL"]["epsilon_x_lon_ol"];
    cip.epsilon_z_lon_ol = j["LONGITUDINAL_OL"]["epsilon_z_lon_ol"];
    cip.use_projection_operator_lon_out = j["LONGITUDINAL_OL"]["use_projection_operator_lon_out"];
    cip.dead_zone_delta_lon_out = j["LONGITUDINAL_OL"]["dead_zone_delta_lon_out"];
    cip.dead_zone_e0_lon_out = j["LONGITUDINAL_OL"]["dead_zone_e0_lon_out"];
    cip.sigma_x_lon_out = j["LONGITUDINAL_OL"]["sigma_x_lon_out"];
    cip.sigma_r_lon_out = j["LONGITUDINAL_OL"]["sigma_r_lon_out"];
    cip.sigma_Theta_lon_out = j["LONGITUDINAL_OL"]["sigma_Theta_lon_out"];
    cip.projection_x_max_x_lon_out = j["LONGITUDINAL_OL"]["projection_x_max_x_lon_out"];
    cip.projection_epsilon_x_lon_out = j["LONGITUDINAL_OL"]["projection_epsilon_x_lon_out"];
    cip.projection_x_max_r_lon_out = j["LONGITUDINAL_OL"]["projection_x_max_r_lon_out"];
    cip.projection_epsilon_r_lon_out = j["LONGITUDINAL_OL"]["projection_epsilon_r_lon_out"];
    cip.projection_x_max_Theta_lon_out = j["LONGITUDINAL_OL"]["projection_x_max_Theta_lon_out"];
    cip.projection_epsilon_Theta_lon_out = j["LONGITUDINAL_OL"]["projection_epsilon_Theta_lon_out"];

    // Longitudinal Innerloop parameters
    cip.Kp_refmod_lon_il = j["LONGITUDINAL_IL"]["Kp_refmod_lon_il"];
    cip.Kd_refmod_lon_il = j["LONGITUDINAL_IL"]["Kd_refmod_lon_il"];
    cip.Kp_cmd_lon_il = j["LONGITUDINAL_IL"]["Kp_cmd_lon_il"];
    cip.Kd_cmd_lon_il = j["LONGITUDINAL_IL"]["Kd_cmd_lon_il"];
    cip.Ki_cmd_lon_il = j["LONGITUDINAL_IL"]["Ki_cmd_lon_il"];
    cip.Kp_lon_il = j["LONGITUDINAL_IL"]["Kp_lon_il"];
    cip.Ki_lon_il = j["LONGITUDINAL_IL"]["Ki_lon_il"];
    cip.Kd_lon_il = j["LONGITUDINAL_IL"]["Kd_lon_il"];
    cip.Q_lon_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_IL"]["Q_lon_il"], 2, 2);
    cip.Gamma_x_lon_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_IL"]["Gamma_x_lon_il"], 2, 2);
    cip.Gamma_r_lon_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_IL"]["Gamma_r_lon_il"], 1, 1);
    cip.Gamma_Theta_lon_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LONGITUDINAL_IL"]["Gamma_Theta_lon_il"], 3, 3);
    cip.theta_max = j["LONGITUDINAL_IL"]["theta_max"];
    cip.use_projection_operator_lon_il = j["LONGITUDINAL_IL"]["use_projection_operator_lon_il"];
    cip.dead_zone_delta_lon_il = j["LONGITUDINAL_IL"]["dead_zone_delta_lon_il"];
    cip.dead_zone_e0_lon_il = j["LONGITUDINAL_IL"]["dead_zone_e0_lon_il"];
    cip.sigma_x_lon_il = j["LONGITUDINAL_IL"]["sigma_x_lon_il"];
    cip.sigma_r_lon_il = j["LONGITUDINAL_IL"]["sigma_r_lon_il"];
    cip.sigma_Theta_lon_il = j["LONGITUDINAL_IL"]["sigma_Theta_lon_il"];
    cip.projection_x_max_x_lon_il = j["LONGITUDINAL_IL"]["projection_x_max_x_lon_il"];
    cip.projection_epsilon_x_lon_il = j["LONGITUDINAL_IL"]["projection_epsilon_x_lon_il"];
    cip.projection_x_max_r_lon_il = j["LONGITUDINAL_IL"]["projection_x_max_r_lon_il"];
    cip.projection_epsilon_r_lon_il = j["LONGITUDINAL_IL"]["projection_epsilon_r_lon_il"];
    cip.projection_x_max_Theta_lon_il = j["LONGITUDINAL_IL"]["projection_x_max_Theta_lon_il"];
    cip.projection_epsilon_Theta_lon_il = j["LONGITUDINAL_IL"]["projection_epsilon_Theta_lon_il"];

    // Lateral Outerloop parameters
    cip.Kp_refmod_lat_ol = j["LATERAL_OL"]["Kp_refmod_lat_ol"];
    cip.Kd_refmod_lat_ol = j["LATERAL_OL"]["Kd_refmod_lat_ol"];
    cip.Kp_cmd_lat_ol = j["LATERAL_OL"]["Kp_cmd_lat_ol"];
    cip.Kd_cmd_lat_ol = j["LATERAL_OL"]["Kd_cmd_lat_ol"];
    cip.Ki_cmd_lat_ol = j["LATERAL_OL"]["Ki_cmd_lat_ol"];
    cip.Kp_lat_ol = j["LATERAL_OL"]["Kp_lat_ol"];
    cip.Kd_lat_ol = j["LATERAL_OL"]["Kd_lat_ol"];
    cip.Ki_lat_ol = j["LATERAL_OL"]["Ki_lat_ol"];
    cip.Q_lat_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_OL"]["Q_lat_ol"], 2, 2);
    cip.Gamma_x_lat_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_OL"]["Gamma_x_lat_ol"], 2, 2);
    cip.Gamma_r_lat_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_OL"]["Gamma_r_lat_ol"], 1, 1);
    cip.Gamma_Theta_lat_ol = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_OL"]["Gamma_Theta_lat_ol"], 2, 2);
    cip.use_projection_operator_lat_ol = j["LATERAL_OL"]["use_projection_operator_lat_ol"];
    cip.dead_zone_delta_lat_ol = j["LATERAL_OL"]["dead_zone_delta_lat_ol"];
    cip.dead_zone_e0_lat_ol = j["LATERAL_OL"]["dead_zone_e0_lat_ol"];
    cip.sigma_x_lat_ol = j["LATERAL_OL"]["sigma_x_lat_ol"];
    cip.sigma_r_lat_ol = j["LATERAL_OL"]["sigma_r_lat_ol"];
    cip.sigma_Theta_lat_ol = j["LATERAL_OL"]["sigma_Theta_lat_ol"];
    cip.projection_x_max_x_lat_ol = j["LATERAL_OL"]["projection_x_max_x_lat_ol"];
    cip.projection_epsilon_x_lat_ol = j["LATERAL_OL"]["projection_epsilon_x_lat_ol"];
    cip.projection_x_max_r_lat_ol = j["LATERAL_OL"]["projection_x_max_r_lat_ol"];
    cip.projection_epsilon_r_lat_ol = j["LATERAL_OL"]["projection_epsilon_r_lat_ol"];
    cip.projection_x_max_Theta_lat_ol = j["LATERAL_OL"]["projection_x_max_Theta_lat_ol"];
    cip.projection_epsilon_Theta_lat_ol = j["LATERAL_OL"]["projection_epsilon_Theta_lat_ol"];

    // Lateral Innerloop parameters
    cip.Kp_refmod_lat_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_IL"]["Kp_refmod_lat_il"], 2, 2);
    cip.Kp_cmd_lat_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_IL"]["Kp_cmd_lat_il"], 2, 2);
    cip.Ki_cmd_lat_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_IL"]["Ki_cmd_lat_il"], 2, 2);
    cip.Kp_lat_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_IL"]["Kp_lat_il"], 2, 2);
    cip.Kd_lat_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_IL"]["Kd_lat_il"], 2, 2);
    cip.Ki_lat_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_IL"]["Ki_lat_il"], 2, 2);
    cip.Q_lat_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_IL"]["Q_lat_il"], 2, 2);
    cip.Gamma_x_lat_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_IL"]["Gamma_x_lat_il"], 2, 2);
    cip.Gamma_r_lat_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_IL"]["Gamma_r_lat_il"], 2, 2);
    cip.Gamma_Theta_lat_il = ::_shared_::_deserialize_::jsonToScaledMatrixXd(j["LATERAL_IL"]["Gamma_Theta_lat_il"], 4, 4);
    cip.use_projection_operator_lat_il = j["LATERAL_IL"]["use_projection_operator_lat_il"];
    cip.dead_zone_delta_lat_il = j["LATERAL_IL"]["dead_zone_delta_lat_il"];
    cip.dead_zone_e0_lat_il = j["LATERAL_IL"]["dead_zone_e0_lat_il"];
    cip.sigma_x_lat_il = j["LATERAL_IL"]["sigma_x_lat_il"];
    cip.sigma_r_lat_il = j["LATERAL_IL"]["sigma_r_lat_il"];
    cip.sigma_Theta_lat_il = j["LATERAL_IL"]["sigma_Theta_lat_il"];
    cip.projection_x_max_x_lat_il = j["LATERAL_IL"]["projection_x_max_x_lat_il"];
    cip.projection_epsilon_x_lat_il = j["LATERAL_IL"]["projection_epsilon_x_lat_il"];
    cip.projection_x_max_r_lat_il = j["LATERAL_IL"]["projection_x_max_r_lat_il"];
    cip.projection_epsilon_r_lat_il = j["LATERAL_IL"]["projection_epsilon_r_lat_il"];
    cip.projection_x_max_Theta_lat_il = j["LATERAL_IL"]["projection_x_max_Theta_lat_il"];
    cip.projection_epsilon_Theta_lat_il = j["LATERAL_IL"]["projection_epsilon_Theta_lat_il"];

}

// Implementing virtual functios from controller_base
void mrac_long_lat::init(){
    // Reading in the parameters
    read_params("../chrono-assets/parameters/qrbp/MRAC_LONG_LAT/gains_MRAC_LONG_LAT.json");

    // Set the intial conditions
    y.fill(0.0);
    dy.fill(0.0);

    // Setup the logging
    InitiateLogging();
    ConfigureHeaders();

    // Initiate the gain matrices for the differentiator
    dsm.K_hat_y_2l_mrad << dip.K_ye_diff;
    dsm.Theta_hat_2l_mrad << dip.Theta_e_diff;
    dsm.K_hat_g_y_2l_mrad << dip.K_gye_diff;

    // Intiate the rk4 vector for the differntiator gain matrices
    int index = 6;
    ::_shared_::_serialize_::assignElementsToDxdt(dip.K_ye_diff, this->y, index);       // For 2L MRAD
    ::_shared_::_serialize_::assignElementsToDxdt(dip.Theta_e_diff, this->y, index);    // For 2L MRAD
    ::_shared_::_serialize_::assignElementsToDxdt(dip.K_gye_diff, this->y, index);      // For 2L MRAD

    // DISPLAY A DEBUG MESSAGE
    _message_::SIMULATOR_INFO("[SIMCTL]: INITIAL PARAMETERS COMPUTED FOR MRAD DIFFERENTIATOR");

    // Set the longituindal outerloop parameters
    // Initialize to zero the 4x4 matrix
    ::_shared_::_initiate_::initMat(cip.A_ref_lon_ol);
    // Set the top-right 2x2 block as follows
    cip.A_ref_lon_ol.block<2, 2>(0,2) = Eigen::Matrix2d::Identity();
    // Set the bottom-left 2x2 block as follows
    cip.A_ref_lon_ol.block<2, 2>(2, 0) = -cip.Kp_refmod_lon_ol;
    // Set the bottom-right 2x2 block as follows
    cip.A_ref_lon_ol.block<2, 2>(2, 2) = -cip.Kd_refmod_lon_ol;

    // Initialize to zero the 4x2 matrix and set the bottom 2x2 block as follows
    ::_shared_::_initiate_::initMat(cip.B_ref_lon_ol);
    cip.B_ref_lon_ol.block<2, 2>(2, 0) = (1.0 / MASS) * Eigen::Matrix2d::Identity();

    // Solve the continuous Lyapunov equation to compute P_lon_ol
    cip.P_long_ol = ::_lyapunov_solver_::RealContinuousLyapunovEquation(cip.A_ref_lon_ol, cip.Q_lon_ol);

    // Initialize to zero the 4x2 matrix and set the bottom 2x2 block as follows
	::_shared_::_initiate_::initMat(cip.B_lon_ol);
	cip.B_lon_ol.block<2, 2>(2, 0) = Eigen::Matrix2d::Identity();

    // DISPLAY A DEBUG MESSAGE
    _message_::SIMULATOR_INFO("[SIMCTL]: INITIAL PARAMETERS COMPUTED FOR LONGITUDINAL OUTERLOOP");

    // Set the longituindal innerloop parameters
    cip.A_ref_lon_il(0,0) = 0.0;
    cip.A_ref_lon_il(0,1) = 1.0;
    cip.A_ref_lon_il(1,0) = -cip.Kp_refmod_lon_il;
    cip.A_ref_lon_il(1,1) = -cip.Kd_refmod_lon_il;

    // Initialize to zero the 4x2 matrix and set the bottom 2x2 block as follows
    cip.B_ref_lon_il(0,0) = 0.0;
    cip.B_ref_lon_il(1,0) = (1.0/inertia_matrix_b(1,1));
    
    // Solve the continuous Lyapunov equation to compute P_lon_ol
    cip.P_long_il = ::_lyapunov_solver_::RealContinuousLyapunovEquation(cip.A_ref_lon_il, cip.Q_lon_il);

    // Set the longitudinal innerloop parameters
    // Initialize to zero the 2x1 matrix and set the bottom 1x1 block as follows
	cip.B_lon_il << 0, 1;

    // DISPLAY A DEBUG MESSAGE
    _message_::SIMULATOR_INFO("[SIMCTL]: INITIAL PARAMETERS COMPUTED FOR LONGITUDINAL INNERLOOP");

    // Set the lateral outerloop parameters
    cip.A_ref_lat_ol(0,0) = 0.0;
    cip.A_ref_lat_ol(0,1) = 1.0;
    cip.A_ref_lat_ol(1,0) = -cip.Kp_refmod_lat_ol;
    cip.A_ref_lat_ol(1,1) = -cip.Kd_refmod_lat_ol;

    cip.B_ref_lat_ol(0,0) = 0.0;
    cip.B_ref_lat_ol(1,0) = (1.0/MASS);

    // Solve the continuous Lyapunov equation to compute P_lat_ol
    cip.P_lat_ol = ::_lyapunov_solver_::RealContinuousLyapunovEquation(cip.A_ref_lat_ol, cip.Q_lat_ol);

    cip.B_lat_ol << 0, 1;

    // DISPLAY A DEBUG MESSAGE
    _message_::SIMULATOR_INFO("[SIMCTL]: INITIAL PARAMETERS COMPUTED FOR LATERAL OUTERLOOP");

    // Set the lateral innerloop parameters
    cip.A_ref_lat_il << -cip.Kp_refmod_lat_il;

    cip.B_ref_lat_il << Eigen::Matrix2d::Identity();

    // Solve the continuous Lyapunov equation to compute P_lat_il
    cip.P_lat_il = ::_lyapunov_solver_::RealContinuousLyapunovEquation(cip.A_ref_lat_il, cip.Q_lat_il);

    cip.B_lat_il << Eigen::Matrix2d::Identity();
    
}

// Update function for the controller.
void mrac_long_lat::update(double time, 
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

    // Assign all the states to internal members for ease of use across the controller functions
    cim.x = x;
    cim.y = y;
    cim.z = z;
    cim.vx = vx;
    cim.vy = vy;
    cim.vz = vz;
    cim.roll = roll;
    cim.pitch = pitch;
    cim.yaw = yaw;
    cim.w_x = w_x;
    cim.w_y = w_y;
    cim.w_z = w_z;
        
    // Assign the quaternion
    cim.q.w() = q0;
    cim.q.x() = q1;
    cim.q.y() = q2;
    cim.q.z() = q3;

    // 2. Get the reference trajectory ---------------------------------------------
    m_traj.UpdateModule(time);
    cim.r_user = m_traj.GetPosition();
    cim.v_user = m_traj.GetVelocity();
    cim.a_user = m_traj.GetAcceleration();
    cim.r_user_lon_out << cim.r_user(0), cim.r_user(2);
    cim.v_user_lon_out << cim.v_user(0), cim.v_user(2);
    cim.a_user_lon_out << cim.a_user(0), cim.a_user(2);

    // 3. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 4. Compute the rotation matrices --------------------------------------------
    cim.R_I_J << ::_shared_::_transformations_::rotationMatrix321GlobalToLocal(cim.roll, cim.pitch, cim.yaw);
    cim.R_J_I << ::_shared_::_transformations_::rotationMatrix321LocalToGlobal(cim.roll, cim.pitch, cim.yaw);

    // 5. Compute the body velocities ----------------------------------------------
    cim.v_I << cim.vx, cim.vy, cim.vz;
    cim.v_J << cim.R_I_J * cim.v_I;
    
    // 6. Compute the norm of the body velocities ----------------------------------
    cim.v_norm = cim.v_J.norm();

    // 7. Compute the angle of attack for the upper and lower wings ----------------
    cim.alpha_up = atan2(cim.v_J(2) - LX * cim.w_y, cim.v_J(0));
    cim.alpha_lw = atan2(cim.v_J(2) + LX * cim.w_y, cim.v_J(0));
    cim.alpha_com = atan2(cim.v_J(2), cim.v_J(0));
    cim.gamma_com = cim.pitch - cim.alpha_com;

    // 5. Assign the values from the integrator ------------------------------------
    assign_from_rk4();

    // 6. Finally, Initialize the differentiator's estimated state vector ----------
    if(!dim.first_run_differentiator)
    {
        // Initiate the x_hat vector
        dsm.x_hat_2l_mrad.setZero(dsm.x_hat_2l_mrad.size());

        // Build 6x1 vector [eta_rot; eta_rot]
        Eigen::Matrix<double, 6, 1> zero_vec;
        zero_vec << cim.roll, cim.pitch, cim.yaw,
                    cim.roll, cim.pitch, cim.yaw;

        // Initate the corresponding rk4 vector
        int index = 0;
        ::_shared_::_serialize_::assignElementsToDxdt(zero_vec, this->y, index);       // For 2L MRAD

        _message_::SIMULATOR_INFO("[SIMCTL]: INITIAL X HAT VECTOR SET FOR DIFFERENTIATOR");

        // Set the initialization condition to true
        dim.first_run_differentiator = true;
    }

    // 9. Load the angular velocity vector -----------------------------------------
    cim.omega_rot << cim.w_x, cim.w_y, cim.w_z;

    // 10. Compute the Jacobian that rotates angular rates to angular velocities ---
    cim.Jacobian << ::_shared_::_transformations_::jacobianMatrix(roll, pitch);
    

}

// Function to assign elements form the rk4 integrator
void mrac_long_lat::assign_from_rk4()
{
    //------------ assign elements of the differentiator after integration  ------------//
    int index = 0;
    ::_shared_::_deserialize_::assignElementsToMembers(dsm.x_hat_2l_mrad, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(dsm.K_hat_y_2l_mrad, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(dsm.Theta_hat_2l_mrad, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(dsm.K_hat_g_y_2l_mrad, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(dsm.int_euler_angles, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(dsm.eta_2l_mrad, y, index);

    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_ref_lon_out_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.x_ref_lon_out, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_lon_out_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_x_lon_out, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_r_lon_out, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.Theta_hat_lon_out, y, index);

    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_theta_cmd_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_theta_cmd_d_filter, y, index);

    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_ref_lon_in_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.x_ref_lon_in, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_lon_in_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_x_lon_in, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_r_lon_in, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.Theta_hat_lon_in, y, index);

    ::_shared_::_deserialize_::assignElementsToMembers(csm.x_ref_lat_out, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_ref_lat_out_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_lat_out_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_x_lat_out, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_r_lat_out, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.Theta_hat_lat_out, y, index);

    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_psi_cmd_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_wx_cmd_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_wz_cmd_filter, y, index);

    ::_shared_::_deserialize_::assignElementsToMembers(csm.x_ref_lat_in, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_lat_in_orientation_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_ref_lat_in_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_x_lat_in, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.K_hat_r_lat_in, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.Theta_hat_lat_in, y, index);

}

// Model function for integration
void mrac_long_lat::model(const _control_::rk4_array<double, NSI> &y, _control_::rk4_array<double, NSI> &dy, double t)
{   
    //------------ Fill up the dy for integration for the differentiator ------------//
    int index = 0;
    ::_shared_::_serialize_::assignElementsToDxdt(dim.x_hat_2l_mrad_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(dim.K_hat_y_2l_mrad_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(dim.Theta_hat_2l_mrad_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(dim.K_hat_g_y_2l_mrad_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(dim.eta_rot_unwrapped, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(dim.eta_2l_mrad_dot, dy, index);
    
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_ref_lon_out, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.x_ref_lon_out_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_lon_out_pos, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_x_lon_out_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_r_lon_out_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.Theta_hat_lon_out_dot, dy, index);
    
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_theta_cmd_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_theta_cmd_d_filter, dy, index);
    
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_ref_lon_in, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.x_ref_lon_in_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_lon_in_theta, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_x_lon_in_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_r_lon_in_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.Theta_hat_lon_in_dot, dy, index);   

    ::_shared_::_serialize_::assignElementsToDxdt(cim.x_ref_lat_out_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_ref_lat_out, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_lat_out_pos, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_x_lat_out_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_r_lat_out_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.Theta_hat_lat_out_dot, dy, index);

    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_psi_cmd_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_wx_cmd_filter, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.internal_state_wz_cmd_filter, dy, index);

    ::_shared_::_serialize_::assignElementsToDxdt(cim.x_ref_lat_in_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_lat_in_orientation, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.e_ref_lat_in, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_x_lat_in_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.K_hat_r_lat_in_dot, dy, index);
    ::_shared_::_serialize_::assignElementsToDxdt(cim.Theta_hat_lat_in_dot, dy, index);  
    

}

// Function to compute the differentiator
void mrac_long_lat::differentiate_for_euler_rates() {

    // ----------------------------------------------------------------------------- COMMON
    dim.eta_rot_unwrapped << cim.roll,                                                          // \phi
                             cim.pitch,                                                         // \theta
                             ::_shared_::_compute_::unwrapPsiSimple(cim.yaw, this->psiState);   // \psi

    // ----------------------------------------------------------------------------- 2L MRAD
    // Assign the total z_measured vector (use actual state vector)
    dim.z_measured_2l_mrad << dsm.int_euler_angles(0),  // \int \phi(\tau) d\tau
                              dsm.int_euler_angles(1),  // \int \theta(\tau) d\tau
                              dsm.int_euler_angles(2),  // \int \psi(\tau) d\tau
                              dim.eta_rot_unwrapped(0), // \phi 
                              dim.eta_rot_unwrapped(1), // \theta
                              dim.eta_rot_unwrapped(2); // \psi

    // Assign the measured output (use actual state vector)
    dim.y_measured_2l_mrad << dip.C_diff * dim.z_measured_2l_mrad;

    // Compute the observation error
    dim.obs_error_2l_mrad << dim.y_measured_2l_mrad - dip.C_diff * dsm.x_hat_2l_mrad;

    // Compute the transient error
    dim.nu_2l_mrad << dim.z_measured_2l_mrad - dsm.x_hat_2l_mrad - dsm.eta_2l_mrad;

    // Regressor vector for the observer - It is the signal itself as we assume some noise in it
    dim.Phi_y_2l_mrad << dim.eta_rot_unwrapped;

    // Compute the virtual control input for the differentitator
    dim.u_2l_mrad << dim.eta_rot_unwrapped - dsm.K_hat_y_2l_mrad.transpose() * dim.y_measured_2l_mrad 
                    + dsm.Theta_hat_2l_mrad.transpose() * dim.Phi_y_2l_mrad
                    - dsm.K_hat_g_y_2l_mrad.transpose() * dim.obs_error_2l_mrad;

    // Compute the estimated state to be integrated
    dim.x_hat_2l_mrad_dot << dip.A_ref_y_diff * dsm.x_hat_2l_mrad + dip.B_diff * dim.u_2l_mrad
                           + dip.L_diff * dim.obs_error_2l_mrad;

    // Compute the transient error model
    dim.eta_2l_mrad_dot << dip.A_tran_y_diff * dsm.eta_2l_mrad;

    // Compute the derivative of the differentiator gains to be integrated
    dim.K_hat_y_2l_mrad_dot << -dip.Gamma_y_diff * dim.y_measured_2l_mrad * dim.nu_2l_mrad.transpose();
    dim.Theta_hat_2l_mrad_dot << -dip.Gamma_Theta_diff * dim.Phi_y_2l_mrad * dim.nu_2l_mrad.transpose();
    dim.K_hat_g_y_2l_mrad_dot << -dip.Gamma_g_y_diff * dim.obs_error_2l_mrad * dim.nu_2l_mrad.transpose() * dip.C_diff.transpose();

    // Projection operator - Ball - NO boolean to switch off projection. It is always on.

    // Projection operator K_hat_y_2l_mrad
    ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(dsm.K_hat_y_2l_mrad)> proj_op_output_K_hat_y_2l_mrad = 
        ::_shared_::_projection_operator_::_ball_::projectionMatrix(dsm.K_hat_y_2l_mrad,
                                                                    dim.K_hat_y_2l_mrad_dot,
                                                                    dip.projection_x_max_K_hat_y_diff,
                                                                    dip.projection_epsilon_K_hat_y_diff);
                                                                    
    dim.K_hat_y_2l_mrad_dot = proj_op_output_K_hat_y_2l_mrad.projected_matrix;
    dim.proj_op_activated_K_hat_y_2l_mrad = proj_op_output_K_hat_y_2l_mrad.projection_operator_activated;

    // Projection operator Theta_hat_2l_mrad
    ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(dsm.Theta_hat_2l_mrad)> proj_op_output_Theta_hat_2l_mrad = 
        ::_shared_::_projection_operator_::_ball_::projectionMatrix(dsm.Theta_hat_2l_mrad,
                                                                    dim.Theta_hat_2l_mrad_dot,
                                                                    dip.projection_x_max_Theta_hat_diff,
                                                                    dip.projection_epsilon_Theta_hat_diff);

    dim.Theta_hat_2l_mrad_dot = proj_op_output_Theta_hat_2l_mrad.projected_matrix;
    dim.proj_op_activated_Theta_hat_2l_mrad = proj_op_output_Theta_hat_2l_mrad.projection_operator_activated;

    // Projection operator K_hat_g_y_2l_mrad
    ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(dsm.K_hat_g_y_2l_mrad)> proj_op_output_K_hat_g_y_2l_mrad = 
        ::_shared_::_projection_operator_::_ball_::projectionMatrix(dsm.K_hat_g_y_2l_mrad,
                                                                    dim.K_hat_g_y_2l_mrad_dot,
                                                                    dip.projection_x_max_K_hat_g_y_diff,
                                                                    dip.projection_epsilon_K_hat_g_y_diff);

    dim.K_hat_g_y_2l_mrad_dot = proj_op_output_K_hat_g_y_2l_mrad.projected_matrix;
    dim.proj_op_activated_K_hat_g_y_2l_mrad = proj_op_output_K_hat_g_y_2l_mrad.projection_operator_activated;

}

// Function to compute the outerloop of the longitudinal controller
void mrac_long_lat::compute_outerloop_longitudinal()
{
    // Load the state vector
    cim.x_lon_out << cim.x, cim.z, cim.vx, cim.vz;

    // Compute the error 
    cim.e_lon_out_pos << csm.x_ref_lon_out.head<2>() - cim.x_lon_out.head<2>();
    cim.e_lon_out_vel << csm.x_ref_lon_out.tail<2>() - cim.x_lon_out.tail<2>();

    // Compute the longitudinal position error between the reference model and the user defined trajectory
    cim.e_ref_lon_out << csm.x_ref_lon_out.head<2>() - cim.r_user_lon_out;

    // Compute the reference command input [reference model - user_defined_trajectory]
    cim.r_cmd_lon_out << MASS * (- cip.Ki_cmd_lon_ol * csm.e_ref_lon_out_I  // Integral term
                                 + cip.Kp_cmd_lon_ol * cim.r_user_lon_out   // Proportional term
                                 + cip.Kd_cmd_lon_ol * cim.v_user_lon_out   // Derivative term
                                 + cim.a_user_lon_out);                     // Feedforward term

    // Compute the reference model
    cim.x_ref_lon_out_dot << cip.A_ref_lon_ol * csm.x_ref_lon_out
                           + cip.B_ref_lon_ol * cim.r_cmd_lon_out;          

    // Compute the aerodynamic inversion term
    auto lift_cache = (ComputeCL(cim.alpha_lw) + ComputeCL(cim.alpha_up));
    auto drag_cache = (ComputeCD(cim.alpha_lw) + ComputeCD(cim.alpha_up));
    cim.F_inv_lon_out(0) = - DYN_PRESS_COEFF_W * cim.v_norm * cim.v_norm * (lift_cache * sin(cim.gamma_com) + drag_cache * cos(cim.gamma_com));
    cim.F_inv_lon_out(1) = - DYN_PRESS_COEFF_W * cim.v_norm * cim.v_norm * (lift_cache * cos(cim.gamma_com) - drag_cache * sin(cim.gamma_com));
                           
    // Compute the baseline control input
    cim.F_baseline_lon_out << MASS * (- cip.Kp_lon_ol * cim.e_lon_out_pos        // Proportional term
                                      - cip.Kd_lon_ol * cim.e_lon_out_vel        // Derivative term
                                      - cip.Ki_lon_ol * csm.e_lon_out_I          // Integral term
                                      + cim.x_ref_lon_out_dot.tail<2>()          // Feedforward term
                                      + cim.F_inv_lon_out);                      // Aerodynamic inversion term

    // Compute the augmented regressor vector
    cim.regressor_lon_out << cim.v_norm * cim.v_norm * (lift_cache * sin(cim.gamma_com) + drag_cache * cos(cim.gamma_com)),
                             cim.v_norm * cim.v_norm * (lift_cache * cos(cim.gamma_com) - drag_cache * sin(cim.gamma_com)),
                             G;
    cim.augmented_regressor_lon_out << cim.F_baseline_lon_out,
                                       cim.regressor_lon_out;

    // Cache the transpose of the tracking error * P * B
    Eigen::Matrix<double, 4, 1> e_total;
    e_total << cim.e_lon_out_pos, cim.e_lon_out_vel;
    auto e_transpose_p_b = e_total.transpose() * cip.P_long_ol * cip.B_lon_ol;     
    
    // Computing the scalar value output from the dead-zone modification modulation function 
    cim.dead_zone_value_lon_out = ::_shared_::_deadzone_operator_::deadZoneModulationFunction(e_total.transpose(),
                                                                                              cip.dead_zone_delta_lon_out,
                                                                                              cip.dead_zone_e0_lon_out);

    // Adaptive laws
    cim.K_hat_x_lon_out_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_x_lon_ol,
                                                                                   cim.dead_zone_value_lon_out,
                                                                                   cim.x_lon_out,
                                                                                   e_transpose_p_b,
                                                                                   cip.sigma_x_lon_out,
                                                                                   csm.K_hat_x_lon_out);

    cim.K_hat_r_lon_out_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_r_lon_ol,
                                                                                   cim.dead_zone_value_lon_out,
                                                                                   cim.r_cmd_lon_out,
                                                                                   e_transpose_p_b,
                                                                                   cip.sigma_r_lon_out,
                                                                                   csm.K_hat_r_lon_out);
                                                                                   
    cim.Theta_hat_lon_out_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(cip.Gamma_Theta_lon_ol,
                                                                                     cim.dead_zone_value_lon_out,
                                                                                     cim.augmented_regressor_lon_out,
                                                                                     e_transpose_p_b,
                                                                                     cip.sigma_Theta_lon_out,
                                                                                     csm.Theta_hat_lon_out);

    // Projection operator - Ball
    if (cip.use_projection_operator_lon_out)
    {
        // Projection operator K_hat_x_lon_out
        ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_x_lon_out)> proj_op_output_K_hat_x_lon_out = 
            ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_x_lon_out,
                                                                        cim.K_hat_x_lon_out_dot,
                                                                        cip.projection_x_max_x_lon_out,
                                                                        cip.projection_epsilon_x_lon_out);

        cim.K_hat_x_lon_out_dot = proj_op_output_K_hat_x_lon_out.projected_matrix;
        cim.proj_op_activated_K_hat_x_lon_out = proj_op_output_K_hat_x_lon_out.projection_operator_activated;
        
        // Projection operator K_hat_r_lon_out
        ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_r_lon_out)> proj_op_output_K_hat_r_lon_out = 
            ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_r_lon_out,
                                                                        cim.K_hat_r_lon_out_dot,
                                                                        cip.projection_x_max_r_lon_out,
                                                                        cip.projection_epsilon_r_lon_out);

        cim.K_hat_r_lon_out_dot = proj_op_output_K_hat_r_lon_out.projected_matrix;
        cim.proj_op_activated_K_hat_r_lon_out = proj_op_output_K_hat_r_lon_out.projection_operator_activated;   
        
        // Projection operator Theta_hat_lon_out
        ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.Theta_hat_lon_out)> proj_op_output_Theta_hat_lon_out = 
            ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.Theta_hat_lon_out,
                                                                        cim.Theta_hat_lon_out_dot,
                                                                        cip.projection_x_max_Theta_lon_out,
                                                                        cip.projection_epsilon_Theta_lon_out);

        cim.Theta_hat_lon_out_dot = proj_op_output_Theta_hat_lon_out.projected_matrix;
        cim.proj_op_activated_Theta_hat_lon_out = proj_op_output_Theta_hat_lon_out.projection_operator_activated;                                                                        
    }

    // Adaptive control law
    cim.F_adaptive_lon_out << csm.K_hat_x_lon_out.transpose() * cim.x_lon_out
                            + csm.K_hat_r_lon_out.transpose() * cim.r_cmd_lon_out
                            - csm.Theta_hat_lon_out.transpose() * cim.augmented_regressor_lon_out;

    // Compute the total baseline + adaptive control input
    cim.F_lon_out << cim.F_baseline_lon_out + cim.F_adaptive_lon_out;                            
}

// Function to compute the theta_cmd for the longitudinal controller's inner loop
void mrac_long_lat::compute_theta_cmd_longitudinal()
{
    // Cache the forces needed to compute \theta_cmd
    double fx = cim.F_lon_out(0);
    double fz = cim.F_lon_out(1);
    double mg = MASS * G;
    double ex = cip.epsilon_x_lon_ol;
    double ez = cip.epsilon_z_lon_ol;

    // Function block to find the \theta_cmd such that we do not point the nose down
    if ((abs(fx) > ex) && (-fz + mg > ez))
    {
        cim.theta_cmd = atan2(fz, fx);
        cim.theta_cmd_regime = 1;
    }
    else if((abs(fx) > ex) && (-fz + mg <= -ez))
    {
        cim.theta_cmd = atan2(abs(fz), fx);
        cim.theta_cmd_regime = 2;
    }
    else if ((abs(fx) > ex) && (abs(mg - fz) < ez))
    {
        cim.theta_cmd = atan2(abs(fz), fx);
        cim.theta_cmd_regime = 3;
    }
    else if ((abs(fx) <= ex) && (-fz + mg >= ez))
    {
        cim.theta_cmd = atan2(abs(fz), fx);
        cim.theta_cmd_regime = 4;
    }
    else if ((abs(fx) <= ex) && (-mg + fz > ez))
    {
        cim.theta_cmd = PIHALF;
        cim.theta_cmd_regime = 5;
    }
    else if ((abs(fx) <= ex) && (abs(fz - mg) <= ez)) 
    {
        cim.theta_cmd = PIHALF;
        cim.theta_cmd_regime = 6;
    }

    // Compute the internal state for desired pitch rate
    cim.internal_state_theta_cmd_filter << A_filter_pitch_ref * csm.state_theta_cmd_filter
                                         + B_filter_pitch_ref * cim.theta_cmd;

    // Compute the desired pitch rate
    cim.theta_cmd_dot = C_filter_pitch_ref * csm.state_theta_cmd_filter;

    // Compute the internal state for the desired pitch acceleration
    cim.internal_state_theta_cmd_d_filter << A_filter_pitch_dot_ref * csm.state_theta_cmd_d_filter
                                           + B_filter_pitch_dot_ref * cim.theta_cmd_dot;

    // Compute the desired pitch acceleration
    cim.theta_cmd_ddot = C_filter_pitch_dot_ref * csm.state_theta_cmd_d_filter;  
}

// Function to comptue the innerloop of the longitudnal controller
void mrac_long_lat::compute_innerloop_longitudinal()
{
    // Load the state vector [\theta, \dot{\theta}]
    cim.x_lon_in << cim.pitch, dim.x_hat_2l_mrad_dot(4);

    // Cache the I_y value
    double Iy = inertia_matrix_b(1,1);

    // Compute the error
    cim.e_lon_in_theta << csm.x_ref_lon_in(0) - cim.x_lon_in(0);
    cim.e_lon_in_theta_dot << csm.x_ref_lon_in(0) - cim.x_lon_in(1);

    // Compute the longitudinal orientation error between the reference model and the user defined trajectory
    cim.e_ref_lon_in << csm.x_ref_lon_in(0) - cim.theta_cmd;

    // Compute the reference command input [reference model - user_defined_trajectory]
    cim.r_cmd_lon_in = Iy * (- cip.Ki_cmd_lon_il * csm.e_ref_lon_in_I(0,0)   // Integral term
                             + cip.Kp_cmd_lon_il * cim.theta_cmd             // Proportional term
                             + cip.Kd_cmd_lon_il * cim.theta_cmd_dot         // Derivative term
                             + cim.theta_cmd_ddot);                          // Feedforward term

    // Compute the reference model
    cim.x_ref_lon_in_dot << cip.A_ref_lon_il * csm.x_ref_lon_in
                          + cip.B_ref_lon_il * cim.r_cmd_lon_in;

    // Compute the aerodynamic inversion term - NEEDS CHECKING
    auto lift_cache = DYN_PRESS_COEFF_W * cim.v_norm * cim.v_norm * (ComputeCL(cim.alpha_up) + ComputeCL(cim.alpha_lw));
    auto drag_cache = DYN_PRESS_COEFF_W * cim.v_norm * cim.v_norm * (ComputeCD(cim.alpha_up) + ComputeCD(cim.alpha_lw));
    auto moment_cache = DYN_PRESS_COEFF_W * cim.v_norm * cim.v_norm * LX * (ComputeCM(cim.alpha_up) + ComputeCM(cim.alpha_lw));
    cim.M_inv_lon_in << moment_cache - 2 * LZ * (drag_cache * cos(cim.alpha_com) + lift_cache * sin(cim.alpha_com));

    // Compute the baseline control input
    cim.M_baseline_lon_in << Iy * (- cip.Kp_lon_il * cim.e_lon_in_theta(0,0)    // Proportional term
                                  - cip.Kd_lon_il * cim.e_lon_in_theta_dot(0,0) // Derivative term
                                  - cip.Ki_lon_il * csm.e_lon_in_I(0,0)         // Integral term
                                  + cim.x_ref_lon_in_dot(1)                     // Feedforward term
                                  + cim.M_inv_lon_in);                          // Aerodynamic inversion term

    // Compute the augmented regressor vector
    cim.regressor_lon_in << cim.v_norm * cim.v_norm * ( (ComputeCL(cim.alpha_up) + ComputeCL(cim.alpha_lw)) 
                                                        * sin(cim.alpha_com) 
                                                        + (ComputeCD(cim.alpha_up) + ComputeCD(cim.alpha_lw)) 
                                                        * cos(cim.alpha_com) ),
                            cim.v_norm * cim.v_norm * (ComputeCM(cim.alpha_up) + ComputeCM(cim.alpha_lw));
    cim.augmented_regressor_lon_in << cim.M_baseline_lon_in,
                                      cim.regressor_lon_in;                            

    // Cache the tranpose of the tracking error * P * B
    Eigen::Matrix<double, 2, 1> e_total;
    e_total << cim.e_lon_in_theta, cim.e_lon_in_theta_dot;
    auto e_transpose_p_b = e_total.transpose() * cip.P_long_il * cip.B_lon_il;

    // Computing the scalar value output from the dead-zone modification modulation function
    cim.dead_zone_value_lon_in = ::_shared_::_deadzone_operator_::deadZoneModulationFunction(e_total.transpose(),
                                                                                             cip.dead_zone_delta_lon_il,
                                                                                             cip.dead_zone_e0_lon_il);
    // Adaptive laws
    cim.K_hat_x_lon_in_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_x_lon_il,
                                                                                   cim.dead_zone_value_lon_in,
                                                                                   cim.x_lon_in,
                                                                                   e_transpose_p_b,
                                                                                   cip.sigma_x_lon_il,
                                                                                   csm.K_hat_x_lon_in);

    cim.K_hat_r_lon_in_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_r_lon_il,
                                                                                   cim.dead_zone_value_lon_in,
                                                                                   cim.r_cmd_lon_in,
                                                                                   e_transpose_p_b,
                                                                                   cip.sigma_r_lon_il,
                                                                                   csm.K_hat_r_lon_in);

    cim.Theta_hat_lon_in_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(cip.Gamma_Theta_lon_il,
                                                                                    cim.dead_zone_value_lon_in,
                                                                                    cim.augmented_regressor_lon_in,
                                                                                    e_transpose_p_b,
                                                                                    cip.sigma_Theta_lon_il,
                                                                                    csm.Theta_hat_lon_in);


    // Projection operator - Ball
    if (cip.use_projection_operator_lon_il)
    {
        // Projection operator K_hat_x_lon_in
        ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_x_lon_in)> proj_op_output_K_hat_x_lon_in = 
            ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_x_lon_in,
                                                                        cim.K_hat_x_lon_in_dot,
                                                                        cip.projection_x_max_x_lon_il,
                                                                        cip.projection_epsilon_x_lon_il);

        cim.K_hat_x_lon_in_dot = proj_op_output_K_hat_x_lon_in.projected_matrix;
        cim.proj_op_activated_K_hat_x_lon_in = proj_op_output_K_hat_x_lon_in.projection_operator_activated;
        
        // Projection operator K_hat_r_lon_in
        ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_r_lon_in)> proj_op_output_K_hat_r_lon_in = 
            ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_r_lon_in,
                                                                        cim.K_hat_r_lon_in_dot,
                                                                        cip.projection_x_max_r_lon_il,
                                                                        cip.projection_epsilon_r_lon_il);

        cim.K_hat_r_lon_in_dot = proj_op_output_K_hat_r_lon_in.projected_matrix;
        cim.proj_op_activated_K_hat_r_lon_in = proj_op_output_K_hat_r_lon_in.projection_operator_activated;   
        
        // Projection operator Theta_hat_lon_in
        ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.Theta_hat_lon_in)> proj_op_output_Theta_hat_lon_in = 
            ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.Theta_hat_lon_in,
                                                                        cim.Theta_hat_lon_in_dot,
                                                                        cip.projection_x_max_Theta_lon_il,
                                                                        cip.projection_epsilon_Theta_lon_il);

        cim.Theta_hat_lon_in_dot = proj_op_output_Theta_hat_lon_in.projected_matrix;
        cim.proj_op_activated_Theta_hat_lon_in = proj_op_output_Theta_hat_lon_in.projection_operator_activated;                                                                        
    }

    // Adaptive control law
    cim.M_adaptive_lon_in << csm.K_hat_x_lon_in.transpose() * cim.x_lon_in
                           + csm.K_hat_r_lon_in.transpose() * cim.r_cmd_lon_in
                           - csm.Theta_hat_lon_in.transpose() * cim.augmented_regressor_lon_in;

    // Compute the total baseline + adaptive control input
    cim.M_lon_in << cim.M_baseline_lon_in + cim.M_adaptive_lon_in;
    
}

// Function to compute the saturated thrust for the longitudinal controller
void mrac_long_lat::compute_T_hat()
{
    // Compute the weight inverstion term to be added to Fz
    double weight_inversion_term = - MASS * G;

    // Cache the forces
    double fx = cim.F_lon_out(0);
    double fz = cim.F_lon_out(1) + weight_inversion_term;

    // Cache the theta_max in radians
    double theta_max = ::_shared_::_conversions_::deg2rad(cip.theta_max);

    // Compute the total thrust command
    if ( (abs(cim.theta_cmd - cim.x_lon_in(0)) < theta_max) && (fz <= 0.0) ) {
        cim.T_hat = (sqrt(fx * fx + fz * fz)) / (cos(cim.theta_cmd - cim.x_lon_in(0)));
        cim.T_hat_regime = 1;
    }
    else if ( (abs(cim.theta_cmd - cim.x_lon_in(0)) < theta_max) && (fz > 0.0) ) {
        cim.T_hat = ( abs(fx) ) / (cos(cim.theta_cmd - cim.x_lon_in(0)));
        cim.T_hat_regime = 2;
    }
    else
    {
        cim.T_hat = sqrt( fx * fx + fz * fz );
        cim.T_hat_regime = 3;
    }

}

// Function to compute the T_upper and T_lower values based on My and T_hat
void mrac_long_lat::compute_T_upper_T_lower()
{

    // Cache the necessary values
    double My = cim.M_lon_in(0);
    double theta_max = ::_shared_::_conversions_::deg2rad(cip.theta_max);
    double theta_diff = cim.theta_cmd - cim.x_lon_in(0);
    double T_hat = cim.T_hat;
    double T_hat_2 = cim.T_hat / 2.0;
    double My_lz = My / LX;
    double My_lz_2 = My / (2.0 * LX);

    // Compute T_upper
    if ( (T_hat < My_lz) && (My > 0) ) {
        cim.T_upper = 0.0;
        cim.T_upper_regime = 1;
    }
    else if ( (T_hat < - My_lz) && (My < 0) ) {
        cim.T_upper = -My_lz;
        cim.T_upper_regime = 2;
    }
    else if ( (T_hat >= abs(My_lz)) && (abs(theta_diff) <= theta_max) ) {
        cim.T_upper = T_hat_2 - My_lz_2;
        cim.T_upper_regime = 3;
    }
    else if ( (T_hat >= abs(My_lz)) && (theta_diff > theta_max) ) {
        cim.T_upper = 0.0;
        cim.T_upper_regime = 4;
    }
    else if ( (T_hat >= abs(My_lz)) && (theta_diff < -theta_max) ) {
        cim.T_upper = abs(My_lz);
        cim.T_upper_regime = 5;
    }
    
    // Compute T_lower
    if ( (T_hat < My_lz) && (My > 0) ) {
        cim.T_lower = My_lz;
        cim.T_lower_regime = 1;
    }
    else if ( (T_hat < - My_lz) && (My < 0) ) {
        cim.T_lower = 0.0;
        cim.T_lower_regime = 2;
    }
    else if ( (T_hat >= abs(My_lz)) && (abs(theta_diff) <= theta_max) ) {
        cim.T_lower = T_hat_2 + My_lz_2;
        cim.T_lower_regime = 3;
    }
    else if ( (T_hat >= abs(My_lz)) && (theta_diff > theta_max) ) {
        cim.T_lower = abs(My_lz);
        cim.T_lower_regime = 4;
    }
    else if ( (T_hat >= abs(My_lz)) && (theta_diff < -theta_max) ) {
        cim.T_lower = 0.0;
        cim.T_lower_regime = 5;
    }

}

// Function to compute the outerloop of the lateral controller
void mrac_long_lat::compute_outerloop_lateral()
{
    // Load the state vector 
    cim.x_lat_out << cim.y, cim.vy;

    // Compute the error 
    cim.e_lat_out_pos << csm.x_ref_lat_out(0) - cim.x_lat_out(0);
    cim.e_lat_out_vel << csm.x_ref_lat_out(1) - cim.x_lat_out(1); 

    // Compute the lateral position error between the reference model and the user defined trajectory
    cim.e_ref_lat_out << csm.x_ref_lat_out(0) - cim.r_user(1);

    // Compute the reference command input [reference model - user_defined_trajectory]
    cim.r_cmd_lat_out = MASS * (- cip.Ki_cmd_lat_ol * csm.e_ref_lat_out_I(0,0)  // Integral term
                                 + cip.Kp_cmd_lat_ol * cim.r_user(1)            // Proportional term
                                 + cip.Kd_cmd_lat_ol * cim.v_user(1)            // Derivative term
                                 + cim.a_user(1));                              // Feedforward term

    // Compute the reference model
    cim.x_ref_lat_out_dot << cip.A_ref_lat_ol * csm.x_ref_lat_out
                           + cip.B_ref_lat_ol * cim.r_cmd_lat_out;     
                           
    // Compute the aerodynamic inversion term
    auto lift_cahce = DYN_PRESS_COEFF_W * cim.v_norm * cim.v_norm * (ComputeCL(cim.alpha_up) + ComputeCL(cim.alpha_lw));
    auto drag_cache = DYN_PRESS_COEFF_W * cim.v_norm * cim.v_norm * (ComputeCD(cim.alpha_up) + ComputeCD(cim.alpha_lw));
    cim.F_inv_lat_out = - drag_cache * (cim.R_J_I(1,0) * cos(cim.alpha_com) 
                                      + cim.R_J_I(1,2) * sin(cim.alpha_com))
                        - lift_cahce * (cim.R_J_I(1,0) * sin(cim.alpha_com) 
                                      - cim.R_J_I(1,2) * cos(cim.alpha_com));

    // Compute the baseline control input
    cim.F_baseline_lat_out = MASS * (- cip.Kp_lat_ol * cim.e_lat_out_pos(0,0)    // Proportional term
                                     - cip.Kd_lat_ol * cim.e_lat_out_vel(0,0)    // Derivative term
                                     - cip.Ki_lat_ol * csm.e_lat_out_I(0,0)      // Integral term
                                     + cim.x_ref_lat_out_dot(1)                  // Feedforward term
                                     + cim.F_inv_lat_out);                       // Aerodynamic inversion term

    // Compute the augmented regressor vector
    cim.regressor_lat_out = cim.F_inv_lat_out / DYN_PRESS_COEFF_W;
    cim.augmented_regressor_lat_out << cim.F_baseline_lat_out,
                                       cim.regressor_lat_out;

    // Cache the transpose of the tracking error * P * B
    Eigen::Matrix<double, 2, 1> e_total;
    e_total << cim.e_lat_out_pos, cim.e_lat_out_vel;
    auto e_transpose_p_b = e_total.transpose() * cip.P_lat_ol * cip.B_lat_ol;

    // Computing the scalar value output from the dead-zone modification modulation function
    cim.dead_zone_value_lat_out = ::_shared_::_deadzone_operator_::deadZoneModulationFunction(e_total.transpose(),
                                                                                              cip.dead_zone_delta_lat_ol,
                                                                                              cip.dead_zone_e0_lat_ol);

    // Adaptive laws
    cim.K_hat_x_lat_out_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_x_lat_ol,
                                                                                   cim.dead_zone_value_lat_out,
                                                                                   cim.x_lat_out,
                                                                                   e_transpose_p_b,
                                                                                   cip.sigma_x_lat_ol,
                                                                                   csm.K_hat_x_lat_out);

    cim.K_hat_r_lat_out_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_r_lat_ol,
                                                                                   cim.dead_zone_value_lat_out,
                                                                                   cim.r_cmd_lat_out,
                                                                                   e_transpose_p_b,
                                                                                   cip.sigma_r_lat_ol,
                                                                                   csm.K_hat_r_lat_out);
                                                                                   
    cim.Theta_hat_lat_out_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(cip.Gamma_Theta_lat_ol,
                                                                                     cim.dead_zone_value_lat_out,
                                                                                     cim.augmented_regressor_lat_out,
                                                                                     e_transpose_p_b,
                                                                                     cip.sigma_Theta_lat_ol,
                                                                                     csm.Theta_hat_lat_out);

    // Projection operator - Ball
    if (cip.use_projection_operator_lat_ol)
    {
        // Projection operator K_hat_x_lat_out
        ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_x_lat_out)> proj_op_output_K_hat_x_lat_out = 
            ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_x_lat_out,
                                                                        cim.K_hat_x_lat_out_dot,
                                                                        cip.projection_x_max_x_lat_ol,
                                                                        cip.projection_epsilon_x_lat_ol);

        cim.K_hat_x_lat_out_dot = proj_op_output_K_hat_x_lat_out.projected_matrix;
        cim.proj_op_activated_K_hat_x_lat_out = proj_op_output_K_hat_x_lat_out.projection_operator_activated;
        
        // Projection operator K_hat_r_lat_out
        ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_r_lat_out)> proj_op_output_K_hat_r_lat_out = 
            ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_r_lat_out,
                                                                        cim.K_hat_r_lat_out_dot,
                                                                        cip.projection_x_max_r_lat_ol,
                                                                        cip.projection_epsilon_r_lat_ol);

        cim.K_hat_r_lat_out_dot = proj_op_output_K_hat_r_lat_out.projected_matrix;
        cim.proj_op_activated_K_hat_r_lat_out = proj_op_output_K_hat_r_lat_out.projection_operator_activated;   
        
        // Projection operator Theta_hat_lat_out
        ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.Theta_hat_lat_out)> proj_op_output_Theta_hat_lat_out = 
            ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.Theta_hat_lat_out,
                                                                        cim.Theta_hat_lat_out_dot,
                                                                        cip.projection_x_max_Theta_lat_ol,
                                                                        cip.projection_epsilon_Theta_lat_ol);

        cim.Theta_hat_lat_out_dot = proj_op_output_Theta_hat_lat_out.projected_matrix;
        cim.proj_op_activated_Theta_hat_lat_out = proj_op_output_Theta_hat_lat_out.projection_operator_activated;                                                                        
    }

     // Adaptive control law
    cim.F_adaptive_lat_out << csm.K_hat_x_lat_out.transpose() * cim.x_lat_out
                            + csm.K_hat_r_lat_out.transpose() * cim.r_cmd_lat_out
                            - csm.Theta_hat_lat_out.transpose() * cim.augmented_regressor_lat_out;

    // Compute the total baseline + adaptive control input
    cim.F_lat_out = cim.F_baseline_lat_out + cim.F_adaptive_lat_out(0,0);   
}

// Function to compute the roll and pitch commands for the lateral controller's inner loop
void mrac_long_lat::compute_roll_pitch_cmd_lateral()
{
    // NOTE: We are using the roll filter params for yaw and yaw filter params for roll, this is because
    //       the filters were tuned for the NED frame of reference, and roll and yaw essentially "flip" in
    //       the biplane frame of reference.

    // Load up the Force vector
    cim.Hat_F << cim.F_lon_out(0), cim.F_lat_out, cim.F_lon_out(1);
    
    // Compute the forces in the body frame
    cim.Tilde_F = cim.R_I_J * cim.Hat_F;

    // Cache the values
    double tilde_fy = cim.Tilde_F(1);
    double tilde_T = cim.Tilde_F.norm();
    double r1 = tilde_fy / tilde_T;

    // Compute psi_cmd
    cim.psi_cmd = atan2(r1, sqrt(1 - r1*r1));

    // Compute phi_cmd - We always want to keep the body roll at the origin
    cim.phi_cmd = 0.0;

    // Compute the internal state for yaw rate
    cim.internal_state_psi_cmd_filter << A_filter_roll_ref * csm.state_psi_cmd_filter
                                       + B_filter_roll_ref * cim.psi_cmd;

    // Compute the desired yaw rate
    cim.phi_cmd_dot = C_filter_roll_ref * csm.state_psi_cmd_filter;               
    
    // Compute the desired roll rate
    cim.phi_cmd_dot = 0.0;

    // Compute the desired p_cmd and r_cmd
    Eigen::Matrix<double, 2, 1> p_cmd_r_cmd_vector;
    Eigen::Matrix<double, 2, 2> reduced_J_matrix;
    Eigen::Matrix<double, 2, 1> phi_dot_cmd_psi_dot_cmd_vector;
    Eigen::Matrix<double, 2, 1> nonlinearity_vector;

    phi_dot_cmd_psi_dot_cmd_vector << cim.phi_cmd_dot,  cim.psi_cmd_dot;
    nonlinearity_vector << 0, sin(cim.roll) * cim.x_lon_in(1);
    reduced_J_matrix << 1.0,                -sin(cim.pitch),
                        0.0, cos(cim.pitch) * cos(cim.roll);
    p_cmd_r_cmd_vector << reduced_J_matrix * phi_dot_cmd_psi_dot_cmd_vector - nonlinearity_vector;  
    
    // Assign p_cmd and r_cmd
    cim.wx_cmd = p_cmd_r_cmd_vector(0);
    cim.wz_cmd = p_cmd_r_cmd_vector(1);

    // Compute the internal state for \dot{p}_cmd an \dot{r}_cmd
    cim.internal_state_wx_cmd_filter << A_filter_yaw_dot_ref * csm.state_wx_cmd_filter
                                      + B_filter_yaw_dot_ref * cim.wx_cmd;

    cim.internal_state_wz_cmd_filter << A_filter_roll_dot_ref * csm.state_wz_cmd_filter
                                      + B_filter_roll_dot_ref * cim.wz_cmd;
                                      
    // Compute the desired angular acceleration
    cim.wx_cmd_dot = C_filter_yaw_dot_ref * csm.state_wx_cmd_filter;
    cim.wz_cmd_dot = C_filter_roll_dot_ref * csm.state_wz_cmd_filter;

}

// Function to compute the innerloop of the lateral controller
void mrac_long_lat::compute_innerloop_lateral()
{
    // Load the state vector
    cim.x_lat_in << cim.w_x, cim.w_z;

    // Compute the error in orientation [actual - desired]
    cim.e_lat_in_orientation(0) = cim.roll - cim.phi_cmd;
    cim.e_lat_in_orientation(1) = cim.yaw - cim.psi_cmd;

    // Compute the error in angular velocity
    cim.e_lat_in_angular_vel << cim.x_lat_in - csm.x_ref_lat_in;

    // Compute the error between the angular velocity reference model and the desired angular velocity
    cim.e_ref_lat_in(0) = csm.x_ref_lat_in(0) - cim.wx_cmd;
    cim.e_ref_lat_in(0) = csm.x_ref_lat_in(1) - cim.wz_cmd;

    // Compute the reference command input [reference model - user_defined_trajectory]
    Eigen::Matrix<double, 2, 1> w_cmd;
    Eigen::Matrix<double, 2, 1> w_cmd_dot;
    w_cmd << cim.wx_cmd, cim.wz_cmd;
    w_cmd_dot << cim.wx_cmd_dot, cim.wz_cmd_dot;
    cim.r_cmd_lat_in << cip.Kp_cmd_lat_il * w_cmd - cip.Ki_cmd_lat_il * csm.e_ref_lat_in_I + w_cmd_dot;

    // Compute the reference model
    cim.x_ref_lat_in_dot << cip.A_ref_lat_il * csm.x_ref_lat_in + cip.B_ref_lat_il * cim.r_cmd_lat_in;

    // Baseline Control input
    Eigen::Matrix<double, 2, 2> inertia_matrix;
    inertia_matrix(0,0) = inertia_matrix_b(0,0);
    inertia_matrix(1,1) = inertia_matrix_b(2,2);
    inertia_matrix(0,1) = 0.0;
    inertia_matrix(1,0) = 0.0;
    cim.M_baseline_lat_in << inertia_matrix * (- cip.Kp_lat_il * cim.e_lat_in_orientation       // Proportional term
                                               - cip.Kd_lat_il * cim.e_lat_in_angular_vel       // Derivative term
                                               - cip.Ki_lat_il * csm.e_lat_in_orientation_I     // Integral term
                                               + cim.x_ref_lat_in_dot);

    // Compute the augmented regressor vector    
    cim.regressor_lat_in << cim.w_x * cim.w_y,
                            cim.w_z * cim.w_y;
    cim.augmented_regressor_lat_in << cim.M_baseline_lat_in,
                                      cim.regressor_lat_in;
                                      
    // Cache the transpose of the tracking error * P * B
    auto e_transpose_p_b = cim.e_lat_in_angular_vel.transpose() * cip.P_lat_il * cip.B_lat_il;
    
    // Computing the scalar value output from the dead-zone modification modulation function
    cim.dead_zone_value_lat_in = ::_shared_::_deadzone_operator_::deadZoneModulationFunction(cim.e_lat_in_angular_vel.transpose(),
                                                                                             cip.dead_zone_delta_lat_il,
                                                                                             cip.dead_zone_e0_lat_il);

    // Adaptive laws
    cim.K_hat_x_lat_in_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_x_lat_il,
                                                                                   cim.dead_zone_value_lat_in,
                                                                                   cim.x_lat_in,
                                                                                   e_transpose_p_b,
                                                                                   cip.sigma_x_lat_il,
                                                                                   csm.K_hat_x_lat_in);

    cim.K_hat_r_lat_in_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(-cip.Gamma_r_lat_il,
                                                                                   cim.dead_zone_value_lat_in,
                                                                                   cim.r_cmd_lat_in,
                                                                                   e_transpose_p_b,
                                                                                   cip.sigma_r_lat_il,
                                                                                   csm.K_hat_r_lat_in);

    cim.Theta_hat_lat_in_dot = ::_shared_::_adaptive_laws_::AdaptiveLawDeadZoneEMod(cip.Gamma_Theta_lat_il,
                                                                                    cim.dead_zone_value_lat_in,
                                                                                    cim.augmented_regressor_lat_in,
                                                                                    e_transpose_p_b,
                                                                                    cip.sigma_Theta_lat_il,
                                                                                    csm.Theta_hat_lat_in);

    // Projection operator - Ball
    if (cip.use_projection_operator_lat_il)
    {
        // Projection operator K_hat_x_lat_in
        ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_x_lat_in)> proj_op_output_K_hat_x_lat_in = 
            ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_x_lat_in,
                                                                        cim.K_hat_x_lat_in_dot,
                                                                        cip.projection_x_max_x_lat_il,
                                                                        cip.projection_epsilon_x_lat_il);

        cim.K_hat_x_lat_in_dot = proj_op_output_K_hat_x_lat_in.projected_matrix;
        cim.proj_op_activated_K_hat_x_lat_in = proj_op_output_K_hat_x_lat_in.projection_operator_activated;
        
        // Projection operator K_hat_r_lat_in
        ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.K_hat_r_lat_in)> proj_op_output_K_hat_r_lat_in = 
            ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.K_hat_r_lat_in,
                                                                        cim.K_hat_r_lat_in_dot,
                                                                        cip.projection_x_max_r_lat_il,
                                                                        cip.projection_epsilon_r_lat_il);

        cim.K_hat_r_lat_in_dot = proj_op_output_K_hat_r_lat_in.projected_matrix;
        cim.proj_op_activated_K_hat_r_lat_in = proj_op_output_K_hat_r_lat_in.projection_operator_activated;   
        
        // Projection operator Theta_hat_lon_in
        ::_shared_::_projection_operator_::MatrixProjectionOutput<decltype(csm.Theta_hat_lat_in)> proj_op_output_Theta_hat_lat_in = 
            ::_shared_::_projection_operator_::_ball_::projectionMatrix(csm.Theta_hat_lat_in,
                                                                        cim.Theta_hat_lat_in_dot,
                                                                        cip.projection_x_max_Theta_lat_il,
                                                                        cip.projection_epsilon_Theta_lat_il);

        cim.Theta_hat_lat_in_dot = proj_op_output_Theta_hat_lat_in.projected_matrix;
        cim.proj_op_activated_Theta_hat_lat_in = proj_op_output_Theta_hat_lat_in.projection_operator_activated;   
    }

    // Adaptive control law
    cim.M_adaptive_lat_in << csm.K_hat_x_lat_in.transpose() * cim.x_lat_in
                           + csm.K_hat_r_lat_in.transpose() * cim.r_cmd_lat_in
                           - csm.Theta_hat_lat_in.transpose() * cim.augmented_regressor_lat_in;

    // Compute the total baseline + adaptive control input
    cim.M_lat_in << cim.M_baseline_lat_in + cim.M_adaptive_lat_in;
}

// Function to compute the normalied thrusts
void mrac_long_lat::compute_normalized_thrusts()
{
    // Set the control forces directly to com
    this->com_u << cim.T_hat, cim.M_lat_in(1), cim.M_lon_in(0), cim.M_lat_in(0);

    // Cache the mixer matrix
    Eigen::Matrix<double, 4, 4> mixer_matrix;
    
    mixer_matrix(0, 0) = 0.0;
    mixer_matrix(0, 1) = 0.5;
    mixer_matrix(0, 2) = 1.0 / (4.0 * CT_MOTOR);
    mixer_matrix(0, 3) = - 1.0 / (4.0 * LY);

    mixer_matrix(1, 0) = 0.5;
    mixer_matrix(1, 1) = 0.0;
    mixer_matrix(1, 2) = 1.0 / (4.0 * CT_MOTOR);
    mixer_matrix(1, 3) = 1.0 / (4.0 * LY);

    mixer_matrix(2, 0) = 0.0;
    mixer_matrix(2, 1) = 0.5;
    mixer_matrix(2, 2) = - 1.0 / (4.0 * CT_MOTOR);
    mixer_matrix(2, 3) = 1.0 / (4.0 * LY);

    mixer_matrix(3, 0) = 0.5;
    mixer_matrix(3, 1) = 0.0;
    mixer_matrix(3, 2) = - 1.0 / (4.0 * CT_MOTOR);
    mixer_matrix(3, 3) = - 1.0 / (4.0 * LY);

    // Cache the total control input vector u = [T_upper T_lower Mx Mz]^{\rm T}
    Eigen::Matrix<double, 4, 1> u;
    u << cim.T_upper, cim.T_lower, cim.M_lat_in(0), cim.M_lat_in(1);

    // Compute the individual thrusts in Newtons
    cim.Thrust << mixer_matrix * u;

    // Saturate each element of the Thrust vector between MIN_THRUST and MAX_THRUST
    cim.Sat_Thrust = (cim.Thrust.cwiseMin(MAX_THRUST).cwiseMax(MIN_THRUST));

    // Compute the final control inputs
    control_input(0) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(0));
    control_input(1) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(1));
    control_input(2) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(2));
    control_input(3) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, cim.Sat_Thrust(3));
}

// Function that is called in sim-bridge.cpp
void mrac_long_lat::run(const double time_step_rk4_)
{
    // 1. Differentiate the euler angle singals to get the rates
    differentiate_for_euler_rates();

    // 2. Compute the outerloop for the longitudinal controller
    compute_outerloop_longitudinal();

    // 3. Compute the innerloop commands for the longitudinal controller
    compute_theta_cmd_longitudinal();

    // 4. Compute the innerloop for the longitudinal controller
    compute_innerloop_longitudinal();

    // 5. Compute the total thrust T_hat
    compute_T_hat();

    // 6. Compute the longitudinal control inputs T_upper and T_lower
    compute_T_upper_T_lower();

    // 7. Compute the outerloop for the lateral controller
    compute_outerloop_lateral();

    // 8. Compute the innerloop commands for the lateral controller
    compute_roll_pitch_cmd_lateral();

    // 9. Compute the innerloop for the lateral controller
    compute_innerloop_lateral();

    // 10. Compute the normalized thrust - Final Step
    compute_normalized_thrusts();

    // 11. Do the integration
    rk54.do_step(boost::bind(&mrac_long_lat::model, this, bph::_1, bph::_2, bph::_3),
                 y, cim.t, time_step_rk4_);

    // Capture the time after the execution of the controller
    cim.alg_end_time = std::chrono::high_resolution_clock::now();
    
    // Calculate the duration of the execution 
    cim.alg_duration = std::chrono::duration_cast<std::chrono::microseconds>(
                            cim.alg_end_time - cim.alg_start_time).count();

    // 11. Log the Data after all the calculataions
    this->LogData();
}


// Function that is called during the constructor.
bool mrac_long_lat::InitiateLogging()
{
    auto status = _logger_::_filesystem_::setupControllerLogging(this->m_logger, "qrbp" ,"MRAC_LONG_LAT");
    return status;
}

// Function that sets up the headers for the log file
void mrac_long_lat::ConfigureHeaders()
{
    // Create the oss object
    std::ostringstream oss;

    oss << ", "
        << "Controller Time [s], "
        << "x_hat 2l mrad int phi, "
        << "x_hat 2l mrad int theta, "
        << "x_hat 2l mrad int psi, "
        << "x_hat 2l mrad phi, "
        << "x_hat 2l mrad theta, "
        << "x_hat 2l mrad psi, "
        << "x_hat 2l mrad dot phi, "
        << "x_hat 2l mrad dot theta, "
        << "x_hat 2l mrad dot psi, "
        << "x_hat 2l mrad dot phi dot, "
        << "x_hat 2l mrad dot theta dot, "
        << "x_hat 2l mrad dot psi dot, "
        << "x_ref_lon_out x, "
        << "x_ref_lon_out z, "
        << "x_ref_lon_out vx, "
        << "x_ref_lon_out vz, "
        << "x_ref_lon_in theta, "
        << "x_ref_lon_in theta_dot, "
        << "x_ref_lat_out y, "
        << "x_ref_lat_out vy, "
        << "x_ref_lat_in p, "
        << "x_ref_lat_in r, "
        << "e_ref_lon_out_I x, "
        << "e_ref_lon_out_I z, "
        << "e_ref_lon_in_I theta, "
        << "e_ref_lat_out_I y, "
        << "e_ref_lat_in_I p, "
        << "e_ref_lat_in_I r, "
        << "e_lon_out_I x, "
        << "e_lon_out_I z, "
        << "e_lon_in_I theta, "
        << "e_lat_out_I y, "
        << "e_lat_in_orientation_I phi, "
        << "e_lat_in_orientation_I psi, "
        << "x, "
        << "y, "
        << "z, "
        << "vx, "
        << "vy, "
        << "vz, "
        << "vx J, "
        << "vy J, "
        << "vz J, "
        << "v_norm, "
        << "alpha_up, "
        << "alpha_lw, "
        << "alpha_com, "
        << "gamma_com, "
        << "roll, "
        << "pitch, "
        << "yaw, "
        << "w_x, "
        << "w_y, "
        << "w_z, "
        << "user x, "
        << "user y, "
        << "user z, "
        << "user vx, "
        << "user vy, "
        << "user vz, "
        << "user ax, "
        << "user ay, "
        << "user az, "
        << "error x, "
        << "error z, "
        << "error vx, "
        << "error vz, "
        << "error theta, "
        << "error theta_dot, "
        << "error y, "
        << "error vy, "
        << "error phi, "
        << "error psi, "
        << "error w_x, "
        << "error w_z, "
        << "r_cmd_lon_out x, "
        << "r_cmd_lon_out z, "
        << "r_cmd_lon_out theta, "
        << "r_cmd_lon_out y, "
        << "r_cmd_lat_in p, "
        << "r_cmd_lat_in r, "
        << "Fx baseline, "
        << "Fz baseline, "
        << "Fx adaptive, "
        << "Fz adaptive, "
        << "Fx inversion, "
        << "Fz inversion, "
        << "Fx total, "
        << "Fz total, "
        << "My baseline, "
        << "My adaptive, "
        << "My inversion, "
        << "My total, "
        << "Fy baseline, "
        << "Fy adaptive, "
        << "Fy inversion, "
        << "Fy total, "
        << "Mx baseline, "
        << "Mz baseline, "
        << "Mx adaptive, "
        << "Mz adaptive, "
        << "Mx total, "
        << "Mz total, "
        ;

    
    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();

        _message_::SIMULATOR_INFO("[SIMCTL]: WROTE MRAC LONG LAT LOG HEADER DATA");
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE MRAC LONG LAT LOG HEADER DATA", e.what());
    }
}

// Function that writes the data corresponding to the headers to the log file.
void mrac_long_lat::LogData()
{
    // Log the data
    std::ostringstream oss;
    oss << ", "
        << cim.t << ", "
        << dsm.x_hat_2l_mrad(0) << ", "
        << dsm.x_hat_2l_mrad(1) << ", "
        << dsm.x_hat_2l_mrad(2) << ", "
        << dsm.x_hat_2l_mrad(3) << ", "
        << dsm.x_hat_2l_mrad(4) << ", "
        << dsm.x_hat_2l_mrad(5) << ", "
        << dim.x_hat_2l_mrad_dot(0) << ", "
        << dim.x_hat_2l_mrad_dot(1) << ", "
        << dim.x_hat_2l_mrad_dot(2) << ", "
        << dim.x_hat_2l_mrad_dot(3) << ", "
        << dim.x_hat_2l_mrad_dot(4) << ", "
        << dim.x_hat_2l_mrad_dot(5) << ", "
        << csm.x_ref_lon_out(0) << ", "
        << csm.x_ref_lon_out(1) << ", "
        << csm.x_ref_lon_out(2) << ", "
        << csm.x_ref_lon_out(3) << ", "
        << csm.x_ref_lon_in(0) << ", "
        << csm.x_ref_lon_in(1) << ", "
        << csm.x_ref_lat_out(0) << ", "
        << csm.x_ref_lat_out(1) << ", "
        << csm.x_ref_lat_in(0) << ", "
        << csm.x_ref_lat_in(1) << ", "
        << csm.e_ref_lon_out_I(0) << ", "
        << csm.e_ref_lon_out_I(1) << ", "
        << csm.e_ref_lon_in_I(0) << ", "
        << csm.e_ref_lat_out_I(0) << ", "
        << csm.e_ref_lat_in_I(0) << ", "
        << csm.e_ref_lat_in_I(1) << ", "
        << csm.e_lon_out_I(0) << ", "
        << csm.e_lon_out_I(1) << ", "
        << csm.e_lon_in_I(0) << ", "
        << csm.e_lat_out_I(0) << ", "
        << csm.e_lat_in_orientation_I(0) << ", "
        << csm.e_lat_in_orientation_I(1) << ", "
        << cim.x << ", "
        << cim.y << ", "
        << cim.z << ", "
        << cim.vx << ", "
        << cim.vy << ", "
        << cim.vz << ", "
        << cim.v_J(0) << ", "
        << cim.v_J(1) << ", "
        << cim.v_J(2) << ", "
        << cim.v_norm << ", "
        << cim.alpha_up << ", "
        << cim.alpha_lw << ", "
        << cim.alpha_com << ", "
        << cim.gamma_com << ", "
        << cim.roll << ", "
        << cim.pitch << ", "
        << cim.yaw << ", "
        << cim.w_x << ", "
        << cim.w_y << ", "
        << cim.w_z << ", "
        << cim.r_user(0) << ", "
        << cim.r_user(1) << ", "
        << cim.r_user(2) << ", "
        << cim.v_user(0) << ", "
        << cim.v_user(1) << ", "
        << cim.v_user(2) << ", "
        << cim.a_user(0) << ", "
        << cim.a_user(1) << ", "
        << cim.a_user(2) << ", "
        << cim.e_lon_out_pos(0) << ", "
        << cim.e_lon_out_pos(1) << ", "
        << cim.e_lon_out_vel(0) << ", "
        << cim.e_lon_out_vel(1) << ", "
        << cim.e_lon_in_theta(0) << ", "
        << cim.e_lon_in_theta_dot(0) << ", "
        << cim.e_lat_out_pos(0) << ", "
        << cim.e_lat_out_vel(0) << ", "
        << cim.e_lat_in_orientation(0) << ", "
        << cim.e_lat_in_orientation(1) << ", "
        << cim.e_lat_in_angular_vel(0) << ", "
        << cim.e_lat_in_angular_vel(1) << ", "
        << cim.r_cmd_lon_out(0) << ", "
        << cim.r_cmd_lon_out(1) << ", "
        << cim.r_cmd_lon_in << ", "
        << cim.r_cmd_lat_out << ", "
        << cim.r_cmd_lat_in(0) << ", "
        << cim.r_cmd_lat_in(1) << ", "
        << cim.F_baseline_lon_out(0) << ", "
        << cim.F_baseline_lon_out(1) << ", "
        << cim.F_adaptive_lon_out(0) << ", "
        << cim.F_adaptive_lon_out(1) << ", "
        << cim.F_inv_lon_out(0) << ", "
        << cim.F_inv_lon_out(1) << ", "
        << cim.F_lon_out(0) << ", "
        << cim.F_lon_out(1) << ", "
        << cim.M_baseline_lon_in(0) << ", "
        << cim.M_adaptive_lon_in(0) << ", "
        << cim.M_inv_lon_in(0) << ", "
        << cim.M_lon_in(0) << ", "
        << cim.F_baseline_lat_out << ", "
        << cim.F_adaptive_lat_out << ", "
        << cim.F_inv_lat_out << ", "
        << cim.F_lat_out << ", "
        << cim.M_baseline_lat_in(0) << ", "
        << cim.M_baseline_lat_in(1) << ", "
        << cim.M_adaptive_lat_in(0) << ", "
        << cim.M_adaptive_lat_in(1) << ", "
        << cim.M_lat_in(0) << ", "
        << cim.M_lat_in(1) << ", "
        ;

    try {
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "ControllerTag");

        BOOST_LOG(m_logger.GetControlLogger()) << oss.str();
    }
    catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMCTL]: FAILED TO WRITE MRAC LONG LAT LOG DATA", e.what());
    }
}


}   // namespace _mrac_long_lat_


}   // namespace _qrbp_

}   // namespace _acsl_