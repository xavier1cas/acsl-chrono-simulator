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

    // Initialize to zero the 4x2 matrix and set the bottom 2x2 bloack as follows
    ::_shared_::_initiate_::initMat(cip.B_ref_lon_ol);
    cip.B_ref_lon_ol.block<2, 2>(2, 0) = (1.0 / MASS) * Eigen::Matrix2d::Identity();

    // Solve the continuous Lyapunov equation to compute P_lon_ol
    cip.P_long_ol = ::_lyapunov_solver_::RealContinuousLyapunovEquation(cip.A_ref_lon_ol, cip.Q_lon_ol);

    // Initialize to zero the 4x2 matrix and set the bottom 2x2 bloack as follows
	::_shared_::_initiate_::initMat(cip.B_lon_ol);
	cip.B_lon_ol.block<2, 2>(2, 0) = Eigen::Matrix2d::Identity();

    // DISPLAY A DEBUG MESSAGE
    _message_::SIMULATOR_INFO("[SIMCTL]: INITIAL PARAMETERS COMPUTED FOR LONGITUDINAL OUTERLOOP");


    
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

    // 9. Compute the angular rates from angular velocity --------------------------------
    cim.omega_rot << cim.w_x, cim.w_y, cim.w_z;
    

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
    cim.r_cmd_lon_out << MASS * (-cip.Ki_cmd_lon_ol * csm.e_ref_lon_out_I   // Integral term
                                 + cip.Kp_cmd_lon_ol * cim.r_user_lon_out   // Proportional term
                                 + cip.Kd_cmd_lon_ol * cim.v_user_lon_out   // Derivative term
                                 + cim.a_user_lon_out);                     // Feedforward term

    // Compute the reference model
    cim.x_ref_lon_out_dot << cip.A_ref_lon_ol * csm.x_ref_lon_out
                           + cip.B_ref_lon_ol * cim.r_cmd_lon_out;          

    // Compute the aerodynamic inversion term
    auto lift_cache = (ComputeCL(cim.alpha_lw) + ComputeCL(cim.alpha_up));
    auto drag_cache = (ComputeCD(cim.alpha_lw) + ComputeCD(cim.alpha_up));
    cim.F_inv_lon_out(0) = - DYN_PRESS_COEFF_W * cim.v_norm * (lift_cache * sin(cim.gamma_com) + drag_cache * cos(cim.gamma_com));
    cim.F_inv_lon_out(1) = - DYN_PRESS_COEFF_W * cim.v_norm * (lift_cache * cos(cim.gamma_com) - drag_cache * sin(cim.gamma_com));
                           
    // Compute the baseline control input
    cim.F_baseline_lon_out << MASS * (- cip.Kp_lon_ol * cim.e_lon_out_pos        // Proportional term
                                      - cip.Kd_lon_ol * cim.e_lon_out_vel        // Derivative term
                                      - cip.Ki_lon_ol * csm.e_lon_out_I          // Integral term
                                      + cim.x_ref_lon_out_dot.tail<2>()          // Feedforward term
                                      + cim.F_inv_lon_out);                      // Aerodynamic inversion term

    // Compute the augmented regressor vector
    cim.regressor_lon_out << cim.v_norm * (lift_cache * sin(cim.gamma_com) + drag_cache * cos(cim.gamma_com)),
                             cim.v_norm * (lift_cache * cos(cim.gamma_com) - drag_cache * sin(cim.gamma_com)),
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
    
}

// Function to comptue the innerloop of the longitudnal controller
void mrac_long_lat::compute_innerloop_longitudinal()
{
    
}

// Function to compute the saturated thrust for the longitudinal controller
void mrac_long_lat::compute_T_hat()
{
    
}

// Function to compute the T_upper and T_lower values based on My and T_hat
void mrac_long_lat::compute_T_upper_T_lower()
{
    
}

// Function to compute the outerloop of the lateral controller
void mrac_long_lat::compute_outerloop_lateral()
{
    
}

// Function to compute the roll and pitch commands for the lateral controller's inner loop
void mrac_long_lat::compute_roll_pitch_cmd_lateral()
{
    
}

// Function to compute the innerloop of the lateral controller
void mrac_long_lat::compute_innerloop_lateral()
{
    
}

// Function to compute the normalied thrusts
void mrac_long_lat::compute_normalized_thrusts()
{
    
}

// Function that is called in sim-bridge.cpp
void mrac_long_lat::run(const double time_step_rk4_)
{
    // 1. Differentiate the euler angle singals to get the rates
    differentiate_for_euler_rates();

    // 2. Compute the outerloop for the longitudinal controller
    compute_outerloop_longitudinal();

    // 3. Do the integration
    rk54.do_step(boost::bind(&mrac_long_lat::model, this, bph::_1, bph::_2, bph::_3),
                 y, cim.t, time_step_rk4_);

    // PLACEHOLDER TO GIVE RANDOM INPUTS
    control_input(0) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, 0.0);
    control_input(1) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, 0.0);
    control_input(2) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, 0.0);
    control_input(3) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, 0.0);

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
        << "phi, "
        << "theta, "
        << "psi, "
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
        << cim.roll << ", "
        << cim.pitch << ", "
        << cim.yaw << ", "
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