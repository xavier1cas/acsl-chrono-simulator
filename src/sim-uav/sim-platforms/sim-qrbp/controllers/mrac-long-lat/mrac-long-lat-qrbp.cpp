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
                            : ::_acsl_::_control_::controller_base(logger, trajectory)
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
    auto r_user = m_traj.GetPosition();
    auto v_user = m_traj.GetVelocity();
    auto a_user = m_traj.GetAcceleration();

    // 3. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 4. Compute the rotation matrices --------------------------------------------
    

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

    // 9. Compute the inverse jacobian for the Euler angles -------------------------------
    cim.Jacobian_inv = ::_shared_::_transformations_::jacobianMatrixInverse(cim.roll, cim.pitch);

    // 10. Compute the angular rates from angular velocity --------------------------------
    cim.omega_rot << cim.w_x, cim.w_y, cim.w_z;
    cim.eta_rot_rate_real = cim.Jacobian_inv * cim.omega_rot;

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

    // 2. Do the integration
    rk54.do_step(boost::bind(&mrac_long_lat::model, this, bph::_1, bph::_2, bph::_3),
                 y, cim.t, time_step_rk4_);

    // PLACEHOLDER TO GIVE RANDOM INPUTS
    control_input(0) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, 0.55*MAX_THRUST);
    control_input(1) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, 0.55*MAX_THRUST);
    control_input(2) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, 0.45*MAX_THRUST);
    control_input(3) = ::_shared_::_compute_::evaluatePolynomial(thrust_polynomial_coeff_qrbp, 0.65*MAX_THRUST);

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
        << "phi dot real, " 
        << "theta dot real, "
        << "psi dot real, "
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
        << cim.eta_rot_rate_real(0) << ", "
        << cim.eta_rot_rate_real(1) << ", "
        << cim.eta_rot_rate_real(2) << ", "
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