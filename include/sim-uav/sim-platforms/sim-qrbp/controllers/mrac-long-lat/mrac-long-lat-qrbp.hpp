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
 * File:        mrac-long-lat-qrbp.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        February 11, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for mrac longitudinal/lateral controller for the QRBP.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef MRAC_LONG_LAT_QRBP_HPP_
#define MRAC_LONG_LAT_QRBP_HPP_

#include "sim-control-base.hpp"     // Include for the base class of a controller defined in the simualtor
#include "qrbp-parameter-file.hpp"  // Include for the hardcoded qrbp parameters that are common for all controllers
#include "sim-aerofoil.hpp"         // Include file for the aerofoil coeff computations

namespace _acsl_
{

namespace _qrbp_
{   

namespace _mrac_long_lat_
{

// Define the number of states in the boost array for integration
constexpr int NSI = 43;

// Structure for all parameter members of the controller
struct controller_internal_parameters {
    Eigen::Matrix<double, 2, 2> Kp_refmod_lon_ol;                  // Proportional gains for the long ol ref model
    Eigen::Matrix<double, 2, 2> Kd_refmod_lon_ol;                  // Derivative gains for the long ol ref model
    Eigen::Matrix<double, 4, 4> A_ref_lon_ol;                      // A ref for the longitudinal outerloop 
    Eigen::Matrix<double, 4, 2> B_ref_lon_ol;                      // B ref for the longitudinal outerloop
    Eigen::Matrix<double, 2, 2> Kp_cmd_lon_ol;                     // Proportional gains for the reference command
    Eigen::Matrix<double, 2, 2> Kd_cmd_lon_ol;                     // Derivative gains for the reference command
    Eigen::Matrix<double, 2, 2> Ki_cmd_lon_ol;                     // Integral gains for the reference command 
    Eigen::Matrix<double, 2, 2> Kp_lon_ol;                         // Baseline proportional gains for the longitudinal outerloop
    Eigen::Matrix<double, 2, 2> Kd_lon_ol;                         // Baseline derivative gains for the longitudinal outerloop
    Eigen::Matrix<double, 2, 2> Ki_lon_ol;                         // Baseline integral gains for the longitudinal outerloop
    Eigen::Matrix<double, 4, 4> Q_lon_ol;                          // Lyapunov weighting matrix for the longitudinal outerloop
    Eigen::Matrix<double, 4, 4> P_long_ol;                         // SOlutaion matrix to the continuous Lyapunov eqn for longitudinal outerloop
    Eigen::Matrix<double, 4, 2> B_lon_ol;                          // B matrix for the longitudinal outerloop
    Eigen::Matrix<double, 4, 4> Gamma_x_lon_ol;   	               // Adaptive gain for state feedback parameters
    Eigen::Matrix<double, 2, 2> Gamma_r_lon_ol;   	               // Adaptive gain for command tracking parameters
    Eigen::Matrix<double, 5, 5> Gamma_Theta_lon_ol;                // Adaptive gain for dynamic parameter regression
    double epsilon_x_lon_ol;                                       // Parameter to determine theta_cmd
    double epsilon_z_lon_ol;                                       // Parameter to determine theta_cmd
    double dead_zone_delta_lon_out;                                // Longitudinal outerloop deadzone delta radius 
    double dead_zone_e0_lon_out;                                   // Longitudinal outerloop deadzone error tolerance
    double sigma_x_lon_out;                	                       // Longitudinal outerloop E-mod gain for x (states)
    double sigma_r_lon_out;                	                       // Longitudinal outerloop E-mod gain for r (commands)
    double sigma_Theta_lon_out;            	                       // Longitudinal outerloop E-mod gain for Theta (parameters)
    double projection_x_max_x_lon_out;                             // Longitudinal outerloop Projection limit for Kx_hat
    double projection_epsilon_x_lon_out;   	                       // Longitudinal outerloop Projection tolerance for Kx_hat
    double projection_x_max_r_lon_out;     	                       // Longitudinal outerloop Projection limit for Kr_hat
    double projection_epsilon_r_lon_out;   	                       // Longitudinal outerloop Projection tolerance for Kr_hat
    double projection_x_max_Theta_lon_out; 	                       // Longitudinal outerloop Projection limit for Theta_hat
    double projection_epsilon_Theta_lon_out; 	                   // Longitudinal outerloop Projection tolerance for Theta_hat
    bool use_projection_operator_lon_out;                          // Longitudinal outerloop boolean for switching on/off the projection operator
};

// Structure for all the members that are mapped to the rk4 vector AFTER integration
struct controller_integrated_state_members {
    Eigen::Matrix<double, 4, 1> x_ref_lon_out;                     // Longitudinal outerloop reference model
    Eigen::Matrix<double, 2, 1> e_ref_lon_out_I;                   // Longitudinal outerloop reference model error
    Eigen::Matrix<double, 2, 1> e_lon_out_I;                       // Longitudinal outerloop position error
    Eigen::Matrix<double, 4, 2> K_hat_x_lon_out;                   // Longitudinal outerloop adaptive gain after integration
    Eigen::Matrix<double, 2, 2> K_hat_r_lon_out;                   // Longitudinal outerloop adaptive gain after integration
    Eigen::Matrix<double, 5, 2> Theta_hat_lon_out;                 // Longitudinal outerloop adaptive gain after integration
};

// Structure for all internal members of the controller
struct controller_internal_members {    
    double t;                                                      // Time
    double alg_duration;                                           // Control execution duration
    std::chrono::high_resolution_clock::time_point alg_start_time; // Algorithm Start timepoint
    std::chrono::high_resolution_clock::time_point alg_end_time;   // Algorithm End timepoint
    double x;                                                      // Position in x 
    double y;                                                      // Position in y
    double z;                                                      // Position in z
    double vx;                                                     // Velocity in x
    double vy;                                                     // Velocity in y
    double vz;                                                     // Velocity in z
    Eigen::Matrix<double, 3, 1> v_I;                               // Velocity in I
    Eigen::Matrix<double, 3, 1> v_J;                               // Velocity in J
    double v_norm;                                                 // Norm of the translational velocities
    double alpha_up;                                               // Angle of attack for the upper wing
    double alpha_lw;                                               // Angle of attack for the lower wing
    double alpha_com;                                              // Angle of attack for the center of mass
    double gamma_com;                                              // Flight path angle for the center of mass
    Eigen::Quaterniond q;                                          // Quaternion
    double roll;                                                   // Roll
    double pitch;                                                  // Pitch
    double yaw;                                                    // Yaw
    double w_x;                                                    // Angular velocity in x
    double w_y;                                                    // Angular velocity in y
    double w_z;                                                    // Angular velocity in z
    double roll_rate;                                              // Roll rate
    double pitch_rate;                                             // Pitch rate
    double yaw_rate;                                               // Yaw rate
    Eigen::Matrix<double, 3, 1> r_user;                            // User-defined position
    Eigen::Matrix<double, 3, 1> v_user;                            // User-defined velocity
    Eigen::Matrix<double, 3, 1> a_user;                            // User-defined acceleration
    Eigen::Matrix<double, 2, 1> r_user_lon_out;                    // Longitudinal outerloop user-defined position
    Eigen::Matrix<double, 2, 1> v_user_lon_out;                    // Longitudinal outerloop user-defined velocity
    Eigen::Matrix<double, 2, 1> a_user_lon_out;                    // Longitudinal outerloop user-defined acceleration
    Eigen::Matrix<double, 3, 3> R_I_J;                             // Rotation from inertial to body
    Eigen::Matrix<double, 3, 3> R_J_I;                             // Rotation from body to inertial    
    Eigen::Matrix<double, 3, 3> Jacobian_inv;                      // Inverse of Jacobian Matrix
    Eigen::Matrix<double, 3, 1> omega_rot;                         // Angular velocities
    Eigen::Matrix<double, 4, 1> x_lon_out;                         // Longitudinal outerloop states
    Eigen::Matrix<double, 4, 1> x_ref_lon_out_dot;                 // Longitudinal outerloop reference model
    Eigen::Matrix<double, 2, 1> e_lon_out_pos;                     // Longitudinal outerloop error in position
    Eigen::Matrix<double, 2, 1> e_lon_out_vel;                     // Longitudinal outerloop error in velocity
    Eigen::Matrix<double, 2, 1> e_ref_lon_out;                     // Longitudinal outerloop reference model error
    Eigen::Matrix<double, 2, 1> r_cmd_lon_out;                     // Longitudinal outerloop reference command
    Eigen::Matrix<double, 2, 1> F_baseline_lon_out;                // Longitudinal outerloop baseline control input
    Eigen::Matrix<double, 2, 1> F_adaptive_lon_out;                // Longitudinal outerloop adaptive control input
    Eigen::Matrix<double, 2, 1> F_lon_out;                         // Longitudinal outerloop total control input
    Eigen::Matrix<double, 2, 1> F_inv_lon_out;                     // Longitudinal outerloop aerodynamic inversion term
    Eigen::Matrix<double, 3, 1> regressor_lon_out;                 // Longitudinal outerloop regressor vector
    Eigen::Matrix<double, 5, 1> augmented_regressor_lon_out;       // Longitudinal outerloop augmented regressor vector
    double dead_zone_value_lon_out;                                // Longitudinal outerloop deadzone modulation function value
    Eigen::Matrix<double, 4, 2> K_hat_x_lon_out_dot;               // Longitudinal outerloop adaptive gain to be integrated
    Eigen::Matrix<double, 2, 2> K_hat_r_lon_out_dot;               // Longitudinal outerloop adaptive gain to be integrated
    Eigen::Matrix<double, 5, 2> Theta_hat_lon_out_dot;             // Longitudinal outerloop adaptive gain to be integrated 
    bool proj_op_activated_K_hat_x_lon_out;                        // Boolean to record projection operator activation for K_hat_x_lon_out
    bool proj_op_activated_K_hat_r_lon_out;                        // Boolean to record projection operator activation for K_hat_r_lon_out
    bool proj_op_activated_Theta_hat_lon_out;                      // Boolean to record projection operator activation for Theta_hat_lon_out
};

// Structue for all parameter members of the differentiator - I am using the 2L version. 
struct differentiator_internal_paramters
{
    Eigen::Matrix<double, 3, 6> C_diff;                             // C matrix for the differentiator plants - Both 2L and MRAO
    Eigen::Matrix<double, 6, 3> B_diff;                             // B matrix for the differentiator plants - Both 2L and MRAO  
    Eigen::Matrix<double, 6, 3> L_diff;                             // L matrix for the differentiator plants - Both 2L and MRAO  
    Eigen::Matrix<double, 6, 6> A_ref_y_diff;                       // A matrix for the differentiator reference models - Both 2L and MRAO
    Eigen::Matrix<double, 6, 6> A_tran_y_diff;                      // A matrix for the transient error plant - Both 2L and MRAO
    Eigen::Matrix<double, 3, 3> Gamma_y_diff;                       // Adaptive gain matrix for the differentiator - Both 2L and MRAO
    Eigen::Matrix<double, 3, 3> Gamma_Theta_diff;                   // Adaptive gain matrix for the differentiator - Both 2L and MRAO
    Eigen::Matrix<double, 3, 3> Gamma_g_y_diff;                     // Adaptive gain matrix for the differentiator - Only 2L MRAO
    
    double projection_x_max_K_hat_y_diff;                           // Projection params
    double projection_epsilon_K_hat_y_diff;                         // Projection params
    
    double projection_x_max_Theta_hat_diff;                         // Projection params
    double projection_epsilon_Theta_hat_diff;                       // Projection params
    
    double projection_x_max_K_hat_g_y_diff;                         // Projection params
    double projection_epsilon_K_hat_g_y_diff;                       // Projection params

    Eigen::Matrix<double, 3, 3> K_ye_diff;                          // Initial gains for the differentiator
    Eigen::Matrix<double, 3, 3> Theta_e_diff;                       // Initial gains for the differentiator
    Eigen::Matrix<double, 3, 3> K_gye_diff;                         // Initial gains for the differentiator
};

// Structure for the members that are mapped to the rk4 vector after integration
struct differentiator_integrated_state_members
{
    Eigen::Matrix<double, 3, 1> int_euler_angles;                   // The integral of the Euler angles

    Eigen::Matrix<double, 6, 1> x_hat_2l_mrad;                      // Estimated rotational states for the 2L MRAD
    Eigen::Matrix<double, 3, 3> K_hat_y_2l_mrad;                    // Adaptive gain matrix for the 2L MRAD
    Eigen::Matrix<double, 3, 3> Theta_hat_2l_mrad;                  // Adaptive gain matrix for the 2L MRAD
    Eigen::Matrix<double, 3, 3> K_hat_g_y_2l_mrad;                  // Adaptive gain matrix for the 2L MRAD transient term

    Eigen::Matrix<double, 6, 1> eta_2l_mrad;                        // Transient error model for the 2L MRAD
};

// Structure for the internal members of the differntaiator
struct differentiator_internal_members
{
    Eigen::Matrix<double, 3, 1> eta_rot_unwrapped;                  // Unwrapped psi in the Euler state.
    Eigen::Matrix<double, 6, 1> x_hat_2l_mrad_dot;                  // Estimated rotational states for the 2L MRAD
    Eigen::Matrix<double, 3, 3> K_hat_y_2l_mrad_dot;                // Adaptive gain matrix for the 2L MRAD
    Eigen::Matrix<double, 3, 3> Theta_hat_2l_mrad_dot;              // Adaptive gain matrix for the 2L MRAD
    Eigen::Matrix<double, 3, 3> K_hat_g_y_2l_mrad_dot;              // Adaptive gain matrix for the 2L MRAD transient term
    Eigen::Matrix<double, 6, 1> eta_2l_mrad_dot;                    // Transient error model for the 2L MRAD
    Eigen::Matrix<double, 3, 1> y_measured_2l_mrad;                 // Measured outputs for the 2L MRAD
    Eigen::Matrix<double, 6, 1> z_measured_2l_mrad;                 // Measured full outputs for the 2L MRAD
    Eigen::Matrix<double, 3, 1> Phi_y_2l_mrad;                      // Regressor vector for the 2L MRAD
    Eigen::Matrix<double, 6, 1> nu_2l_mrad;                         // Transient error for the 2L MRAD
    Eigen::Matrix<double, 3, 1> u_2l_mrad;                          // Virtual control input of the 2L MRAD
    Eigen::Matrix<double, 3, 1> obs_error_2l_mrad;                  // (y_measured - y_estimated) for 2L MRAD
    bool proj_op_activated_K_hat_y_2l_mrad;                         // Boolean to record projection operator activation - 2L MRAD
    bool proj_op_activated_Theta_hat_2l_mrad;                       // Boolean to record projection operator activation - 2L MRAD
    bool proj_op_activated_K_hat_g_y_2l_mrad;                       // Boolean to record projection operator activation - 2L MRAD
    bool first_run_differentiator = false;                          // First run boolean to initialize the differentiator state
};


// =========================================================================================================
// mrac_long_lat_qrbp.hpp   -- QRBP Longitudinal/Lateral MRAC Controller
//   - Implements MRAC for the longitudinal and lateral dynamics of the QRBP.
//   - Inherits base routines and actuator interface from controller_base.
//   - Inherits base routines from blackbox to setup the logging.
// =========================================================================================================
class mrac_long_lat : public ::_acsl_::_control_::controller_base, public ::_acsl_::_logger_::blackbox,
                      public ::_acsl_::_uav_::_aerofoil_::simairfoil
{
public:
    // -------------------------------------------------------------------------
    // Constructor:
    //   - Accepts simlog reference for logging duties.
    //   - Passes simlog to controller_base for unified logging.
    // -------------------------------------------------------------------------
    mrac_long_lat(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory);

    // -------------------------------------------------------------------------
    // Destructor:
    //   - Allows cleanup in derived class.
    // -------------------------------------------------------------------------
    virtual ~mrac_long_lat() = default;

    // -------------------------------------------------------------------------
    // Override: update()
    //   - Update controller state (to be implemented in mrac_omega.cpp).
    //   - Provide logic in the source file.
    // -------------------------------------------------------------------------
    void update([[maybe_unused]] double time,
                [[maybe_unused]] double x,
                [[maybe_unused]] double y,
                [[maybe_unused]] double z,
                [[maybe_unused]] double vx,
                [[maybe_unused]] double vy,
                [[maybe_unused]] double vz,
                [[maybe_unused]] double q0,
                [[maybe_unused]] double q1,
                [[maybe_unused]] double q2,
                [[maybe_unused]] double q3,
                [[maybe_unused]] double roll,
                [[maybe_unused]] double pitch,
                [[maybe_unused]] double yaw,
                [[maybe_unused]] double w_x,
                [[maybe_unused]] double w_y,
                [[maybe_unused]] double w_z) override;

    // -------------------------------------------------------------------------
    // Override: init()
    //   - Provides the initiation of the controller. This includes reading the
    //     parameter files.
    // -------------------------------------------------------------------------
    void init() override;


    // ------------------------------------------------------------------------
    // Blackbox functions for the physics logger.
    // ------------------------------------------------------------------------
    // -------------------------------------------
    // Function: InitiateLogging
    //   - Initiates the logging for controller.
    //   - Creates the log folder structure.
    //   - Copies the gains to the logs.
    // -------------------------------------------
    bool InitiateLogging() override;

    // -------------------------------------------
    // Function: ConfigureHeaders
    //   - Simple function that has an oss stream 
    //     and writes the headers to the log file
    // -------------------------------------------
    void ConfigureHeaders() override;
        
    // -------------------------------------------
    // Function: LogData
    //   - Simple function that has an oss stream 
    //     and writes the data corresponding to 
    //     the headers to the log file.
    // ------------------------------------------
    void LogData() override;

    // -------------------------------------------------------------------------
    // Override: run()
    //   - Provides the entire logic of the controller.
    // -------------------------------------------------------------------------
    void run([[maybe_unused]] const double time_step_rk4_) override;

    // -------------------------------------------------------------------------
    // Override: read_params()
    //   - Reads in the paramters for the controller
    // -------------------------------------------------------------------------
    void read_params([[maybe_unused]] const std::string& jsonFile) override;

private:
    // -------------------------------------------------------------------------
    // NON API FUNCTIONS NECESSARY FOR ALL CONTROLLERS - MUST BE IMPLEMENTED
    // -------------------------------------------------------------------------
    // Define the state vector
    _control_::rk4_array<double, NSI> y;
    _control_::rk4_array<double, NSI> dy;

    // Define the model
    void model(const _control_::rk4_array<double, NSI> &y, _control_::rk4_array<double, NSI> &dy, double t);

    // Create a RungeKutta-Cash-Karp54 object
    boost::numeric::odeint::runge_kutta_cash_karp54<_control_::rk4_array<double, NSI>> rk54;

    // Define the internal parameter members of the controller 
    ::_acsl_::_qrbp_::_mrac_long_lat_::controller_internal_parameters cip;

    // Define the internal members of the controller
    ::_acsl_::_qrbp_::_mrac_long_lat_::controller_internal_members cim;   

    // Define the internal integrated state members of the controller
    ::_acsl_::_qrbp_::_mrac_long_lat_::controller_integrated_state_members csm; 

    // Define the internal parameter members of the differentiator
    ::_acsl_::_qrbp_::_mrac_long_lat_::differentiator_internal_paramters dip;

    // Define the internal members of the differentiator
    ::_acsl_::_qrbp_::_mrac_long_lat_::differentiator_internal_members dim;

    // Define the internal integrated state members of the differentiator
    ::_acsl_::_qrbp_::_mrac_long_lat_::differentiator_integrated_state_members dsm;

    // Member to unwrap the heading for differentiation
    ::_shared_::_compute_::SimplePsiUnwrapState psiState;

private:
    // -------------------------------------------------------------------------
    // NON API CONTROLLER SPECIFIC FUNCTIONS - WILL BE DIFF FOR DIFF CONTROLLERS
    // -------------------------------------------------------------------------
    // Function to differentiate the innerloop desired orientation commands
    void differentiate_for_euler_rates();

    // Function to compute the outerloop of the longitudinal controller
    void compute_outerloop_longitudinal();

    // Function to compute the theta_cmd for the longitudinal controller's inner loop
    void compute_theta_cmd_longitudinal();

    // Function to comptue the innerloop of the longitudnal controller
    void compute_innerloop_longitudinal();

    // Function to compute the saturated thrust for the longitudinal controller
    void compute_T_hat();

    // Function to compute the T_upper and T_lower values based on My and T_hat
    void compute_T_upper_T_lower();

    // Function to compute the outerloop of the lateral controller
    void compute_outerloop_lateral();

    // Function to compute the roll and pitch commands for the lateral controller's inner loop
    void compute_roll_pitch_cmd_lateral();

    // Function to compute the innerloop of the lateral controller
    void compute_innerloop_lateral();

    // Function to compute the normalized thrust
    void compute_normalized_thrusts();

    // Assign the values from rk4 to controller internal members
    void assign_from_rk4();
};


}   // namespace _mrac_long_lat_

}   // namespace _qrbp_

}   // namespace _acsl_

#endif  // MRAC_LONG_LAT_QRBP_HPP_