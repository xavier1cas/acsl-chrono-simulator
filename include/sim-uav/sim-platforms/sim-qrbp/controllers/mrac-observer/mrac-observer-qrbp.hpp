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
 * File:        mrac-observer-qrbp.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 23, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for mrac omega controller for the QRBP along with the observer estimating the position.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef MRAC_OBSERVER_QRBP_HPP_
#define MRAC_OBSERVER_QRBP_HPP_

#include "sim-control-base.hpp"     // Include for the base class of a controller defined in the simualtor 
#include "qrbp-parameter-file.hpp"  // Include for the hardcoded qrbp parameters that are common for all controllers

namespace _acsl_
{

namespace _qrbp_
{

namespace _mrac_observer_
{

// Define the number of states in the boost array for integration
constexpr int NSI = 241;

// Structure for all parameter members of the controller
struct controller_internal_parameters {
    Eigen::Matrix<double, 3, 3> Kp_refmod_tran;  	// Proportional gains for the translational reference model
    Eigen::Matrix<double, 3, 3> Ki_refmod_tran;  	// Integral gains for the translational reference model
    Eigen::Matrix<double, 3, 3> Kd_refmod_tran;  	// Derivative gains for the translational reference model
	Eigen::Matrix<double, 3, 3> Kp_tran;         	// Proportional translational control gains
    Eigen::Matrix<double, 3, 3> Ki_tran;         	// Integral translational control gains
    Eigen::Matrix<double, 3, 3> Kd_tran;         	// Derivative translational control gains
	Eigen::Matrix<double, 6, 6>  Gamma_x_tran;   	// Adaptive gain for state feedback parameters
    Eigen::Matrix<double, 3, 3>  Gamma_r_tran;   	// Adaptive gain for command tracking parameters
    Eigen::Matrix<double, 30, 30> Gamma_Theta_tran; // Adaptive gain for dynamic parameter regression
    Eigen::Matrix<double, 6, 6> Q_tran;          	// Lyapunov weighting matrix
    Eigen::Matrix<double, 6, 6> P_tran;          	// Solution matrix to continuous Lyapunov equation
    Eigen::Matrix<double, 6, 6> A_tran;          	// Translational system matrix
    Eigen::Matrix<double, 6, 3> B_tran;          	// Translational input matrix
    Eigen::Matrix<double, 6, 6> A_ref_tran;      	// Reference model system matrix
    Eigen::Matrix<double, 6, 3> B_ref_tran;      	// Reference model input matrix
    Eigen::Matrix<double, 3, 3> Kp_omega_ref;    	// Proportional rotational reference model gains
    Eigen::Matrix<double, 3, 3> Ki_omega_ref;    	// Integral rotational reference model gains
    Eigen::Matrix<double, 3, 3> Kp_rot;          	// Proportional rotational control gains
    Eigen::Matrix<double, 3, 3> Ki_rot;          	// Integral rotational control gains
    Eigen::Matrix<double, 3, 3> Kd_rot;          	// Derivative rotational control gains
    Eigen::Matrix<double, 3, 3>  Gamma_x_rot;    	// Adaptive gain for state feedback parameters
    Eigen::Matrix<double, 3, 3>  Gamma_r_rot;    	// Adaptive gain for command tracking parameters
    Eigen::Matrix<double, 12, 12> Gamma_Theta_rot;  // Adaptive gain for dynamic regression parameters
    Eigen::Matrix<double, 3, 3> Q_rot;           	// Lyapunov weighting matrix (rotation)
    Eigen::Matrix<double, 3, 3> P_rot;           	// Lyapunov solution matrix (rotation)
    Eigen::Matrix<double, 3, 3> A_rot;           	// Rotational system matrix
    Eigen::Matrix<double, 3, 3> B_rot;           	// Rotational input matrix
    Eigen::Matrix<double, 3, 3> A_ref_rot;       	// Rotational reference model matrix
    Eigen::Matrix<double, 3, 3> B_ref_rot;       	// Rotational reference input matrix
    double dead_zone_delta_translational;        	// Translational deadzone delta radius
    double dead_zone_e0_translational;           	// Translational deadzone error tolerance
    double dead_zone_delta_rotational;           	// Rotational deadzone delta radius
    double dead_zone_e0_rotational;              	// Rotational deadzone error tolerance
    double sigma_x_translational;                	// Translational E-mod gain for x (states)
    double sigma_r_translational;                	// Translational E-mod gain for r (commands)
    double sigma_Theta_translational;            	// Translational E-mod gain for Theta (parameters)
    double sigma_x_rotational;                   	// Rotational E-mod gain for x (states)
    double sigma_r_rotational;                   	// Rotational E-mod gain for r (commands)
    double sigma_Theta_rotational;               	// Rotational E-mod gain for Theta (parameters)
    double projection_x_max_x_translational;     	// Translational Projection limit for Kx_hat
    double projection_epsilon_x_translational;   	// Translational Projection tolerance for Kx_hat
    double projection_x_max_r_translational;     	// Translational Projection limit for Kr_hat
    double projection_epsilon_r_translational;   	// Translational Projection tolerance for Kr_hat
    double projection_x_max_Theta_translational; 	// Translational Projection limit for Theta_hat
    double projection_epsilon_Theta_translational; 	// Translational Projection tolerance for Theta_hat
    double projection_x_max_x_rotational;        	// Rotational Projection limit for Kx_hat
    double projection_epsilon_x_rotational;      	// Rotational Projection tolerance for Kx_hat
    double projection_x_max_r_rotational;        	// Rotational Projection limit for Kr_hat
    double projection_epsilon_r_rotational;      	// Rotational Projection tolerance for Kr_hat
    double projection_x_max_Theta_rotational;    	// Rotational Projection limit for Theta_hat
    double projection_epsilon_Theta_rotational;  	// Rotational Projection tolerance for Theta_hat 
    bool use_projection_operator;                   // Boolean for switching on/off the projection operator
};

// Structure for holding all parameter members of the observer
struct observer_internal_parameters {
    Eigen::Matrix<double, 3, 6> C_tran_observer;            // Observer C Matrix
    Eigen::Matrix<double, 6, 6> A_tran_observer_ref;        // A_{ref,y} matrix for the observer's plant
    Eigen::Matrix<double, 6, 3> B_tran_observer;            // B matrix for the observer's plant
    Eigen::Matrix<double, 6, 3> L_tran_observer;            // L Matrix for the observer's plant
    Eigen::Matrix<double, 3, 3> Gamma_tran_observer_y;      // Adaptive rate for the plant
    Eigen::Matrix<double, 4, 4> Gamma_tran_observer_Theta;  // Adaptive rate for the observer's regressor
    double projection_x_max_Gamma_tran_observer_y;          // Projection limit for Gamma_y
    double projection_epsilon_Gamma_tran_observer_y;        // Projection tolerance for Gamma_y
    double projection_x_max_Gamma_tran_observer_Theta;      // Projection limit for Gamma_Theta_y
    double projection_epsilon_Gamma_tran_observer_Theta;    // Projection tolerance for Gamma_Theta_y
    double dead_zone_e0_Gamma_tran_observer_y;              // Dead-zone tolerance for Gamma_y
    double dead_zone_e0_Gamma_tran_observer_Theta;          // Dead-zone tolerance for Gamma_Theta_y
};

// Structure for all the members that are mapped to the rk4 vector AFTER integration
struct controller_integrated_state_members {
    Eigen::Matrix<double, 3, 1> e_tran_pos_I;           // Translational integral error
    Eigen::Matrix<double, 3, 1> e_rot_eta_I;            // Integral error in Euler Angles
    Eigen::Matrix<double, 2, 1> state_roll_d_filter;    // States for filter
    Eigen::Matrix<double, 2, 1> state_pitch_d_filter;   // States for filter
    Eigen::Matrix<double, 2, 1> state_omega_x_d_filter; // States for filter
    Eigen::Matrix<double, 2, 1> state_omega_y_d_filter; // States for filter
    Eigen::Matrix<double, 2, 1> state_omega_z_d_filter; // States for filter
	Eigen::Matrix<double, 3, 1> e_tran_pos_ref_I;		// Translational Integral error (reference model - user cmd)
	Eigen::Matrix<double, 6, 1> x_tran_ref;				// Reference model in I
	Eigen::Matrix<double, 6, 3> K_hat_x_tran;			// Translational Adaptive gains for x
	Eigen::Matrix<double, 3, 3> K_hat_r_tran;			// Translational Adaptive gains for r
	Eigen::Matrix<double, 30, 3> Theta_hat_tran;		// Translational Adaptive gains for Theta
	Eigen::Matrix<double, 3, 1> e_rot_omega_ref_I;	    // Integral error (ang vel ref model - desired angular velocity)
	Eigen::Matrix<double, 3, 1> omega_rot_ref;			// Angular Velocity Reference model
	Eigen::Matrix<double, 3, 3> K_hat_x_rot;			// Rotational Adaptive gains for x
	Eigen::Matrix<double, 3, 3> K_hat_r_rot;			// Rotational Adaptive gains for r
	Eigen::Matrix<double, 12, 3> Theta_hat_rot;			// Rotational Adaptive gains for Theta
};

// Structure for all the members that are mapped to the rk4 vector AFTER integration
struct observer_integrated_state_members {
    Eigen::Matrix<double, 6, 1> x_hat_tran_observer;       // Observer estimated states
    Eigen::Matrix<double, 3, 3> K_hat_tran_observer_y;     // Observer Gain
    Eigen::Matrix<double, 4, 3> Theta_hat_tran_observer_y; // Observer Gain
};

// Structure for all the internal members of the controller
struct controller_internal_members {
    double t;                                                      // Time
    Eigen::Matrix<double, 3, 3> R_I_J;                             // Rotation from inertial to body
    Eigen::Matrix<double, 3, 3> R_J_I;                             // Rotation from body to inertial
    Eigen::Matrix<double, 3, 3> Jacobian;                          // Jacobian Matrix
    Eigen::Matrix<double, 3, 1> r_user;                            // Translational user position command
    Eigen::Matrix<double, 3, 1> r_dot_user;                        // Translational user veloctiy command
    Eigen::Matrix<double, 3, 1> r_ddot_user;                       // Translational user acceleration command 
    Eigen::Matrix<double, 3, 1> x_tran_pos;                        // Translational position
    Eigen::Matrix<double, 3, 1> x_tran_vel;                        // Translational velocity
    Eigen::Matrix<double, 3, 1> x_tran_vel_J;                      // Translational velocity in body
    Eigen::Matrix<double, 3, 1> e_tran_pos;                        // Translational error in position
    Eigen::Matrix<double, 3, 1> e_tran_vel;                        // Translational error in velocity
    Eigen::Matrix<double, 3, 1> mu_tran_baseline;                  // Baseline control input
    Eigen::Matrix<double, 3, 1> mu_tran_I;                         // Virtual control action in the in inertial frame
    Eigen::Matrix<double, 3, 1> mu_tran_J;                         // Virtual control action in the in body frame
    Eigen::Matrix<double, 3, 1> eta_rot_d;                         // Desired angle command
    Eigen::Matrix<double, 3, 1> eta_rot_rate_d;                    // Desired angular rates
    Eigen::Matrix<double, 3, 1> omega_rot_d;                       // Desired angular velocities
    Eigen::Matrix<double, 3, 1> alpha_rot_d;                       // Desired angular acceleration
    Eigen::Matrix<double, 3, 1> eta_rot;                           // Euler Angles
    Eigen::Matrix<double, 3, 1> omega_rot;                         // Angular velocities
    Eigen::Matrix<double, 3, 1> e_rot_eta;                         // Error in the angles
    Eigen::Matrix<double, 3, 1> e_rot_omega;                       // Error in the angular velocities
    Eigen::Matrix<double, 3, 1> tau_rot_baseline;                  // Baseline rotational control input 
    Eigen::Matrix<double, 3, 1> tau_rot;                           // Rotational Control action
    Eigen::Matrix<double, 4, 1> u;                                 // [thrust; mx; my; mz]
    Eigen::Matrix<double, 4, 1> Thrust;                            // Newtons
    Eigen::Matrix<double, 4, 1> Sat_Thrust;                        // Saturated thrust 
    Eigen::Quaterniond q;                                          // Quaternion
    Eigen::Matrix<double, 2, 1> internal_state_roll_d_filter;      // Internal States for filter
    Eigen::Matrix<double, 2, 1> internal_state_pitch_d_filter;     // Internal States for filter
    Eigen::Matrix<double, 2, 1> internal_state_omega_x_d_filter;   // Internal States for filter
    Eigen::Matrix<double, 2, 1> internal_state_omega_y_d_filter;   // Internal States for filter
    Eigen::Matrix<double, 2, 1> internal_state_omega_z_d_filter;   // Internal States for filter
    double alg_duration;                                           // Control execution duration
    std::chrono::high_resolution_clock::time_point alg_start_time; // Algorithm Start timepoint
    std::chrono::high_resolution_clock::time_point alg_end_time;   // Algorithm End timepoint
	Eigen::Matrix<double, 3, 3> R_W_J;							   // Rotation from wind frame to body 
	Eigen::Matrix<double, 6, 1> x_tran;							   // [x_tran_pos; x_tran_vel]
	Eigen::Matrix<double, 6, 1> x_tran_ref_dot;					   // Ref model to be integrated
	Eigen::Matrix<double, 6, 3> K_hat_x_tran_dot;				   // Adaptive gain to be integrated
	Eigen::Matrix<double, 3, 3> K_hat_r_tran_dot;				   // Adaptive gain to be integrated
	Eigen::Matrix<double, 30, 3> Theta_hat_tran_dot;			   // Adaptive gain to be integrated
	Eigen::Matrix<double, 27, 1> outer_loop_regressor;			   // Outer loop regressor
	Eigen::Matrix<double, 30, 1> augmented_outer_loop_regressor;   // Outer loop augmented regressor
	Eigen::Matrix<double, 6, 1> e_tran;							   // [e_tran_pos; e_tran_vel]
	Eigen::Matrix<double, 3, 1> e_tran_pos_ref;					   // ref model errror
	Eigen::Matrix<double, 3, 1> r_cmd_tran;						   // r_cmd for outerloop
	Eigen::Matrix<double, 3, 1> mu_tran_adaptive;				   // Virtual adaptive control input - outerloop
	Eigen::Matrix<double, 9, 1> inner_loop_regressor;			   // Inner loop regressor vector
	Eigen::Matrix<double, 12, 1> augmented_inner_loop_regressor;   // Inner loop augmented regressor vector
	Eigen::Matrix<double, 3, 3> K_hat_x_rot_dot;				   // Adaptive gain to be integrated
	Eigen::Matrix<double, 3, 3> K_hat_r_rot_dot;				   // Adaptive gain to be integrated
	Eigen::Matrix<double, 12, 3> Theta_hat_rot_dot;				   // Adaptive gain to be integrated
	Eigen::Matrix<double, 3, 1> tau_rot_adaptive;				   // Innerloop adaptive control input
	Eigen::Matrix<double, 3, 1> e_rot_omega_ref;				   // Inner loop ref model error
	Eigen::Matrix<double, 3, 1> omega_cmd;						   // r_cmd for inner loop
	Eigen::Matrix<double, 3, 1> omega_rot_ref_dot;				   // Ref model to be integrated
	double dead_zone_value_translational;						   // Dead zone val - OL
	double dead_zone_value_rotational;							   // Dead zone val - IL
	bool proj_op_activated_K_hat_x_translational;				   // Projection activation boolean - OL - K_hat_x
	bool proj_op_activated_K_hat_r_translational;				   // Projection activation boolean - OL - K_hat_r
	bool proj_op_activated_Theta_hat_translational;				   // Projection activation boolean - OL - Theta_hat
	bool proj_op_activated_K_hat_x_rotational;					   // Projection activation boolean - IL - K_hat_x
	bool proj_op_activated_K_hat_r_rotational;					   // Projection activation boolean - IL - K_hat_r
	bool proj_op_activated_Theta_hat_rotational;				   // Projection activation boolean - IL - Theta_hat
};

// Structure for all the internal members of the observer
struct observer_internal_members {
    Eigen::Matrix<double, 3, 1> y_tran_observer_output;            // y_{output} = C x (true measured value)
    Eigen::Matrix<double, 3, 1> y_tran_observer_est;               // y_{estimate} = C \hat{x} (estimate)
    Eigen::Matrix<double, 4, 1> Phi_tran_observer_y;               // Regressor for the observer plant
    Eigen::Matrix<double, 3, 3> K_hat_tran_observer_y_dot;         // Derivative of observer Gain
    Eigen::Matrix<double, 4, 3> Theta_hat_tran_observer_y_dot;     // Derivative of observer Gain
    Eigen::Matrix<double, 6, 1> x_hat_tran_observer_dot;                         // Observer model
    Eigen::Matrix<double, 3, 1> u_tran_observer;                   // The virtual control of the observer
    Eigen::Matrix<double, 3, 1> e_tran_observer;                   // Y_output - Y_est
    double dead_zone_value_K_hat_y;                                // dead zone value for the gains
    double dead_zone_value_Theta_hat_y;                            // dead zone value for the gains
    bool proj_op_activated_K_hat_tran_observer_y;				   // Projection activation boolean - K_hat_tran_observer_y
    bool proj_op_activated_Theta_hat_tran_observer_y;			   // Projection activation boolean - Theta_hat_tran_observer_y
};


// =========================================================================================================
// mrac_observer.hpp   -- QRBP MRAC Omega Controller w/ PID Baseline and Adaptive Observer for position.
//   - Implements a MRAC controller for angular rates on the QRBP platform.
//   - Implements an Adaptive observer for estimating the positional data by measuring the velocity and u.
//   - Inherits base routines and actuator interface from controller_base.
//   - Inherits base routines from blackbox to setup the logging.
// =========================================================================================================
class mrac_observer : public ::_acsl_::_control_::controller_base, public ::_acsl_::_logger_::blackbox
{

public:
	// -------------------------------------------------------------------------
    // Constructor:
    //   - Accepts simlog reference for logging duties.
    //   - Passes simlog to controller_base for unified logging.
    // -------------------------------------------------------------------------
    mrac_observer(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory);

	// -------------------------------------------------------------------------
    // Destructor:
    //   - Allows cleanup in derived class.
    // -------------------------------------------------------------------------
    virtual ~mrac_observer() = default;

	// -------------------------------------------------------------------------
    // Override: update()
    //   - Update controller state (to be implemented in mrac_observer.cpp).
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

    // Create a RungeKutta object for integration functionality.
    boost::numeric::odeint::runge_kutta4<_control_::rk4_array<double, NSI>> rk4;

    // Define the internal parameter members of the controller 
    ::_acsl_::_qrbp_::_mrac_observer_::controller_internal_parameters cip;

    // Define the internal parameter members of the observer
    ::_acsl_::_qrbp_::_mrac_observer_::observer_internal_parameters oip;

    // Define the internal members of the controller
    ::_acsl_::_qrbp_::_mrac_observer_::controller_internal_members cim;   

    // Define the internal members of the observer
    ::_acsl_::_qrbp_::_mrac_observer_::observer_internal_members oim;

    // Define the internal integrated state members of the controller
    ::_acsl_::_qrbp_::_mrac_observer_::controller_integrated_state_members csm; 

    // Define the internal integrated state members of the observer
    ::_acsl_::_qrbp_::_mrac_observer_::observer_integrated_state_members osm;

private:
    // -------------------------------------------------------------------------
    // NON API CONTROLLER SPECIFIC FUNCTIONS - WILL BE DIFF FOR DIFF CONTROLLERS
    // -------------------------------------------------------------------------
    // Function to compute the translational regressor vector 
    void compute_outer_loop_regressor();

    // Funciton to compute the rotational regressor vector
    void compute_inner_loop_regressor();
    
    // Function to compute the translational control in the inertial frame
    void compute_translational_control_in_I();

    // Function to compute the thrust (u1) and the desired angles
    void compute_u1_eta_d();

    // Function to compute the rotational control input
    void compute_rotational_control();

    // Function to compute the normalized thrust
    void compute_normalized_thrusts();

    // Assign the values from rk4 to controller internal members
    void assign_from_rk4();		

    // Function to compute the dynamics of the observer
    void compute_observer_dynamics();

};

} // namespace _mrac_observer_

} // namespace _qrbp_

} // namespace _acsl_

#endif  // MRAC_OBSERVER_QRBP_HPP_