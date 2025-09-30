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
 * File:        pid-omega-qrbp.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 23, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: PID  with angular velocities for the QRBP. Inherts the class controller_base for the basic 
 *              functionality that is to be used for all control algorithms.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "pid-omega-qrbp.hpp"

namespace _acsl_
{

namespace _qrbp_
{

namespace _pid_omega_
{

// -------------------------------------------------------------------------
// Constructor Implementation:
//   - Calls the base (controller_base) constructor and passes 
//     both logger and trajectory.
// -------------------------------------------------------------------------
pid_omega::pid_omega(_acsl_::_logger_::simlog& logger, ::_acsl_::_trajectory_::trajectorybase& trajectory)
                    : ::_acsl_::_control_::controller_base(logger, trajectory)
{
    
}

// -------------------------------------------------------------------------
// read_params Implementation:
// - Takes the hardcoded path for the gains and parameters and reads it in
// -------------------------------------------------------------------------
void pid_omega::read_params(const std::string& jsonFile)
{
    // Implementation here
    std::ifstream file(jsonFile);
    nlohmann::json j;
    file >> j;

    // Translational parameters
    cip.Kp_tran = _shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KP_translational"], 3, 3);
    cip.Ki_tran = _shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KI_translational"], 3, 3);
    cip.Kd_tran = _shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KD_translational"], 3, 3);

    // Rotational parameters
    cip.Kp_rot = _shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KP_rotational"], 3, 3);
    cip.Ki_rot = _shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KI_rotational"], 3, 3);
    cip.Kd_rot = _shared_::_deserialize_::jsonToScaledMatrixXd(j["BASELINE"]["KD_rotational"], 3, 3);

}

// Implementing virutal functions from controller_base
void pid_omega::init(){

    // Set the inital conditions
    y.fill(0.0);
    dy.fill(0.0);
}

// Update function for the controller
void pid_omega::update(double time, 
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

    // Assign the Rotational States
    cim.eta_rot << roll, pitch, yaw;
    cim.omega_rot << w_x, w_y, w_z;

    // Assign the quaternions
    cim.q.w() = q0;
    cim.q.z() = q1;
    cim.q.y() = q2;
    cim.q.z() = q3;

    // 2. Get the reference trajectory ---------------------------------------------
    m_traj.UpdateModule(time);
    cim.r_user = m_traj.GetPosition();
    cim.r_dot_user = m_traj.GetVelocity();
    cim.r_ddot_user = m_traj.GetAcceleration();   

    // Set the desired yaw and yaw_rate; we don't need the acceleration as 
    // we are going to compute it from the filter.
    cim.eta_rot_d(2) = m_traj.GetEulerAngle()(2);
    cim.eta_rot_rate_d(2) = m_traj.GetEulerRate()(2);

    // 3. Capture the time before the execution of the controller ------------------
    cim.alg_start_time = std::chrono::high_resolution_clock::now();

    // 4. Compute the rotation matrices --------------------------------------------
    cim.R_I_J = _shared_::_transformations_::rotationMatrix321GlobalToLocal(cim.eta_rot(0), cim.eta_rot(1), cim.eta_rot(2));
    cim.R_J_I = _shared_::_transformations_::rotationMatrix321LocalToGlobal(cim.eta_rot(0), cim.eta_rot(1), cim.eta_rot(2));

    // 5. Compute the body velocities in the quadcopter frame from the inertial velocities
    cim.x_tran_vel_J = cim.R_I_J * cim.x_tran_vel;

    // 6. Assign the values from the integrator ------------------------------------
    assign_from_rk4();
}

// Function to assign elements from the rk4 integrator
void pid_omega::assign_from_rk4()
{
    //------------ assign after integration  ------------//
    int index = 0;
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_roll_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_pitch_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_x_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_y_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.state_omega_z_d_filter, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_tran_pos_I, y, index);
    ::_shared_::_deserialize_::assignElementsToMembers(csm.e_rot_eta_I, y, index);
}

}   // namespace _pid_omega_

}   // namespace _qrbp_
    
}   // namespace _acsl_
