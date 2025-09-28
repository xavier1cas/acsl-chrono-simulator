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
 * File:        vector-interpolation-trajectory.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        Sep 16, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Vector Interpolation trajectory which works with the transform_trajectory module of matlab. Inherits 
 *              the trajectorybase API.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "vector-interpolation-trajectory.hpp"

namespace _acsl_
{

namespace _trajectory_
{

// Function to initate the interpolation trajectory module
// NOTE: Should only be run after the SetFileName() command is run.
void interpolation::InitiateModule()
{
    // Check if the file exists
    _shared_::_deserialize_::FileExists(this->GetFileName());

    // Read in the data file
    rapidcsv::Document doc(this->GetFileName());

    // Process the csv datafile
    this->m_data.time = _shared_::_deserialize_::GetCSVColumn(doc, "time");

    this->m_data.pos_x = _shared_::_deserialize_::GetCSVColumn(doc, "pos_x");
    this->m_data.pos_y = _shared_::_deserialize_::GetCSVColumn(doc, "pos_y");
    this->m_data.pos_z = _shared_::_deserialize_::GetCSVColumn(doc, "pos_z");

    this->m_data.vel_x = _shared_::_deserialize_::GetCSVColumn(doc, "vel_x");
    this->m_data.vel_y = _shared_::_deserialize_::GetCSVColumn(doc, "vel_y");
    this->m_data.vel_z = _shared_::_deserialize_::GetCSVColumn(doc, "vel_z");

    this->m_data.acc_x = _shared_::_deserialize_::GetCSVColumn(doc, "acc_x");
    this->m_data.acc_y = _shared_::_deserialize_::GetCSVColumn(doc, "acc_y");
    this->m_data.acc_z = _shared_::_deserialize_::GetCSVColumn(doc, "acc_z");

    this->m_data.euler_roll = _shared_::_deserialize_::GetCSVColumn(doc, "euler_roll");
    this->m_data.euler_pitch = _shared_::_deserialize_::GetCSVColumn(doc, "euler_pitch");
    this->m_data.euler_yaw = _shared_::_deserialize_::GetCSVColumn(doc, "euler_yaw");

    this->m_data.euler_rate_r = _shared_::_deserialize_::GetCSVColumn(doc, "euler_rate_r");
    this->m_data.euler_rate_p = _shared_::_deserialize_::GetCSVColumn(doc, "euler_rate_p");
    this->m_data.euler_rate_y = _shared_::_deserialize_::GetCSVColumn(doc, "euler_rate_y");

    this->m_data.euler_acc_r = _shared_::_deserialize_::GetCSVColumn(doc, "euler_acc_r");
    this->m_data.euler_acc_p = _shared_::_deserialize_::GetCSVColumn(doc, "euler_acc_p");
    this->m_data.euler_acc_y = _shared_::_deserialize_::GetCSVColumn(doc, "euler_acc_y");

    this->m_data.quat_w = _shared_::_deserialize_::GetCSVColumn(doc, "quat_w");
    this->m_data.quat_x = _shared_::_deserialize_::GetCSVColumn(doc, "quat_x");
    this->m_data.quat_y = _shared_::_deserialize_::GetCSVColumn(doc, "quat_y");
    this->m_data.quat_z = _shared_::_deserialize_::GetCSVColumn(doc, "quat_z");

    this->m_data.quat_rate_w = _shared_::_deserialize_::GetCSVColumn(doc, "quat_rate_w");
    this->m_data.quat_rate_x = _shared_::_deserialize_::GetCSVColumn(doc, "quat_rate_x");
    this->m_data.quat_rate_y = _shared_::_deserialize_::GetCSVColumn(doc, "quat_rate_y");
    this->m_data.quat_rate_z = _shared_::_deserialize_::GetCSVColumn(doc, "quat_rate_z");

    this->m_data.quat_acc_w = _shared_::_deserialize_::GetCSVColumn(doc, "quat_acc_w");
    this->m_data.quat_acc_x = _shared_::_deserialize_::GetCSVColumn(doc, "quat_acc_x");
    this->m_data.quat_acc_y = _shared_::_deserialize_::GetCSVColumn(doc, "quat_acc_y");
    this->m_data.quat_acc_z = _shared_::_deserialize_::GetCSVColumn(doc, "quat_acc_z");

    this->m_data.ang_vel_x = _shared_::_deserialize_::GetCSVColumn(doc, "ang_vel_x");
    this->m_data.ang_vel_y = _shared_::_deserialize_::GetCSVColumn(doc, "ang_vel_y");
    this->m_data.ang_vel_z = _shared_::_deserialize_::GetCSVColumn(doc, "ang_vel_z");
    
    this->m_data.ang_acc_x = _shared_::_deserialize_::GetCSVColumn(doc, "ang_acc_x");
    this->m_data.ang_acc_y = _shared_::_deserialize_::GetCSVColumn(doc, "ang_acc_y");
    this->m_data.ang_acc_z = _shared_::_deserialize_::GetCSVColumn(doc, "ang_acc_z");

    // Create and assign to interpolators
    this->interp_x = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.pos_x);
    this->interp_y = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.pos_y);
    this->interp_z = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.pos_z);

    this->interp_vx = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.vel_x);
    this->interp_vy = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.vel_y);
    this->interp_vz = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.vel_z);

    this->interp_ax = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.acc_x);
    this->interp_ay = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.acc_y);
    this->interp_az = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.acc_z);
    
    this->interp_eul_roll = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.euler_roll);
    this->interp_eul_pitch = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.euler_pitch);
    this->interp_eul_yaw = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.euler_yaw);

    this->interp_eul_rate_r = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.euler_rate_r);
    this->interp_eul_rate_p = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.euler_rate_p);
    this->interp_eul_rate_y = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.euler_rate_y);

    this->interp_eul_acc_r = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.euler_acc_r);
    this->interp_eul_acc_p = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.euler_acc_p);
    this->interp_eul_acc_y = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.euler_acc_y);

    this->interp_quat_w = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.quat_w);
    this->interp_quat_x = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.quat_x);
    this->interp_quat_y = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.quat_y);
    this->interp_quat_z = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.quat_z);

    this->interp_quat_rate_w = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.quat_rate_w);
    this->interp_quat_rate_x = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.quat_rate_x);
    this->interp_quat_rate_y = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.quat_rate_y);
    this->interp_quat_rate_z = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.quat_rate_z);

    this->interp_quat_acc_w = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.quat_acc_w);
    this->interp_quat_acc_x = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.quat_acc_y);
    this->interp_quat_acc_y = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.quat_acc_x);
    this->interp_quat_acc_z = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.quat_acc_z);

    this->interp_angvel_x = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.ang_vel_x);
    this->interp_angvel_y = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.ang_vel_y);
    this->interp_angvel_z = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.ang_vel_z);

    this->interp_angacc_x = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.ang_acc_x);
    this->interp_angacc_y = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.ang_acc_y);
    this->interp_angacc_z = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.ang_acc_z);

    // Assign the maximum time
    this->SetTmax(_shared_::_compute_::vecmax(m_data.time));

    // Create and assign to nurbsasset
    auto controlpts = _shared_::_serialize_::serialize2ChVector3d(m_data.pos_x, m_data.pos_y, m_data.pos_z);
    this->visualshape = _shared_::_visualize_::createNurbsVisual(controlpts);
    
}

// Function to update the interpolation trajectory module to the current time
// NOTE: Needs to be run every cycle to have the trajectory ready to go.
void interpolation::UpdateModule(const double t)
{
    // Update the time stamp
    this->SetTime(t);

    // Update the trajectory
    this->SetPosition(Eigen::Vector3d(interp_x->interp(&t), 
                                      interp_y->interp(&t), 
                                      interp_z->interp(&t)));

    this->SetVelocity(Eigen::Vector3d(interp_vx->interp(&t), 
                                      interp_vy->interp(&t),
                                      interp_vz->interp(&t)));

    this->SetAcceleration(Eigen::Vector3d(interp_ax->interp(&t),
                                          interp_ay->interp(&t),
                                          interp_az->interp(&t)));

    this->SetEulerAngle(Eigen::Vector3d(interp_eul_roll->interp(&t),
                                        interp_eul_pitch->interp(&t),
                                        interp_eul_yaw->interp(&t)));

    this->SetEulerRate(Eigen::Vector3d(interp_eul_rate_r->interp(&t),
                                       interp_eul_rate_p->interp(&t),
                                       interp_eul_rate_y->interp(&t)));

    this->SetEulerAcc(Eigen::Vector3d(interp_eul_acc_r->interp(&t),
                                      interp_eul_acc_p->interp(&t),
                                      interp_eul_acc_y->interp(&t)));

    this->SetQuat(Eigen::Quaterniond(interp_quat_w->interp(&t),
                                     interp_quat_x->interp(&t),
                                     interp_quat_y->interp(&t),
                                     interp_quat_z->interp(&t)));

    this->SetQuatRate(Eigen::Quaterniond(interp_quat_rate_w->interp(&t),
                                         interp_quat_rate_x->interp(&t),
                                         interp_quat_rate_y->interp(&t),
                                         interp_quat_rate_z->interp(&t)));

    this->SetQuatAcc(Eigen::Quaterniond(interp_quat_acc_w->interp(&t),
                                        interp_quat_acc_x->interp(&t),
                                        interp_quat_acc_y->interp(&t),
                                        interp_quat_acc_z->interp(&t)));

    this->SetAngVel(Eigen::Vector3d(interp_angvel_x->interp(&t),
                                    interp_angvel_y->interp(&t),
                                    interp_angvel_z->interp(&t)));

    this->SetAngAcc(Eigen::Vector3d(interp_angacc_x->interp(&t),
                                    interp_angacc_y->interp(&t),
                                    interp_angacc_z->interp(&t)));
    
}

}   // namespace _trajectory_

}   // namespace _acsl_