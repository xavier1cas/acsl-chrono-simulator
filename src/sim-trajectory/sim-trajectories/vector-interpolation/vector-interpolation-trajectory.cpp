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
    // Read in the data file
    rapidcsv::Document doc(this->GetFileName());

    // Process the csv datafile
    this->m_data.time = _shared_::_deserialize_::GetCSVColumn(doc, "time");
    this->m_data.pos_x = _shared_::_deserialize_::GetCSVColumn(doc, "pos_x");
    this->m_data.pos_y = _shared_::_deserialize_::GetCSVColumn(doc, "pos_y");
    this->m_data.pos_z = _shared_::_deserialize_::GetCSVColumn(doc, "pos_z");

    // Create and assign to interpolators
    auto interpx = _shared_::_compute_::create1DInterpolator(m_data.time, m_data.pos_x);

    // Assign the maximum time

    // Create and assign to nurbsasset
    
}

// Function to update the interpolation trajectory module to the current time
// NOTE: Needs to be run every cycle to have the trajectory ready to go.
void interpolation::UpdateModule(const double t)
{
    // Update the time stamp
    this->SetTime(t);

    // Update the trajectory
    
}

}   // namespace _trajectory_

}   // namespace _acsl_