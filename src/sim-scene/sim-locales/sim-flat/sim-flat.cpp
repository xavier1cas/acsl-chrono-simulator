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


 /**********************************************************************************************************************
 * File:        sim-flat.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        August 11, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Derived environment class for the simulated flat surface.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-flat.hpp"

namespace _acsl_
{

namespace _flat_
{

// Compulsary derived class function that initiates the chassis with all the necessary parameters
void simflat::ConfigureFlatGround()
{
    // Convert the dimensions into the Chrono frame
    auto floorDimAbs = _transformations_::GetChronoPosFromNED(chrono::ChVector3d(FLOOR_LENGTH, FLOOR_HEIGHT, FLOOR_WIDTH));
    auto floorPlacementAbs = _transformations_::GetChronoPosFromNED(chrono::ChVector3d(FLOOR_PLACEMENT_X, FLOOR_PLACEMENT_Y, FLOOR_PLACEMENT_Z));

    // Create the floor body in the NED frame
    auto floorBody = std::make_shared<chrono::ChBodyEasyBox>(floorDimAbs.x(), floorDimAbs.y(), floorDimAbs.z(), 
                                                             FLOOR_DENSITY, FLOOR_VISIBILITY, FLOOR_COLLIDABLE);

    // Set the floors initial position
    floorBody->SetPos(floorPlacementAbs);
    floorBody->SetRot(_acsl_::_transformations_::GetChronoOrientaitonFromNED());
    floorBody->SetFixed(true);
    floorBody->GetVisualShape(0)->SetOpacity(1.0);
    floorBody->GetVisualShape(0)->SetTexture(chrono::GetChronoDataFile("textures/concrete.jpg"), floorDimAbs.x(), floorDimAbs.y());

    // Construct parameter vectors for all environment bodies
    std::vector<std::shared_ptr<chrono::ChBody>> body_vec;

    // Fill data for the floor (index 0)
    body_vec.push_back(floorBody);

    // Pass on the data to the configurator
    ConfigureEnvDefBodyList(body_vec);

}

}   // namespace _flat_

}   // namespace _acsl_