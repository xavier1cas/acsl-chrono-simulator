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
 * File:        sim-qrbp.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        August 11, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Derived UAV class for the simulated qrbp.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-qrbp.hpp"

namespace _acsl_
{

namespace _qrbp_
{

// Compulsary derived class function that initiates the chassis with all the necessary parameters
void simqrbp::ConfigureQRBPChassis()
{
    // Configure the initial positiion of the chassis (NED FRAME)
    ConfigureUAVChassisInitPos(chrono::ChVector3d(0.0984471609373536,0.187071795527612,0.17256855449029));

    // Configure the initial rotation of the chasssis (0 ROTATION IN NED FRAME)
    ConfigureUAVChassisInitRot(chrono::ChQuaternion<>(1,0,0,0));

    // Configure the mass of the chassis
    ConfigureUAVChassisMass(1.67236138725861);

    // Configure the Chassis Inertia vectors
    ConfigureUAVChassisInertiaXX(chrono::ChVector3d(0.0233070324786724,0.0160580843956144,0.0271717864469372));
    ConfigureUAVChassisInertiaXY(chrono::ChVector3d(1.10006555036583e-07,-0.000482088295393743,1.23260346821255e-06));

    // Configure the chassis COM alogn with it's auxilliary frame at the COM. This is the most important as we will
    // consider this to be the pixhawk in our subsequent calculations
    ConfigureUAVChassisCOM(chrono::ChFramed(chrono::ChVector3d(-0.0163151142227842,0.0102080920244478,0.017016814810521),
                                            chrono::ChQuaternion<>(1,0,0,0)));

    // Finally tell the UAV class what the name of the chassis obj file is
    ConfigureUAVChassisOBJName("body_3_1.obj");

    // Initiate the UAV chassis
    InitiateUAVChassis();
}


}   // namespace _qrbp_

}   // namespace _acsl_