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
    // Create the struct for processing the uav chassis properties
    ::_acsl_::_uav_::chassisstruct chassis_properties;

    // Configure the initial position of the chassis ------------ -----------------------------------------------------
    chassis_properties.init_pos = chrono::ChVector3d(0.0984471609373536,0.187071795527612,0.17256855449029);
    
    // Configure the initial rotation of the chasssis -----------------------------------------------------------------
    chassis_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);
    
    // Configure the mass of the chassis ------------------------------------------------------------------------------
    chassis_properties.mass = 1.67236138725861;

    // Configure the Chassis Inertia vectors --------------------------------------------------------------------------
    chassis_properties.InertiaXX = chrono::ChVector3d(0.0233070324786724,0.0160580843956144,0.0271717864469372);
    chassis_properties.InertiaXY = chrono::ChVector3d(1.10006555036583e-07,-0.000482088295393743,1.23260346821255e-06);

    // Configure the chassis COM along with it's auxilliary frame at the COM ------------------------------------------
    // This is the most important as we will consider this to be the pixhawk in our subsequent calculations
    chassis_properties.COM  = chrono::ChFramed(chrono::ChVector3d(-0.0163151142227842,0.0102080920244478,0.017016814810521),
                                               chrono::ChQuaternion<>(1,0,0,0));                            

    // Configure the name of the visualization obj file ---------------------------------------------------------------
    chassis_properties.vis_obj_name = "body_3_1.obj";

    // Configure the chassis collision shapes -------------------------------------------------------------------------
    // Cache some variables for this 
    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();   // Material
    std::shared_ptr<chrono::ChCollisionShape> col;                          // Collision shape
    chrono::ChMatrix33<> mr;                                                // Rotation Matrix for the frame definition
    chrono::ChVector3d pos;                                                 // Position Vector for the frame definition
    chrono::ChVector3d dim;                                                 // Dimensions of the collison box 
    chrono::ChFrame<> frame;                                                // Frame of the collision shape

    // 1
    mr(0,0)=-1;                    mr(1,0)=-3.38384126574542E-15; mr(2,0)=0;
    mr(0,1)=1.13691526083878E-16;  mr(1,1)=-7.04887461720045E-15; mr(2,1)=1;
    mr(0,2)=-3.38384126574542E-15; mr(1,2)=1;                     mr(2,2)=7.04887461720045E-15;
    pos = chrono::ChVector3d(-0.107892648208808,0.0102797625509045,0.0890880486150274);
    dim = chrono::ChVector3d(0.0164047740043827,0.122065278616947,0.5);
    col = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat,dim);
    frame = chrono::ChFramed(pos, mr);

    chassis_properties.collision.emplace_back(col, frame);

    // 2
    mr(0,0)=-1;                   mr(1,0)=1.85656158318431E-16; mr(2,0)=-3.37275354278483E-15;
    mr(0,1)=0;                    mr(1,1)=1;                    mr(2,1)=5.73186685091805E-15;
    mr(0,2)=3.37275354278483E-15; mr(1,2)=5.73186685091805E-15; mr(2,2)=-1;
    pos = chrono::ChVector3d(-0.0109730630310874,0.0102812455678535,-0.0391445906934458);
    dim = chrono::ChVector3d(0.224249837982941,0.265117422382603,0.114399999999996);
    col = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat,dim);
    frame = chrono::ChFramed(pos,mr);

    chassis_properties.collision.emplace_back(col, frame);

    // 3
    mr(0,0)=-1;                    mr(1,0)=-3.38384126574542E-15; mr(2,0)=0;
    mr(0,1)=-1.13691526083878E-16; mr(1,1)=6.82149156503269E-15;  mr(2,1)=-1;
    mr(0,2)=3.38384126574542E-15;  mr(1,2)=-1;                    mr(2,2)=-6.82149156503269E-15;
    pos = chrono::ChVector3d(0.0859073517911931,0.0102797625509052,0.0890880486150245);
    dim = chrono::ChVector3d(0.0164047740043827,0.122065278616947,0.5);
    col = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat,dim);
    frame = chrono::ChFramed(pos,mr);
    
    chassis_properties.collision.emplace_back(col, frame);

    // Call all the configuration helper functions --------------------------------------------------------------------
    ConfigureUAVChassisInitPos(chassis_properties.init_pos);
    ConfigureUAVChassisInitRot(chassis_properties.init_rot);
    ConfigureUAVChassisMass(chassis_properties.mass);
    ConfigureUAVChassisInertiaXX(chassis_properties.InertiaXX);
    ConfigureUAVChassisInertiaXY(chassis_properties.InertiaXY);                                               
    ConfigureUAVChassisCOM(chassis_properties.COM);
    ConfigureUAVChassisOBJName(chassis_properties.vis_obj_name);
    ConfigureUAVChassisCollisionShapes(chassis_properties.collision);

    // Initiate the UAV chassis ---------------------------------------------------------------------------------------
    InitiateUAVChassis();
}

// Compulsary derived class function that initiates the propellers with all the necessary parameters
void simqrbp::ConfigureQRBPProps()
{
    // Create the struct for processing the uav propeller properties
    ::_acsl_::_uav_::propstruct prop_properties;

    // ------------------------------------------------ 1 ------------------------------------------------ //
    // Configure the intial position of the propeller

    // Configure the initial rotation of the propeller

    // Configure the mass of the propeller

    // Configure the propeller Interia vectors

    // Configure the propeller COM along with it's auxilliary frame at the COM
    
    // Configure the name of the visualization obj file

    // There are no collision geometries - Therefore we ignore that step

    // Call all the configuration helper functions
    
    // ------------------------------------------------ 2 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(-0.00942816685338318,0.086637844787446,0.154498963796845);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    prop_properties.mass = 0.00395402309271749;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(2.83955219476957e-06,2.84026926028763e-06,5.66580058802151e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(7.53921615234973e-10,-1.55407912210431e-11,-5.50513541652702e-11);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_2_1.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(2, prop_properties.init_pos);
    ConfigureUAVPropInitRot(2, prop_properties.init_rot);
    ConfigureUAVPropMass(2, prop_properties.mass);
    ConfigureUAVPropInertiaXX(2, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(2, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(2, prop_properties.COM);
    ConfigureUAVPropOBJName(2, prop_properties.vis_obj_name);

    // ------------------------------------------------ 3 ------------------------------------------------ //
    // Configure the intial position of the propeller
    
    // Configure the initial rotation of the propeller

    // Configure the mass of the propeller

    // Configure the propeller Interia vectors

    // Configure the propeller COM along with it's auxilliary frame at the COM
    
    // Configure the name of the visualization obj file

    // There are no collision geometries - Therefore we ignore that step

    // Call all the configuration helper functions

    // ------------------------------------------------ 4 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(-0.00942816685338454,0.308068237403483,0.154498963796847);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);

    // Configure the mass of the propeller
    prop_properties.mass = 0.00395402309271749;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(2.83955219476957e-06,2.84026926028763e-06,5.66580058802151e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(7.53921615234973e-10,-1.55407912210431e-11,-5.50513541652702e-11);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-5.52286503295572e-10,3.32677464985852e-06,-0.000645546072066471),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_1_1.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(4, prop_properties.init_pos);
    ConfigureUAVPropInitRot(4, prop_properties.init_rot);
    ConfigureUAVPropMass(4, prop_properties.mass);
    ConfigureUAVPropInertiaXX(4, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(4, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(4, prop_properties.COM);
    ConfigureUAVPropOBJName(4, prop_properties.vis_obj_name);

    // ------------------------------------------------------------------------------ INITIATE THE PROPELLERS
    InitiateUAVProp();
}

}   // namespace _qrbp_

}   // namespace _acsl_