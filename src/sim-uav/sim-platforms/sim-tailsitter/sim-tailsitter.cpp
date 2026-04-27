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
 * File:        sim-tailsitter.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        April 23, 2026
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Derived UAV class for the simulated tailsitter.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-tailsitter.hpp"

namespace _acsl_
{

namespace _tailsitter_
{

// Compulsary derived class function that initiates the chassis with all the necessary parameters
void simtailsitter::ConfigureTAILSITTERChassis()
{
    // Create the struct for processing the uav chassis properties
    _uav_::chassisstruct chassis_properties;

    // Configure the initial position of the chassis ------------ -----------------------------------------------------
    chassis_properties.init_pos = chrono::ChVector3d(0.0626960034167829,0.0398098665855967,0.291715918886598);
    
    // Configure the initial rotation of the chasssis -----------------------------------------------------------------
    chassis_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);
    
    // Configure the mass of the chassis ------------------------------------------------------------------------------
    chassis_properties.mass = 1.54005578358517;

    // Configure the Chassis Inertia vectors --------------------------------------------------------------------------
    chassis_properties.InertiaXX = chrono::ChVector3d(0.144934841382282,0.0467202293377085,0.178211716360287);
    chassis_properties.InertiaXY = chrono::ChVector3d(1.50847693993492e-06,7.33299121677796e-06,-0.00105481465615566);

    // Configure the chassis COM along with it's auxilliary frame at the COM ------------------------------------------
    // This is the most important as we will consider this to be the pixhawk in our subsequent calculations
    // chassis_properties.COM  = chrono::ChFramed(chrono::ChVector3d(-0.0163151142227842,0.0102080920244478,0.017016814810521),
    //                                            chrono::ChQuaternion<>(1,0,0,0)); 

    chassis_properties.COM  = chrono::ChFramed(chrono::ChVector3d(-0.0352349286341472,0.322872417873955,0.186003857246375),
                                               chrono::ChQuaternion<>(1,0,0,0)); 

    // Configure the name of the visualization obj file ---------------------------------------------------------------
    chassis_properties.vis_obj_name = "body_2_1.obj";

    // Configure the chassis collision shapes -------------------------------------------------------------------------
    // Cache some variables for this 
    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();   // Material
    std::shared_ptr<chrono::ChCollisionShape> col;                          // Collision shape
    chrono::ChMatrix33<> mr;                                                // Rotation Matrix for the frame definition
    chrono::ChVector3d pos;                                                 // Position Vector for the frame definition
    chrono::ChVector3d dim;                                                 // Dimensions of the collison box 
    chrono::ChFrame<> frame;                                                // Frame of the collision shape

    // 1
    mr(0,0)=1; mr(1,0)=0;                    mr(2,0)=0;
    mr(0,1)=0; mr(1,1)=-0.00801014722760637; mr(2,1)=0.999967918256077;
    mr(0,2)=0; mr(1,2)=-0.999967918256077;   mr(2,2)=-0.00801014722760637;
    pos = chrono::ChVector3d(-0.0354825354921314,0.32198645845824,0.299999171348472);
    dim = chrono::ChVector3d(0.3655,0.0509962779529972,0.928);
    col = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat,dim);
    frame = chrono::ChFramed(pos, mr);

    chassis_properties.collision.emplace_back(col, frame);

    // 2
    mr(0,0)=-2.19672588431727E-16; mr(1,0)=0.00801014722760712; mr(2,0)=-0.999967918256077;
    mr(0,1)=1;                     mr(1,1)=0;                   mr(2,1)=0;
    mr(0,2)=0;                     mr(1,2)=-0.999967918256077;  mr(2,2)=-0.00801014722760712;
    pos = chrono::ChVector3d(-0.258232535492131,0.322598081802313,0.223645553275023);
    dim = chrono::ChVector3d(0.252699490762865,0.06,1.5);
    col = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat,dim);
    frame = chrono::ChFramed(pos,mr);

    chassis_properties.collision.emplace_back(col, frame);

    // 3
    mr(0,0)=-1;                   mr(1,0)=0;                    mr(2,0)=0;
    mr(0,1)=2.19672588431726E-16; mr(1,1)=-0.00801014722760665; mr(2,1)=0.999967918256077;
    mr(0,2)=0;                    mr(1,2)=0.999967918256077;    mr(2,2)=0.00801014722760665;
    pos = chrono::ChVector3d(0.187267464507868,0.322598081802313,0.223645553275024);
    dim = chrono::ChVector3d(0.06,0.252699490762866,1.5);
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
void simtailsitter::ConfigureTAILSITTERProps()
{
    // Create the struct for processing the uav propeller properties
    _uav_::propstruct prop_properties;

    // ------------------------------------------------ 1 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(0.249963467924819,0.622423734499919,0.389590456554933);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(0.709933147284872,-0.704269072433396,1.37055504022756e-17,1.38932008928742e-17);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.02311480613903;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(0.000243481959493161,2.02448155763798e-06,0.000245240519319778);
    prop_properties.InertiaXY = chrono::ChVector3d(-1.27122136125483e-05,1.01829982031449e-07,1.94838378671284e-06);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-3.39798186774076e-09,-0.00151682657378196,1.12688645520965e-09),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_1_1.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::PURPLE;
    prop_properties.opacity = 0.5;

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(1, prop_properties.init_pos);
    ConfigureUAVPropInitRot(1, prop_properties.init_rot);
    ConfigureUAVPropMass(1, prop_properties.mass);
    ConfigureUAVPropInertiaXX(1, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(1, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(1, prop_properties.COM);
    ConfigureUAVPropOBJName(1, prop_properties.vis_obj_name);
    ConfigureUAVPropColor(1, prop_properties.color);
    ConfigureUAVPropOpacity(1, prop_properties.opacity);
    
    // ------------------------------------------------ 2 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(-0.195536532075518,0.104440352842934,0.38544120029103);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(0.709933147284872,-0.704269072433396,1.37055504022756e-17,1.38932008928742e-17);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.02311480613903;
     prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(0.000243481959493161,2.02448155763798e-06,0.000245240519319778);
    prop_properties.InertiaXY = chrono::ChVector3d(-1.27122136125483e-05,1.01829982031449e-07,1.94838378671284e-06);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-3.39798186774076e-09,-0.00151682657378196,1.12688645520965e-09),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_1_1.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::PURPLE;
    prop_properties.opacity = 0.5;

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(2, prop_properties.init_pos);
    ConfigureUAVPropInitRot(2, prop_properties.init_rot);
    ConfigureUAVPropMass(2, prop_properties.mass);
    ConfigureUAVPropInertiaXX(2, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(2, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(2, prop_properties.COM);
    ConfigureUAVPropOBJName(2, prop_properties.vis_obj_name);
    ConfigureUAVPropColor(2, prop_properties.color);
    ConfigureUAVPropOpacity(2, prop_properties.opacity);

    // ------------------------------------------------ 3 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(0.249963467924483,0.104440352842935,0.385441200291029);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(0.709933147284872,-0.704269072433396,-6.67822392031798e-17,-6.63105858532401e-17);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.02311480613903;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(0.000243481959493161,2.02448155763798e-06,0.000245240519319778);
    prop_properties.InertiaXY = chrono::ChVector3d(-1.27122136125483e-05,1.01829982031449e-07,1.94838378671284e-06);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-3.39798186774076e-09,-0.00151682657378196,1.12688645520965e-09),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_1_1.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::PURPLE;
    prop_properties.opacity = 0.5;

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(3, prop_properties.init_pos);
    ConfigureUAVPropInitRot(3, prop_properties.init_rot);
    ConfigureUAVPropMass(3, prop_properties.mass);
    ConfigureUAVPropInertiaXX(3, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(3, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(3, prop_properties.COM);
    ConfigureUAVPropOBJName(3, prop_properties.vis_obj_name);
    ConfigureUAVPropColor(3, prop_properties.color);
    ConfigureUAVPropOpacity(3, prop_properties.opacity);

    // ------------------------------------------------ 4 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(-0.195536532075182,0.622423734499918,0.389590456554933);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(0.709933147284872,-0.704269072433396,1.37055504022756e-17,1.38932008928742e-17);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.02311480613903;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(0.000243481959493161,2.02448155763798e-06,0.000245240519319778);
    prop_properties.InertiaXY = chrono::ChVector3d(-1.27122136125483e-05,1.01829982031449e-07,1.94838378671284e-06);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-3.39798186774076e-09,-0.00151682657378196,1.12688645520965e-09),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_1_1.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::PURPLE;
    prop_properties.opacity = 0.5;

    // Call all the configuration helper functions
    ConfigureUAVPropInitPos(4, prop_properties.init_pos);
    ConfigureUAVPropInitRot(4, prop_properties.init_rot);
    ConfigureUAVPropMass(4, prop_properties.mass);
    ConfigureUAVPropInertiaXX(4, prop_properties.InertiaXX);
    ConfigureUAVPropInertiaXY(4, prop_properties.InertiaXY);
    ConfigureUAVPropCOM(4, prop_properties.COM);
    ConfigureUAVPropOBJName(4, prop_properties.vis_obj_name);
    ConfigureUAVPropColor(4, prop_properties.color);
    ConfigureUAVPropOpacity(4, prop_properties.opacity);

    // ------------------------------------------------------------------------------ INITIATE THE PROPELLERS
    InitiateUAVProps();
}

// Compulsary derived class function that initiates all the links for the UAV and passes
// that data to the UAV for initialization.
void simtailsitter::ConfigureTAILSITTERLinks()
{
    // Create the link data vector based on the structure that was defined in the sim-uav.hpp
    std::vector<_uav_::LinkData> link_data_vec;
    
    // ---- Concentric1 (chassis to propeller_1) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.249963467924819,0.622411719279077,0.391090408432317),
        chrono::ChVector3d(0.249963467924819,0.62243574972076,0.388090504677549),
        chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077),
        chrono::ChVector3d(-3.90312782094781e-17,0.00801014722760722,-0.999967918256077)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.249963467924819,0.622411719279077,0.391090408432317),
        chrono::ChVector3d(0.249963467924819,0.62243574972076,0.388090504677549),
        chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077),
        chrono::ChVector3d(-3.90312782094781e-17,0.00801014722760722,-0.999967918256077),
        true, true, false, false, false, false
    });

    // ---- Coincident1 (chassis to propeller_1) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.249963467924819,0.622411719279077,0.391090408432317),
        chrono::ChVector3d(0.249963467924819,0.622411719279077,0.391090408432317),
        chrono::ChVector3d(3.90312782094781e-17,-0.00801014722760722,0.999967918256077),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.249963467924819,0.622411719279077,0.391090408432317),
        chrono::ChVector3d(0.249963467924819,0.622411719279077,0.391090408432317),
        chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077),
        chrono::ChVector3d(3.90312782094781e-17,-0.00801014722760722,0.999967918256077)
    });

    // ---- Concentric3 (chassis to propeller_2) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric3", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414),
        chrono::ChVector3d(-0.195536532075518,0.104452368063775,0.383941248413645),
        chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077),
        chrono::ChVector3d(-3.90312782094781e-17,0.00801014722760722,-0.999967918256077)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric3", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414),
        chrono::ChVector3d(-0.195536532075518,0.104452368063775,0.383941248413645),
        chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077),
        chrono::ChVector3d(-3.90312782094781e-17,0.00801014722760722,-0.999967918256077),
        true, true, false, false, false, false
    });

    // ---- Coincident2 (chassis to propeller_2) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414),
        chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414),
        chrono::ChVector3d(3.90312782094781e-17,-0.00801014722760722,0.999967918256077),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414),
        chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414),
        chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077),
        chrono::ChVector3d(3.90312782094781e-17,-0.00801014722760722,0.999967918256077)
    });

    // ---- Concentric4 (chassis to propeller_3) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric4", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(0.249963467924483,0.104428337622093,0.386941152168414),
        chrono::ChVector3d(0.249963467924483,0.104452368063776,0.383941248413645),
        chrono::ChVector3d(1.8821749714349e-16,0.00801014722760746,-0.999967918256077),
        chrono::ChVector3d(1.8821749714349e-16,0.00801014722760746,-0.999967918256077)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric4", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(0.249963467924483,0.104428337622093,0.386941152168414),
        chrono::ChVector3d(0.249963467924483,0.104452368063776,0.383941248413645),
        chrono::ChVector3d(1.8821749714349e-16,0.00801014722760746,-0.999967918256077),
        chrono::ChVector3d(1.8821749714349e-16,0.00801014722760746,-0.999967918256077),
        true, true, false, false, false, false
    });

    // ---- Coincident3 (chassis to propeller_3) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(0.249963467924483,0.104428337622093,0.386941152168414),
        chrono::ChVector3d(0.249963467924483,0.104428337622093,0.386941152168414),
        chrono::ChVector3d(-1.8821749714349e-16,-0.00801014722760746,0.999967918256077),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(0.249963467924483,0.104428337622093,0.386941152168414),
        chrono::ChVector3d(0.249963467924483,0.104428337622093,0.386941152168414),
        chrono::ChVector3d(1.8821749714349e-16,0.00801014722760746,-0.999967918256077),
        chrono::ChVector3d(-1.8821749714349e-16,-0.00801014722760746,0.999967918256077)
    });

    // ---- Concentric5 (chassis to propeller_4) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric5", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.195536532075182,0.622411719279077,0.391090408432317),
        chrono::ChVector3d(-0.195536532075182,0.62243574972076,0.388090504677549),
        chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077),
        chrono::ChVector3d(-3.90312782094781e-17,0.00801014722760722,-0.999967918256077)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric5", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.195536532075182,0.622411719279077,0.391090408432317),
        chrono::ChVector3d(-0.195536532075182,0.62243574972076,0.388090504677549),
        chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077),
        chrono::ChVector3d(-3.90312782094781e-17,0.00801014722760722,-0.999967918256077),
        true, true, false, false, false, false
    });

    // ---- Coincident4 (chasssis to propeller_4) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.195536532075182,0.622411719279077,0.391090408432317),
        chrono::ChVector3d(-0.195536532075182,0.622411719279077,0.391090408432317),
        chrono::ChVector3d(3.90312782094781e-17,-0.00801014722760722,0.999967918256077),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.195536532075182,0.622411719279077,0.391090408432317),
        chrono::ChVector3d(-0.195536532075182,0.622411719279077,0.391090408432317),
        chrono::ChVector3d(-3.90312782094782e-17,0.00801014722760722,-0.999967918256077),
        chrono::ChVector3d(3.90312782094781e-17,-0.00801014722760722,0.999967918256077)
    });


    // Finally link all the bodies together
    LinkUAVBodies(link_data_vec);
}

// Compulsary derived class function that initiates the motors with all the necessary parameters
void simtailsitter::ConfigureTAILSITTERMotors()
{
    // Create the struct for processing the uav motor properties
    _uav_::motorstruct motor_properties;

    // Cache the motor torque constant
    motor_properties.ct = 0.542418958461701;

    // Cache the motor polynomial coefficients
    // Polynomial coefficients vector to evaluate the Commanded Thrust [-] based on the Thrust in Newton
    // TMotor F35A - Velox V2808 Kv1300
    motor_properties.newt2norm = (Eigen::VectorXd(8) << 
                                     2.4591731397119028E-10,
                                    -2.9764216922011841E-8,
                                     1.5256975606726995E-6,
                                    -4.4210707058374142E-5,
                                     0.00081199079192906959,
                                    -0.0098303333146295951,
                                     0.094801218437836871,
                                    -0.012240927327923512
                                 ).finished();

    // Polynomial coefficients vector to evaluate the thrust in [N] based on the Commanded Thrust [-]
    // based on the TMotor F35A - Velox V2808 Kv1300
    motor_properties.norm2newt = (Eigen::VectorXd(8) <<
                                    -1225.1452159827293,
                                     3889.9087791150923,
                                    -4796.6047697858776,
                                     2844.362204073208,
                                    -809.95536988748518,
                                     128.06200953087625,
                                     4.2767571075650777,
                                     0.22158755108870978
                                 ).finished();

    // Polynomial coefficients vector to evaluate the rad/s of motor based on the Commanded Thrust [-]
    // based on the TMotor F35A - Velox V2808 Kv1300                                 
    motor_properties.norm2rps = (Eigen::VectorXd(8) <<
                                    -32359.819235086237,
                                     113883.27012694081,
                                    -158219.57844917616,
                                     108564.74451572458,
                                    -36915.814365961291,
                                     4757.5215597332008,
                                     1069.2500477197036,
                                    -30.701497890560272
                                ).finished();

    // ------------------------------------------------ 1 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CCW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(0.249963467924819,0.622411719279078,0.391090408432317),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(1, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(1, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(1, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(1, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(1, motor_properties.norm2rps);
    ConfigureUAVMotorCt(1, motor_properties.ct);
    
    // ------------------------------------------------ 2 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CCW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-0.195536532075518,0.104428337622092,0.386941152168414),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(2, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(2, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(2, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(2, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(2, motor_properties.norm2rps);
    ConfigureUAVMotorCt(2, motor_properties.ct);
    
    // ------------------------------------------------ 3 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(0.249963467924819,0.104428337622092,0.386941152168414),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(3, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(3, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(3, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(3, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(3, motor_properties.norm2rps);
    ConfigureUAVMotorCt(3, motor_properties.ct);
    
    // ------------------------------------------------ 4 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-0.195536532075518,0.622411719279078,0.391090408432317),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(4, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(4, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(4, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(4, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(4, motor_properties.norm2rps);
    ConfigureUAVMotorCt(4, motor_properties.ct);
    
    // ---------------------------------------------------------------------------------- INITIATE THE MOTORS
    InitiateUAVMotors();
    
}

// Optional derived class function that initiates the aerodynamic properties with all the necessary parameters
void simtailsitter::ConfigureTAILSITTERAerodynamics()
{
    // Set the chassis drag coefficient - We consider this a flat plane
    GetUAVAerodynamics().chassis_drag_coefficient = 1.28; 

    // Set the chassis drag surface area - We consider this to be the area of the flat plane so only
    // measure the top surface area of the chassis - preferably this should be computed in blender
    // with the 3D print toolbox on the acutal exported model .obj file.
    GetUAVAerodynamics().chassis_body_surface_aera = 3.2126;

    // Set the air density for the simulation
    GetUAVAerodynamics().air_density = 1.228;

    // Set the span of a single wing
    GetUAVAerodynamics().aerofoil_span = 1.5;

    // Set the chord of a single wing
    GetUAVAerodynamics().aerofoil_chord = 0.2;

    // Lower Wing                                      
    ConfigureUAVTailSitterWingAeroCenters(1, 10, chrono::ChVector3d(0.249963469370506,-0.38737054221452,0.484704907166303), 
                                                 chrono::ChVector3d(0.249963469370506,1.1125813351696,0.496720128007714));

    // Upper Wing
    ConfigureUAVTailSitterWingAeroCenters(2, 10, chrono::ChVector3d(-0.195536532075349,-0.38737054221452,0.484704907166303), 
                                                 chrono::ChVector3d(-0.195536532075349,1.1125813351696,0.496720128007713));

}

}   // namespace _tailsitter_

}   // namespace _acsl_