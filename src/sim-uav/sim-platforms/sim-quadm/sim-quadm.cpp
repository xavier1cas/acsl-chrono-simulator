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
 * File:        sim-quadm.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        August 11, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Derived UAV class for the simulated quadm.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-quadm.hpp"

namespace _acsl_
{

namespace _quadm_
{

// Compulsary derived class function that initiates the chassis with all the necessary parameters
void simquadm::ConfigureQUADMChassis()
{
    // Create the struct for processing the uav chassis properties
    _uav_::chassisstruct chassis_properties;

    // Configure the initial position of the chassis ------------ -----------------------------------------------------
    chassis_properties.init_pos = chrono::ChVector3d(-0.00028213672964037,0.000541686303864703,0.000193471194123856);
    
    // Configure the initial rotation of the chasssis -----------------------------------------------------------------
    chassis_properties.init_rot = chrono::ChQuaternion<>(1,0,0,0);
    
    // Configure the mass of the chassis ------------------------------------------------------------------------------
    chassis_properties.mass = 1.04291239130921;

    // Configure the Chassis Inertia vectors --------------------------------------------------------------------------
    chassis_properties.InertiaXX = chrono::ChVector3d(0.00356507620334993,0.00408659486216925,0.00606507369923718);
    chassis_properties.InertiaXY = chrono::ChVector3d(-4.08770048558901e-06,-1.15603516031003e-05,-8.65500570931156e-07);

    // Configure the chassis COM along with it's auxilliary frame at the COM ------------------------------------------
    // This is the most important as we will consider this to be the pixhawk in our subsequent calculations
    // chassis_properties.COM  = chrono::ChFramed(chrono::ChVector3d(-0.0163151142227842,0.0102080920244478,0.017016814810521),
    //                                            chrono::ChQuaternion<>(1,0,0,0)); 

    chassis_properties.COM  = chrono::ChFramed(chrono::ChVector3d(-0.00149597132207608,-0.0145247411685757,-0.0104019824317649),
                                               chrono::ChQuaternion<>(1,0,0,0)); 

    // Configure the name of the visualization obj file ---------------------------------------------------------------
    chassis_properties.vis_obj_name = "body_1_1.obj";

    // Configure the chassis collision shapes -------------------------------------------------------------------------
    // Cache some variables for this 
    auto mat = chrono_types::make_shared<chrono::ChContactMaterialNSC>();   // Material
    std::shared_ptr<chrono::ChCollisionShape> col;                          // Collision shape
    chrono::ChMatrix33<> mr;                                                // Rotation Matrix for the frame definition
    chrono::ChVector3d pos;                                                 // Position Vector for the frame definition
    chrono::ChVector3d dim;                                                 // Dimensions of the collison box 
    chrono::ChFrame<> frame;                                                // Frame of the collision shape

    // 1
    mr(0,0)=-1; mr(1,0)=0; mr(2,0)=-4.15528994464277E-15;
    mr(0,1)=0; mr(1,1)=-1; mr(2,1)=5.57299002691835E-15;
    mr(0,2)=-4.15528994464277E-15; mr(1,2)=5.57299002691835E-15; mr(2,2)=1;
    pos = chrono::ChVector3d(1.23003869083905E-15,-0.0145000000000017,-0.0134999999999997);
    dim = chrono::ChVector3d(0.243804529519431,0.27143039261605,0.116);
    col = chrono_types::make_shared<chrono::ChCollisionShapeBox>(mat,dim);
    frame = chrono::ChFramed(pos, mr);

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
void simquadm::ConfigureQUADMProps()
{
    // Create the struct for processing the uav propeller properties
    _uav_::propstruct prop_properties;

    // ------------------------------------------------ 1 ------------------------------------------------ //
    // Configure the intial position of the propeller
    prop_properties.init_pos = chrono::ChVector3d(0.0966201280300766,0.0967568826118878,-0.0307815288058759);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(0.707106781186549,0.707106781186546,0,2.54517387617414e-15);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(1.05208970154214e-06,1.0522950943804e-06,2.08055101948593e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(-2.58085779370826e-10,2.41096187585024e-11,-6.37155034566722e-11);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(7.33156025842792e-06,-0.0084495854883779,-7.75309227200287e-06),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_2_1.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::RED;
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
    prop_properties.init_pos = chrono::ChVector3d(-0.0971844014893548,-0.124673510004162,-0.0307815288058759);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(0.707106781186549,0.707106781186546,0,2.39756158981543e-15);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(1.05208970154214e-06,1.0522950943804e-06,2.08055101948593e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(-2.58085779370827e-10,2.4109618758717e-11,-6.37155034569005e-11);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(7.33156025842792e-06,-0.0084495854883779,-7.75309227200287e-06),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_2_1.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
        prop_properties.color = _uav_::_prop_color_::RED;
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
    prop_properties.init_pos = chrono::ChVector3d(0.129661990335559,-0.0737276938909061,-0.0378315288058753);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(0.707106781186546,-0.707106781186549,0,-2.46885013108232e-15);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(1.05254980157665e-06,1.0529278562727e-06,2.08159752619661e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(-7.80398918066972e-11,-1.13859879865916e-11,-2.32585720829613e-12);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-0.033036952915471,0.0013976318628222,-0.0509441435522827),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_3_1.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::ORANGE;
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
    prop_properties.init_pos = chrono::ChVector3d(-0.064142539183858,0.147702698725156,-0.0378315288058766);

    // Configure the initial rotation of the propeller
    prop_properties.init_rot = chrono::ChQuaternion<>(0.707106781186546,-0.707106781186549,0,-2.46885013108236e-15);

    // Configure the mass of the propeller
    // prop_properties.mass = 0.00395402309271749;
    prop_properties.mass = 0.0;

    // Configure the propeller Interia vectors
    prop_properties.InertiaXX = chrono::ChVector3d(1.05254980157665e-06,1.0529278562727e-06,2.08159752619661e-06);
    prop_properties.InertiaXY = chrono::ChVector3d(-7.80398918066972e-11,-1.13859879865916e-11,-2.32585720829613e-12);

    // Configure the propeller COM along with it's auxilliary frame at the COM
    prop_properties.COM = chrono::ChFramed(chrono::ChVector3d(-0.033036952915471,0.0013976318628222,-0.0509441435522827),
                                           chrono::ChQuaternion<>(1,0,0,0));
    
    // Configure the name of the visualization obj file
    prop_properties.vis_obj_name = "body_3_1.obj";

    // There are no collision geometries - Therefore we ignore that step

    // Set the color and opacity values
    prop_properties.color = _uav_::_prop_color_::ORANGE;
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
void simquadm::ConfigureQUADMLinks()
{
    // Create the link data vector based on the structure that was defined in the sim-uav.hpp
    std::vector<_uav_::LinkData> link_data_vec;
    
    // ---- Concentric1 (chassis to propeller_1) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0463065288058759),
        chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0343065288058759),
        chrono::ChVector3d(-3.59941941428318e-15,3.43362833808333e-15,1),
        chrono::ChVector3d(3.59941941428318e-15,-3.39190141383284e-15,-1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0463065288058759),
        chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0343065288058759),
        chrono::ChVector3d(-3.59941941428318e-15,3.43362833808333e-15,1),
        chrono::ChVector3d(3.59941941428318e-15,-3.39190141383284e-15,-1),
        true, true, false, false, false, false
    });

    // ---- Coincident1 (chassis to propeller_1) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.0961849811274249,0.0992187207814498,-0.0343065288058759),
        chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0343065288058759),
        chrono::ChVector3d(-3.59941941428318e-15,3.39190141383284e-15,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident1", GetUAVChassis().body, GetUAVProp(1).body,
        chrono::ChVector3d(0.0961849811274249,0.0992187207814498,-0.0343065288058759),
        chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0343065288058759),
        chrono::ChVector3d(3.59941941428318e-15,-3.43362833808333e-15,-1),
        chrono::ChVector3d(-3.59941941428318e-15,3.39190141383284e-15,1)
    });

    // ---- Concentric2 (chassis to propeller_2) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0463065288058759),
        chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0343065288058759),
        chrono::ChVector3d(-3.39066411694179e-15,3.56100461260611e-15,1),
        chrono::ChVector3d(3.39066411694179e-15,-3.61394601875787e-15,-1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric2", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0463065288058759),
        chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0343065288058759),
        chrono::ChVector3d(-3.39066411694179e-15,3.56100461260611e-15,1),
        chrono::ChVector3d(3.39066411694179e-15,-3.61394601875787e-15,-1),
        true, true, false, false, false, false
    });

    // ---- Coincident2 (chassis to propeller_2) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident4", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-0.0967492545867029,-0.127135348173724,-0.0343065288058759),
        chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0343065288058759),
        chrono::ChVector3d(-3.39066411694179e-15,3.61394601875787e-15,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident4", GetUAVChassis().body, GetUAVProp(2).body,
        chrono::ChVector3d(-0.0967492545867029,-0.127135348173724,-0.0343065288058759),
        chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0343065288058759),
        chrono::ChVector3d(3.39066411694179e-15,-3.56100461260611e-15,-1),
        chrono::ChVector3d(-3.39066411694179e-15,3.61394601875787e-15,1)
    });

    // ---- Concentric3 (chassis to propeller_3) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        false, "Concentric3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0463065288058752),
        chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0343065288058752),
        chrono::ChVector3d(-3.49148133884321e-15,3.49148133884312e-15,1),
        chrono::ChVector3d(-3.49148133884321e-15,3.49148133884313e-15,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0463065288058752),
        chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0343065288058752),
        chrono::ChVector3d(-3.49148133884321e-15,3.49148133884312e-15,1),
        chrono::ChVector3d(-3.49148133884321e-15,3.49148133884313e-15,1),
        true, true, false, false, false, false
    });

    // ---- Coincident3 (chassis to propeller_3) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(0.0990819661996306,-0.125108656906818,-0.0343065288058752),
        chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0343065288058752),
        chrono::ChVector3d(-3.49148133884321e-15,3.49148133884313e-15,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident3", GetUAVChassis().body, GetUAVProp(3).body,
        chrono::ChVector3d(0.0990819661996306,-0.125108656906818,-0.0343065288058752),
        chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0343065288058752),
        chrono::ChVector3d(3.49148133884321e-15,-3.49148133884312e-15,-1),
        chrono::ChVector3d(-3.49148133884321e-15,3.49148133884313e-15,1)
    });

    // ---- Concentric4 (chassis to propeller_4) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Concentric4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0468065288058766),
        chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0343065288058766),
        chrono::ChVector3d(3.49148133884326e-15,-3.4914813388431e-15,-1),
        chrono::ChVector3d(-3.49148133884326e-15,3.49148133884313e-15,1)
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Generic>{
        "Concentric4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0468065288058766),
        chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0343065288058766),
        chrono::ChVector3d(3.49148133884326e-15,-3.4914813388431e-15,-1),
        chrono::ChVector3d(-3.49148133884326e-15,3.49148133884313e-15,1),
        true, true, false, false, false, false
    });

    // ---- Coinciden4 (chassis to propeller_4) ----
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::DistanceZ>{
        "Coincident4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.0996462396589105,0.0971920295145476,-0.0343065288058766),
        chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0343065288058766),
        chrono::ChVector3d(-3.49148133884326e-15,3.49148133884313e-15,1),
        0.0
    });
    link_data_vec.push_back(_uav_::LinkProperty<_uav_::LinkType::Parallel>{
        true, "Coincident4", GetUAVChassis().body, GetUAVProp(4).body,
        chrono::ChVector3d(-0.0996462396589105,0.0971920295145476,-0.0343065288058766),
        chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0343065288058766),
        chrono::ChVector3d(3.49148133884326e-15,-3.4914813388431e-15,-1),
        chrono::ChVector3d(-3.49148133884326e-15,3.49148133884313e-15,1)
    });

    // Finally link all the bodies together
    LinkUAVBodies(link_data_vec);
}

// Compulsary derived class function that initiates the motors with all the necessary parameters
void simquadm::ConfigureQUADMMotors()
{
    // Create the struct for processing the uav motor properties
    _uav_::motorstruct motor_properties;

    // Cache the motor torque constant
    motor_properties.ct = 0.19972258741297011;

    // Cache the rps scaler value 
    motor_properties.rps_scaler = 0.015;

    // Cache the motor polynomial coefficients
    // Polynomial coefficients vector to evaluate the Commanded Thrust [-] based on the Thrust in Newton
    // TMotor F35A - Velox V2808 Kv1300
    motor_properties.newt2norm = (Eigen::VectorXd(8) << 
                                    1.1397548468561201E-5,
                                    -0.00038436172518955446,
                                    0.0051947365478567029,
                                    -0.036003519831726248,
                                    0.13652913911293302,
                                    -0.2857385586886379,
                                    0.40625173135017634,
                                    -0.0586576952028257
                                 ).finished();

    // Polynomial coefficients vector to evaluate the thrust in [N] based on the Commanded Thrust [-]
    // based on the TMotor F35A - Velox V2808 Kv1300
    motor_properties.norm2newt = (Eigen::VectorXd(8) <<
                                    474.38534956372183,
                                    -1496.7377035961833,
                                    1804.2309340905622,
                                    -1061.3133392140433,
                                    316.1068384833909,
                                    -33.102487712309681,
                                    6.2571119597408353,
                                    0.046322637768972989
                                 ).finished();

    // Polynomial coefficients vector to evaluate the rad/s of motor based on the Commanded Thrust [-]
    // based on the TMotor F35A - Velox V2808 Kv1300                                 
    motor_properties.norm2rps = (Eigen::VectorXd(8) <<
                                    -4479.502173784761,
                                    0,
                                    9512.8235882135123,
                                    0,
                                    -6821.6115897712389,
                                    0,
                                    4570.664148941808,
                                    233.24081458152327
                                ).finished();

    // ------------------------------------------------ 1 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CCW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(0.0966201280300767,0.0967568826118877,-0.0468065288058759),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(1, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(1, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(1, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(1, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(1, motor_properties.norm2rps);
    ConfigureUAVMotorCt(1, motor_properties.ct);
    ConfigureUAVMotorRPSScaler(1, motor_properties.rps_scaler);
    
    // ------------------------------------------------ 2 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CCW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-0.0971844014893547,-0.124673510004162,-0.0468065288058758),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(2, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(2, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(2, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(2, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(2, motor_properties.norm2rps);
    ConfigureUAVMotorCt(2, motor_properties.ct);
    ConfigureUAVMotorRPSScaler(2, motor_properties.rps_scaler);

    // ------------------------------------------------ 3 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(0.0966201280300686,-0.124673510004166,-0.0468065288058751),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(3, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(3, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(3, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(3, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(3, motor_properties.norm2rps);
    ConfigureUAVMotorCt(3, motor_properties.ct);
    ConfigureUAVMotorRPSScaler(3, motor_properties.rps_scaler);

    // ------------------------------------------------ 4 ------------------------------------------------ //
    // Configure the spin direction of the motor
    motor_properties.spin_dir = _uav_::_motor_dir_::CW;
    motor_properties.frame = chrono::ChFramed(chrono::ChVector3d(-0.0971844014893485,0.0967568826118956,-0.0468065288058766),
                                              chrono::ChQuaternion<>(1,0,0,0));

    // Call all the configuration helper functions
    ConfigureUAVMotorSpinDir(4, motor_properties.spin_dir);
    ConfigureUAVMotorFrame(4, motor_properties.frame);
    ConfigureUAVMotorNewt2Norm(4, motor_properties.newt2norm);
    ConfigureUAVMotorNorm2Newt(4, motor_properties.norm2newt);
    ConfigureUAVMotorNorm2RPS(4, motor_properties.norm2rps);
    ConfigureUAVMotorCt(4, motor_properties.ct);
    ConfigureUAVMotorRPSScaler(4, motor_properties.rps_scaler);
    
    // ---------------------------------------------------------------------------------- INITIATE THE MOTORS
    InitiateUAVMotors();
    
}

// Optional derived class function that initiates the aerodynamic properties with all the necessary parameters
void simquadm::ConfigureQUADMAerodynamics()
{
    // Set the chassis drag coefficient - We consider this a flat plane
    GetUAVAerodynamics().chassis_drag_coefficient = 1.28; 

    // Set the chassis drag surface area - We consider this to be the area of the flat plane so only
    // measure the top surface area of the chassis - preferably this should be computed in blender
    // with the 3D print toolbox on the acutal exported model .obj file.
    GetUAVAerodynamics().chassis_body_surface_aera = 0.06;

    // Set the air density for the simulation
    GetUAVAerodynamics().air_density = 1.228;

}



}   // namespace _quadm_

}   // namespace _acsl_