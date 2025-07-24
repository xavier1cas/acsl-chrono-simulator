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
 * File:        sim-system.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 15, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class file for the chrono system
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/
#include "sim-system.hpp"

namespace _acsl_
{

namespace _system_
{

// Function to load in the configuration of the physics system
void simsystem::ReadPhysicsConfigFile()
{
    // open a YAML file. Other streams or strings are also usable as an input
    std::ifstream ifs(this->phy_config_filename);

    // Check if the configuration file exists
    if (!ifs) { _message_::SIMULATOR_ERROR("COULD NOT OPEN PHYSICS CONFIG FILE:" + this->phy_config_filename); }

    // deserialize the loaded file contents.
    fkyaml::node config_file = fkyaml::node::deserialize(ifs);

    // Access values and set them as needed
    phyconfig.gravity = config_file["physics"]["gravity"].as_bool();

    phyconfig.PSOR = config_file["solver"]["PSOR"].as_bool();
    phyconfig.MaxIterations = config_file["solver"]["MaxIterations"].as_int();
    phyconfig.EnableWarmStart = config_file["solver"]["EnableWarmStart"].as_bool();

    phyconfig.BULLET = config_file["collision"]["BULLET"].as_bool();
    phyconfig.MULTICORE = config_file["collision"]["MULTICORE"].as_bool();
    phyconfig.DefaultSuggestedEnvelope = static_cast<double>(config_file["collision"]["DefaultSuggestedEnvelope"].as_float());
    phyconfig.DefaultSuggestedMargin = static_cast<double>(config_file["collision"]["DefaultSuggestedMargin"].as_float());
    phyconfig.ContactBreakingThreshold = static_cast<double>(config_file["collision"]["ContactBreakingThreshold"].as_float());
}

// Funciton to load in the configuration of the vision system
void simsystem::ReadVisionConfigFile()
{
    // open a YAML file. Other streams or strings are also usable as an input.
    std::ifstream ifs(this->vis_config_filename);

    // Check if the configuration file exists
    if (!ifs) { _message_::SIMULATOR_ERROR("COULD NOT OPEN VISUALIZATION CONFIG FILE:" + this->vis_config_filename); }

    // deserialize the loaded file contents.
    fkyaml::node config_file = fkyaml::node::deserialize(ifs);

    // Access values and set them as needed
    visconfig.enable_vis = config_file["main"]["enable_vis"].as_bool();
    visconfig.enable_static_cam = config_file["main"]["enable_static_cam"].as_bool();
    visconfig.render_ned_frame = config_file["main"]["render_ned_frame"].as_bool();
    visconfig.render_body_frame = config_file["main"]["render_body_frame"].as_bool();
    visconfig.render_shadows = config_file["main"]["render_shadows"].as_bool();
    visconfig.render_collision_zones = config_file["main"]["render_collision_zones"].as_bool();

    visconfig.width = static_cast<uint>(config_file["window"]["width"].as_int());
    visconfig.height = static_cast<uint>(config_file["window"]["height"].as_int());
    visconfig.title = config_file["window"]["title"].as_str();
}

// Function to setup the chrono physics system
void simsystem::SetupPhysicsSystem()
{
    // Preliminary checks
    if (phyconfig.BULLET && phyconfig.MULTICORE) { _message_::SIMULATOR_ERROR("PICK ONE COLLISION SYSTEM AT A TIME. ENDING SIMULATION"); }

    // Assign the gravity vector
    if (phyconfig.gravity) { m_physics.SetGravitationalAcceleration(_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0,0,9.8))); }
    else { m_physics.SetGravitationalAcceleration(_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0,0,0))); }
            
    // Assign the collision system
    if (phyconfig.BULLET) { m_physics.SetCollisionSystemType(chrono::ChCollisionSystem::Type::BULLET); }
    else if (phyconfig.MULTICORE) { m_physics.SetCollisionSystemType(chrono::ChCollisionSystem::Type::MULTICORE); }
  

    // Assign the physics solver type ----------------------------------------------------------------------------------------------
    if (phyconfig.PSOR) 
    { 
        // Assign the PSOR solver
        auto psor_solver = std::make_shared<chrono::ChSolverPSOR>();
        psor_solver->SetMaxIterations(phyconfig.MaxIterations);
        psor_solver->EnableWarmStart(phyconfig.EnableWarmStart);
        
        // Assign the solver to the object owned by the class
        m_solver = psor_solver;            
    }
    // Add additional else if(config.MINRES) {...} blocks here for other solvers, using appropriate downcast

    // Assign the collision model params -------------------------------------------------------------------------------------------
    chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(phyconfig.DefaultSuggestedEnvelope);
    chrono::ChCollisionModel::SetDefaultSuggestedMargin(phyconfig.DefaultSuggestedMargin);
    chrono::ChCollisionSystemBullet::SetContactBreakingThreshold(phyconfig.ContactBreakingThreshold);

    // Pass on the solver to the system
    m_physics.SetSolver(m_solver);

    _message_::SIMULATOR_INFO("SIMULATOR PHYSICS SYSTEM ACTIVE");
    _message_::SIMULATOR_INFO("STARTING NSC SYSTEM");
    _message_::SIMULATOR_INFO("SYSTEM [GRAVITY]: " + std::to_string(phyconfig.gravity));
    
    _message_::SIMULATOR_INFO("SOLVER [PSOR]: " + std::to_string(phyconfig.PSOR));
    _message_::SIMULATOR_INFO("SOLVER [MaxIterations]: " + std::to_string(phyconfig.MaxIterations));
    _message_::SIMULATOR_INFO("SOLVER [EnableWarmStart]: " + std::to_string(phyconfig.EnableWarmStart));

    _message_::SIMULATOR_INFO("COLLISION [BULLET]: " + std::to_string(phyconfig.BULLET));
    _message_::SIMULATOR_INFO("COLLISION [MULTICORE]: " + std::to_string(phyconfig.MULTICORE));
    _message_::SIMULATOR_INFO("COLLISION [DefaultSuggestedEnvelope]: " + std::to_string(phyconfig.DefaultSuggestedEnvelope));
    _message_::SIMULATOR_INFO("COLLISION [DefaultSuggestedMargin]: " + std::to_string(phyconfig.DefaultSuggestedMargin));
    _message_::SIMULATOR_INFO("COLLISION [ContactBreakingThreshold]: " + std::to_string(phyconfig.ContactBreakingThreshold));

}

// Function to setup the visualization for the physics system
void simsystem::SetupVisualizationSystem()
{
    // If visualization is disabled return
    if (!visconfig.enable_vis) { _message_::SIMULATOR_INFO("VISUALIZATION SYSTEM INACTIVE"); return; }

    // Else setup the visualization
    m_irrlicht.SetWindowSize(800, 600);       // Set the window size
    m_irrlicht.SetWindowTitle(visconfig.title);                        // Set the window title
    m_irrlicht.Initialize();                                           // Initialize the visualization
    m_irrlicht.AddLogo();                                              // Add the Project chrono logo
    m_irrlicht.AddSkyBox();                                            // Add the sky box textures
    m_irrlicht.AddTypicalLights();                                     // Add typical lighting

    // If shadows is enabled
    if (visconfig.render_shadows) { m_irrlicht.EnableShadows(); }
    
    // If static cam is enabled
    if (visconfig.enable_static_cam) { m_irrlicht.AddCamera(chrono::ChVector3d(2, 2, -5), chrono::ChVector3d(0, 1, 0)); }

    // If collision zone renderineg is enabled
    if (visconfig.render_collision_zones) { m_irrlicht.EnableCollisionShapeDrawing(true); }

    // Print messages
    _message_::SIMULATOR_INFO("VISUALIZATION SYSTEM ACTIVE");
    _message_::SIMULATOR_INFO("STARTED WINDOW OF SIZE: " + std::to_string(visconfig.width) + " x " + std::to_string(visconfig.height));
    _message_::SIMULATOR_INFO("STARTED WINDOW WITH TITLE: " + visconfig.title);
    _message_::SIMULATOR_INFO("RENDER NED FRAME: ", visconfig.render_ned_frame);
    _message_::SIMULATOR_INFO("RENDER BODY FRAME: ", visconfig.render_ned_frame);
    _message_::SIMULATOR_INFO("RENDER COLLISION ZONES: ", visconfig.render_collision_zones);
    _message_::SIMULATOR_INFO("RENDER SHADOWS: ", visconfig.render_shadows);
    _message_::SIMULATOR_INFO("ENABLE STATIC CAM: ", visconfig.enable_static_cam);

    // Attach the physics system
    m_irrlicht.AttachSystem(&this->m_physics);

}

}   // namespace _system_

}   // namespace _acsl_