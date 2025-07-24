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
 * File:        sim-system.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 15, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for the chrono system
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_SYSTEM_HPP_
#define SIM_SYSTEM_HPP_

#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/solver/ChSolverPSOR.h"
#include "chrono/collision/bullet/ChCollisionSystemBullet.h"
#include "sim-messages.hpp"
#include "sim-helpers.hpp"
#include "fkYAML/node.hpp"
#include <fstream>
#include <iostream>
#include <string>

namespace _acsl_
{

namespace _system_
{

// Structure for storing the system configurations
struct physicsonfig {
    bool PSOR;
    bool gravity;
    int MaxIterations;
    bool EnableWarmStart;
    bool BULLET;
    bool MULTICORE;
    double DefaultSuggestedEnvelope;
    double DefaultSuggestedMargin;
    double ContactBreakingThreshold;
};

// Structure for storing the visualization configurations
struct visionconfig {
    uint width;
    uint height;
    std::string title;
    bool enable_vis;
    bool enable_static_cam;
    bool render_ned_frame;
    bool render_body_frame;
    bool render_shadows;
    bool render_collision_zones;
};

class simsystem
{

public:
    // Constructor: Sets up the chrono system
    simsystem()
    {
        // Read in the config file at initialization
        ReadPhysicsConfigFile();
        ReadVisionConfigFile();
    }

    // Function to setup the chrono physics system
    void SetupPhysicsSystem();

    // Function to setup the visualization for the physics system
    void SetupVisualizationSystem();

    // Accessor for the underlying chrono system
    chrono::ChSystemNSC& GetPhysicsSystem() { return m_physics; }

    // Accessor for the underlying irrlicht system
    chrono::irrlicht::ChVisualSystemIrrlicht& GetVisionSystem() { return m_irrlicht; }

    // Accessor for the configuration of the system
    physicsonfig GetPhyConfig() { return phyconfig; }

    // Accessor for the configuration of the vision system
    visionconfig GetVisConfig() { return visconfig; }

private:
    // Hardcoded configuration filename for the system config
    const std::string& phy_config_filename = "../config/phy-config.yaml";

    // Hardcoded configuration filename for the vision config
    const std::string& vis_config_filename = "../config/vis-config.yaml";

    // Create a chrono system
    chrono::ChSystemNSC m_physics;

    // Create the irrlicht visualization system object
    chrono::irrlicht::ChVisualSystemIrrlicht m_irrlicht;

    // Create the solver
    std::shared_ptr<chrono::ChSolver> m_solver;

    // Create a system config structure object
    physicsonfig phyconfig;

    // Createa a vision config structure object
    visionconfig visconfig;

    // Function to load in the configuration of the physics system
    void ReadPhysicsConfigFile();

    // Function to load in the configuraion of the visualization system
    void ReadVisionConfigFile();

};


}   // namespace _system_

}   // namespace _acsl_

#endif  // SIM_SYSTEM_HPP_