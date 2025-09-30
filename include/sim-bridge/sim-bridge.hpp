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
 * File:        sim-bridge.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        June 13, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: The brains of the acsl physics simulator. This file is responsible for running the simulation mode
 *              (HIL, SIL, MIL simulations) and loading the appropirate uav into the simulation.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_BRIDGE_HPP_
#define SIM_BRIDGE_HPP_

#include "sim-system.hpp"           // Import the physics and visual system
#include "sim-logger.hpp"           // Import the header file for the Logger API
#include "sim-platforms.hpp"        // Import the header file with all the platforms
#include "sim-uav.hpp"              // Import the header file for the UAV API
#include "sim-locales.hpp"          // Import the header file with all the environments
#include "sim-environment.hpp"      // Import the header file for the ENV API
#include "sim-traj-selection.hpp"   // Import the header file for the Trajectory selection module
#include "sim-trajectory.hpp"       // Import the header file for the Trajectory API

// ##################################################################################
// PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER
// ##################################################################################

#include "pid-omega-qrbp.hpp"


// ##################################################################################
// PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER
// ##################################################################################


namespace _acsl_
{

namespace _bridge_
{

// =====================================================================================================================
// class simbridge
//
// Purpose:
//   - Encapsulates high-level simulation runtime setup.
//   - Reads simulation configuration (YAML) from file.
//   - Determines which simulation mode (HIL, SIL, MIL) to run in.
//   - Instantiates the correct UAV platform and integrates it into the
//     simulation system.
//
// Usage:
//   simlog m_logger;
//   simbridge bridge(m_logger);
//   // automatically reads sim-config.yaml upon construction
//   // later methods will handle UAV platform load & simulation start.
//
// Notes:
//   - Configuration file is expected at: ../config/sim-config.yaml
//   - Actual simulation mode execution logic will be in sim-bridge.cpp.
//   - Now accepts and stores a reference to a simlog instance for use across simulation.
// =====================================================================================================================
class simbridge : public ::_acsl_::_logger_::blackbox
{
public:

    // ------------------------------------------------------------------------
    // Constructor:
    //   - Initializes the core simulation subsystems upon object creation.
    //   - Step 1: Call m_sys.SetupPhysicsSystem() to initialize the Chrono 
    //             physics environment.
    //   - Step 2: Read the simulator configuration file and determine which 
    //             UAV platform and locale should be active - Instantiate the 
    //             selected UAV platform and locale then attach it to the 
    //             initialized physics system.
    //   - Step 3: Call m_sys.SetupVisualizationSystem() to initalize the 
    //             Irrlicht visualization environment.
    //   - Step 4: Start the realtime stepper
    //   - Step 5: Start the physics logger if requested.
    // ------------------------------------------------------------------------
    simbridge(_acsl_::_logger_::simlog& logger)
        : m_logger(logger) // store reference to external logger
    {
        // Initialize the Chrono physics environment
        m_sys.SetupPhysicsSystem();

        // Load config, select platform, and attach it to the physics system
        ConfigureSimulatorFromConfig();

        // Initialize the Chrono visualization environment
        m_sys.SetupVisualizationSystem();

        // Start tracking the realtime
        GetRealtimeStepper().start();

        // Setup the physics log file if requested
        if (this->log2file) { InitiateLogging(); ConfigureHeaders(); }


        // ##################################################################################
        // PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER
        // ##################################################################################

        m_controller = std::unique_ptr< ::_acsl_::_control_::controller_base >( new ::_acsl_::_qrbp_::_pid_omega_::pid_omega(m_logger, *GetTraj()) );

        // ##################################################################################
        // PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER
        // ##################################################################################

    }

    // ------------------------------------------------------------------------
    // Function: Main while loop for the entire simulation system.
    // ------------------------------------------------------------------------
    void EverRun();

private:

    // ------------------------------------------------------------------------
    // Reads YAML settings for the simulator:
    //   - Mode selection (HIL, SIL, MIL)
    //   - UAV platform choice
    //   - Other runtime parameters as needed.
    //
    // Implementation: sim-bridge.cpp
    // ------------------------------------------------------------------------
    void ConfigureSimulatorFromConfig();

    // ------------------------------------------------------------------------
    // Function: Updates the visual system for the acsl physics simulator
    // ------------------------------------------------------------------------
    void UpdateVisualizationSystem();

    // ------------------------------------------------------------------------
    // Function: Updates the physics system for the acsl physics simulator
    // ------------------------------------------------------------------------
    void UpdatePhysicsSystem();

    // ------------------------------------------------------------------------
    // Accessor: Returns a reference to the underlying simulator system class
    // so that external changes can be made.
    // ------------------------------------------------------------------------
    ::_acsl_::_system_::simsystem& GetSimSystem() { return m_sys; }

    // ------------------------------------------------------------------------
    // Accessor: Returns a reference to the UAV unique pointer so that
    // the external code can access or reset the UAV object.
    // ------------------------------------------------------------------------
    std::unique_ptr<::_acsl_::_uav_::simuavbase>& GetUAV() { return uav; }

    // ------------------------------------------------------------------------
    // Accessor: Returns a reference to the ENV unique pointer so that
    // the external code can access or reset the ENV object.
    // ------------------------------------------------------------------------
    std::unique_ptr<::_acsl_::_environment_::simenvbase>& GetEnv() { return env; }

    // ------------------------------------------------------------------------
    // Accessor: Retruns a reference to the Traj unique pointer so that
    // the external code can access or reset the Traj object.
    // ------------------------------------------------------------------------
    std::unique_ptr<::_acsl_::_trajectory_::trajectorybase>& GetTraj() { return trj; }

    // ------------------------------------------------------------------------
    // Accessor: Returns a reference to the real-time stepper object used to
    // track and synchronize the simulation's real-world duration.
    // ------------------------------------------------------------------------
    chrono::ChRealtimeStepTimer& GetRealtimeStepper() { return realtimer; }

    // ------------------------------------------------------------------------
    // Accessor: Returns a reference to the logger instance used by the simulator.
    // ------------------------------------------------------------------------
    _acsl_::_logger_::simlog& GetLogger() { return m_logger; }

    // ------------------------------------------------------------------------
    // Hardcoded path to the simulation configuration YAML.
    // Stores settings for:
    //   - Active simulation mode (model/software/hardware-in-loop)
    //   - Active UAV platform
    // ------------------------------------------------------------------------
    const std::string sim_config_filename = "../config/sim-config.yaml";

    // ------------------------------------------------------------------------
    // Boolean for telling the system if it's in HIL/SIL mode
    // ------------------------------------------------------------------------
    bool efsl;          // <- Stands for enable flightstack loop.

    // ------------------------------------------------------------------------
    // Boolean for telling the system it's debugging configuration
    // ------------------------------------------------------------------------
    bool log2file;
    bool log2terminal;

    // ------------------------------------------------------------------------
    // Chrono Physics and Visual system object.
    // ------------------------------------------------------------------------
    ::_acsl_::_system_::simsystem m_sys;

    // ------------------------------------------------------------------------
    // Structure from sim-platforms that holds all the platforms that are 
    // available.
    // ------------------------------------------------------------------------
    platforms available_uavs;
    std::string active_platform;

    // ------------------------------------------------------------------------
    // Structure from sim-locales that holds all the locales that are 
    // available.
    // ------------------------------------------------------------------------
    locales available_locals;
    std::string active_locale;

    // ------------------------------------------------------------------------
    //  Structure from sim-traj-selection that holds all the trajectories
    // ------------------------------------------------------------------------
    trajectories available_trajectories;
    std::string active_trajectory;

    // ------------------------------------------------------------------------
    // Unique pointer to store the UAV object.
    // ------------------------------------------------------------------------
    std::unique_ptr<::_acsl_::_uav_::simuavbase> uav;

    // ------------------------------------------------------------------------
    // Unique pointer to store the ENV object.
    // ------------------------------------------------------------------------
    std::unique_ptr<::_acsl_::_environment_::simenvbase> env;

    // ------------------------------------------------------------------------
    // Unique pointer to store the TRAJECTORY object.
    // ------------------------------------------------------------------------
    std::unique_ptr<::_acsl_::_trajectory_::trajectorybase> trj;

    // ------------------------------------------------------------------------
    // Realtime stepper object to track the simulation's real life duration
    // ------------------------------------------------------------------------
    chrono::ChRealtimeStepTimer realtimer;

    // ------------------------------------------------------------------------
    // Reference to external simlog logger instance for centralized logging.
    // ------------------------------------------------------------------------
    ::_acsl_::_logger_::simlog& m_logger;

    // ------------------------------------------------------------------------
    // Blackbox functions for the physics logger.
    // ------------------------------------------------------------------------
    void ConfigureHeaders() override;
    bool InitiateLogging() override;
    void LogData() override;

    // ##################################################################################
    // PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER
    // ##################################################################################

    std::unique_ptr<::_acsl_::_control_::controller_base> m_controller;

    // ##################################################################################
    // PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER - PLACEHOLDER
    // ##################################################################################

};





}   // namespace _bridge_

}   // namespace _acsl_



#endif // SIM_BRIDGE_HPP_