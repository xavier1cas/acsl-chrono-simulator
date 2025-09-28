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
 * File:        sim-traj-selection.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        Sep 13, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header only file to include all the trajectory modules.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_TRAJ_SELECTION_HPP_
#define SIM_TRAJ_SELECTION_HPP_

#include <string>
#include "fkYAML/node.hpp"
#include "sim-helpers.hpp"
#include "sim-trajectory.hpp"
#include "piecewise-polynomial-trajectory.hpp"
#include "vector-interpolation-trajectory.hpp"

// -----------------------------------------------------------------------------
// Includes for the different trajectory modules (add more as needed)
// -----------------------------------------------------------------------------


namespace _acsl_
{

namespace _bridge_
{

// -----------------------------------------------------------------------------
// Structure: traj_vars
//
// Holds configuration fields for a single trajectory module.
// Members:
//   enabled   - Set to true to activate the module for simulation/control.
//   filename  - Name of the trajectory input file (without path or extension).
//   filetype  - Expected file extension/type for the trajectory data file.
// -----------------------------------------------------------------------------
struct traj_vars {
    bool enabled;            // Boolean for the activation of the module
    std::string filename;    // String for the filename to be ingested
    std::string filetype;    // String for the filetype to be ingested 
};


// -----------------------------------------------------------------------------
// Structure: trajectories
//
// Purpose:
//   Aggregates all trajectory modules for selection.
//   Supports modular YAML configuration parsing and dynamic control logic.
//
// Members:
//   interp     - Trajectory variables for the interpolation trajectory module.
//   polynomial - Trajectory variables for the minimum jerk polynomial
//                trajectory module.
//
// API:
//   - read(config): Loads YAML settings for all modules, ensures only one is
//                   enabled, and checks filetype rules.
//   - GetActiveModule(): Returns the currently enabled trajectory module as
//                        a readable string.
//   - GetTrajectoryFile(): Constructs and returns the expected file path for 
//                          the selected module's trajectory.
// -----------------------------------------------------------------------------
struct trajectories
{
    traj_vars interp;       // Trajectory variables for the interpolation module
    traj_vars polynomial;   // Trajectory variables for the polynomial module


    // -----------------------------------------------------------------------------
    // Reads and validates YAML configuration for all trajectory modules.
    // Ensures only one trajectory module is enabled and checks filetype constraints.
    // Throws/logs error if misconfigured.
    // -----------------------------------------------------------------------------
    void read(const fkyaml::node& config) {
        // Parameters for the interpolation trajectory
        const auto& interp_node = config.at("interp");
        interp.enabled = interp_node.at("enabled").get_value<bool>();
        interp.filename = interp_node.at("filename").get_value<std::string>();
        interp.filetype = interp_node.at("filetype").get_value<std::string>();

        // Parameters for the minimum jerk polynomial trajectory
        const auto& poly_node = config.at("poly");
        polynomial.enabled = poly_node.at("enabled").get_value<bool>();
        polynomial.filename = poly_node.at("filename").get_value<std::string>();
        polynomial.filetype = poly_node.at("filetype").get_value<std::string>();

        // IF more than one module is active
        if (interp.enabled && polynomial.enabled) {
            _message_::SIMULATOR_ERROR("[SIMBRG]: BOTH TRAJECTORY MODULES ACTIVE! PICK ONE TRAJECTORY MODULE");
        }

        // IF interp.filetype is not csv, throw an error
        if (interp.enabled && interp.filetype != "csv") {
            _message_::SIMULATOR_ERROR("[SIMBRG]: INTERP MODULE REQUIRES FILETYPE 'CSV'");
        }

        // IF polynomial.filetype is not json, throw an error
        if (polynomial.enabled && polynomial.filetype != "json") {
            _message_::SIMULATOR_ERROR("[SIMBRG]: POLYNOMIAL MODULE REQUIRES FILETYPE 'JSON'");
        }
    }


    // -----------------------------------------------------------------------------
    // Returns the name of the enabled trajectory module for reporting or control logic.
    // -----------------------------------------------------------------------------
    std::string GetActiveModule() const {
        if (interp.enabled) { return "Interpolation Trajectory Module"; }
        else if (polynomial.enabled) { return "Minimum Jerk Polynomial Trajectory Module"; }
        else { return "No active module"; }
    }

    // -----------------------------------------------------------------------------
    // Constructs the full file path for the enabled trajectory module's input file.
    // Returns "ERR" if none is enabled.
    // -----------------------------------------------------------------------------
    std::string GetTrajectoryFile() const {
        std::string base = "../chrono-assets/trajectories/";
        
        // If the interpolation module is picked 
        if (interp.enabled) { return base + "interpolation/" + interp.filename + "." + interp.filetype; }
        // If the minimum jerk polynomial module is picked
        else if (polynomial.enabled) { return base + "minjerkpoly/" + polynomial.filename + "." + polynomial.filetype; }
        // Default return 
        else { return "ERR"; }
    }

    // -------------------------------------------------------------------------
    // Factory: createSelectedModule()
    //
    // Purpose:
    //   Instantiate the trajectory object corresponding to the active module and
    //   return it as a polymorphic std::unique_ptr<trajectorybase>.
    //
    // Returns:
    //   std::unique_ptr<_acsl_::_uav_::trajectorybase> pointing to
    //   the constructed trajectory module instance.
    //
    // Workflow:
    //   1. Based on which boolean is true, construct the matching module.
    //   3. Return the base-class pointer (trajectorybase) to the caller.
    //
    // Notes:
    //   - Requires that the derived trajectory class headers are included
    //     in sim-traj-selection.hpp.
    // -------------------------------------------------------------------------
    std::unique_ptr< ::_acsl_::_trajectory_::trajectorybase >
    createSelectedModule() const {
        if (interp.enabled) {
            _message_::SIMULATOR_INFO("[SIMBRG]: ATTACHING VECTOR INTERP TRAJ MODULE TO SYS");
            return std::unique_ptr< ::_acsl_::_trajectory_::trajectorybase >( new ::_acsl_::_trajectory_::interpolation );
        }
        else if (polynomial.enabled) {
            _message_::SIMULATOR_INFO("[SIMBRG]: ATTACHING MINJERKPOLY TRAJ MODULE TO SYS");
            return std::unique_ptr< ::_acsl_::_trajectory_::trajectorybase >( new ::_acsl_::_trajectory_::minjerkpoly );
        }

        // Default retrun type
        return nullptr;

    }
};



}   // namespace _bridge_
    
}   // namespace _acsl_


#endif // SIM_TRAJ_SELECTION_HPP_

