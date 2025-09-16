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
#include "sim-messages.hpp"

// -----------------------------------------------------------------------------
// Includes for the different trajectory modules (add more as needed)
// -----------------------------------------------------------------------------


namespace _acsl_
{

namespace _bridge_
{

struct traj_vars {
    bool enabled;            // Boolean for the activation of the module
    std::string filename;    // String for the filename to be injested
    std::string filetype;    // String for the filetype to be injested 
};


struct trajectories
{
    traj_vars interp;       // Trajectory variables for the interpolation module
    traj_vars polynomial;   // Trajectory variables for the polynomial module

    // Member function to load values from yaml node
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

    // Member function to return the enabled trajectory module
    std::string GetActiveModule() const {
        if (interp.enabled) { return "Interpolation Trajectory Module"; }
        else if (polynomial.enabled) { return "Minimum Jerk Polynomial Trajectory Module"; }
        else { return "No active module"; }
    }

    // Member funciton to return the filename for the trajectory to be run
    std::string GetTrajectoryFile() const {
        std::string base = "../chrono-assets/trajectories/";
        
        // If the interpolation module is picked 
        if (interp.enabled) { return base + "interpolation/" + interp.filename + "." + interp.filetype; }
        // If the minimum jerk polynomial module is picked
        else if (polynomial.enabled) { return base + "minjerkpoly/" + polynomial.filename + "." + interp.filetype; }
        // Default return 
        else { return "ERR"; }
    }

};




}   // namespace _bridge_
    
}   // namespace _acsl_


#endif // SIM_TRAJ_SELECTION_HPP_

