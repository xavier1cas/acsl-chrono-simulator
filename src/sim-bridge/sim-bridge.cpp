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

#include "sim-bridge.hpp"

namespace _acsl_
{

namespace _bridge_
{

// =====================================================================================================================
// ReadSimulatorConfigFile()
//
// Purpose:
//   Loads the core simulator configuration file (sim-config.yaml) that defines:
//     - Simulation mode (model_in_loop, software_in_loop, hardware_in_loop)
//     - Which UAV platform is active/enabled
//   Instantiates the selected UAV platform object and stores it in the simbridge.
//
// Workflow:
//   1. Open file from `sim_config_filename`.
//   2. Validate that the file exists.
//   3. Parse YAML via fkyaml.
//   4. Extract "mode" and populate platform values dynamically via asVectorRef().
//   5. Validate that exactly one platform is active and get its name.
//   6. Use platforms::createSelectedUAV() to instantiate the selected UAV.
//   7. Log the loaded config and the instantiated platform.
//
// Notes:
//   - No hardcoding of platform names here — only sim-platforms.hpp + sim-config.yaml
//     need updates when adding/removing platforms.
//   - `m_sys` in this function refers to your _acsl_::_system_::simsystem instance.
// =====================================================================================================================
void simbridge::ConfigureSimulatorFromConfig()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Open the simulator configuration YAML file
    // ------------------------------------------------------------------------
    std::ifstream ifs(sim_config_filename);

    // STEP 2 – Check that the file exists
    if (!ifs) {
        _message_::SIMULATOR_ERROR(
            "COULD NOT OPEN SIMULATOR CONFIG FILE: " + sim_config_filename
        );
    }

    // ------------------------------------------------------------------------
    // STEP 3 – Deserialize the YAML configuration into memory
    // ------------------------------------------------------------------------
    fkyaml::node config_file = fkyaml::node::deserialize(ifs);

    // ------------------------------------------------------------------------
    // STEP 4 – Extract simulation mode
    // ------------------------------------------------------------------------
    efsl = config_file["mode"]["enable_flightstack_loop"].as_bool();

    // ------------------------------------------------------------------------
    // STEP 5 – Populate available_uavs dynamically from YAML
    //   asVectorRef() iterates all platform bools by name without hardcoding.
    // ------------------------------------------------------------------------
    for (auto& [name, ref] : available_uavs.asVectorRef()) {
        ref = config_file["platform"][name].as_bool();
    }

    // ------------------------------------------------------------------------
    // STEP 5.1 – Validate and get the active platform name
    // ------------------------------------------------------------------------
    std::string active_platform = available_uavs.validateExclusiveSelection();

    // ------------------------------------------------------------------------
    // STEP 6 – Instantiate the selected UAV platform using the factory in platforms
    //   - Pass in the Chrono physics system from m_sys for UAV construction.
    //   - Store the UAV in simbridge::uav (std::unique_ptr<simuavbase>).
    // ------------------------------------------------------------------------
    uav = available_uavs.createSelectedUAV(m_sys.GetPhysicsSystem());

    // ------------------------------------------------------------------------
    // STEP 7 – Log the loaded config and UAV instantiation
    // ------------------------------------------------------------------------
    _message_::SIMULATOR_INFO("SIMULATOR CONFIG LOADED SUCCESSFULLY");
    _message_::SIMULATOR_INFO(" - HIL / SIL MODE : " + std::to_string(efsl));
    _message_::SIMULATOR_INFO(" - ACTIVE PLATFORM: " + active_platform);
}





}   // namespace _bridge_

}   // namespace _acsl_