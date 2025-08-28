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
// ConfigureSimulatorFromConfig()
//
// Purpose:
//   Loads the main simulator configuration from sim-config.yaml, dynamically selects and instantiates:
//     - The simulation mode (e.g., SIL/HIL setting)
//     - The UAV platform to fly
//     - The environment locale (scene) to use
//   This function configures the simbridge by parsing the YAML config and relying on registry structs
//   (platforms and locales) to update, validate, and instantiate the appropriate UAV and environment classes.
//
// Workflow:
//   1. Open the simulator configuration file (`sim_config_filename`).
//   2. Validate file existence and handle errors if missing.
//   3. Parse the YAML configuration into memory using fkyaml.
//   4. Extract the simulation mode (software/hardware-in-the-loop).
//   5. Populate all available environment locales from YAML and registry.
//      - Validates that one and only one locale is set true.
//      - Instantiates the selected locale via factory (createSelectedLocale).
//   6. Populate all available UAV platforms by dynamically iterating the YAML config and registry.
//      - Validates that one and only one platform is set true.
//      - Instantiates the selected UAV via factory (createSelectedUAV).
//   7. Log success messages about the loaded config, mode, and UAV/locale instantiation.
//
// Notes:
//   - There is no hardcoding of platform or environment names in this function: all lookups and instantiation
//     logic are handled by iterating struct registries and using the YAML keys.
//   - To add a new UAV/platform or environment/locale, only sim-platforms.hpp, sim-locales.hpp, and sim-config.yaml
//     need updating — not this function.
//   - `m_sys` refers to the physics system object; `uav` and `env` are unique_ptrs owned by simbridge.
//   - Errors (e.g., missing config, invalid selection) are reported via SIMULATOR_ERROR and halt execution.
// =====================================================================================================================
void simbridge::ConfigureSimulatorFromConfig()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Open the simulator configuration YAML file
    // ------------------------------------------------------------------------
    std::ifstream ifs(sim_config_filename);

    // ------------------------------------------------------------------------
    // STEP 2 – Check that the file exists
    // ------------------------------------------------------------------------
    if (!ifs) {
        _message_::SIMULATOR_ERROR(
            "[SIMBRG]: COULD NOT OPEN SIMULATOR CONFIG FILE: " + sim_config_filename
        );
    }

    // ------------------------------------------------------------------------
    // STEP 3 – Deserialize the YAML configuration into memory
    // ------------------------------------------------------------------------
    fkyaml::node config_file = fkyaml::node::deserialize(ifs);

    // ------------------------------------------------------------------------
    // STEP 4 – Extract simulation mode
    // ------------------------------------------------------------------------
    this->efsl = config_file["mode"]["enable_flightstack_loop"].as_bool();

    // ------------------------------------------------------------------------
    // STEP 5 – Populate available_locales dynamically from YAML
    //   asVectorRef() iterates all platform bools by name without hardcoding.
    // ------------------------------------------------------------------------
    for (auto& [name, ref] : available_locals.asVectorRef()) {
        ref = config_file["environment"][name].as_bool();
    }

    // ------------------------------------------------------------------------
    // STEP 5.1 – Validate and get the active locale name
    // ------------------------------------------------------------------------
    this->active_locale = available_locals.validateExclusiveSelection();

    // ------------------------------------------------------------------------
    // STEP 5.2 – Instantiate the selected locale using the factory in locales
    //   - Pass in the Chrono physics system from m_sys for ENV construction.
    //   - Store the ENV in simbridge::locale (std::unique_ptr<simenvbase>).
    // ------------------------------------------------------------------------
    this->env = available_locals.createSelectedLocale(this->m_sys.GetPhysicsSystem());

    // ------------------------------------------------------------------------
    // STEP 6 – Populate available_uavs dynamically from YAML
    //   asVectorRef() iterates all platform bools by name without hardcoding.
    // ------------------------------------------------------------------------
    for (auto& [name, ref] : available_uavs.asVectorRef()) {
        ref = config_file["platform"][name].as_bool();
    }

    // ------------------------------------------------------------------------
    // STEP 6.1 – Validate and get the active platform name
    // ------------------------------------------------------------------------
    this->active_platform = available_uavs.validateExclusiveSelection();

    // ------------------------------------------------------------------------
    // STEP 6.2 – Instantiate the selected UAV using the factory in platforms
    //   - Pass in the Chrono physics system from m_sys for UAV construction.
    //   - Store the UAV in simbridge::uav (std::unique_ptr<simuavbase>).
    // ------------------------------------------------------------------------
    this->uav = available_uavs.createSelectedUAV(this->m_sys.GetPhysicsSystem());

    // ------------------------------------------------------------------------
    // STEP 7 – Log the loaded config and UAV instantiation
    // ------------------------------------------------------------------------
    _message_::SIMULATOR_INFO("[SIMBRG]: SIMULATOR CONFIG LOADED SUCCESSFULLY");
    _message_::SIMULATOR_INFO("[SIMBRG]:  - HIL / SIL MODE : " + std::to_string(efsl));
    _message_::SIMULATOR_INFO("[SIMBRG]:  - ACTIVE PLATFORM: " + active_platform);
    _message_::SIMULATOR_INFO("[SIMBRG]:  - ACTIVE LOCALE: " + active_locale);
}

// =====================================================================================================================
// ConfigureSimulatorLog()
//
// Purpose:
//   Creates the directory structure for flight simulation logs, organizing output files by platform and run time.
//   Copies main config files needed for reproducibility into log directory.
//
// Workflow:
//   1. Format current local time as date ("YYYY_MM_DD") and time ("HH_MM_SS") strings.
//   2. Assemble the log directory path using the provided platform name.
//   3. Create the required directory tree recursively using std::filesystem.
//   4. Report errors to stderr if directory creation fails.
//   5. Print the created directory path on success.
//   6. Make a "config" subdirectory under the log directory.
//   7. Copy specific config files to the log/config subdirectory.
//
// Notes:
//   - Only files "phy-config.yaml", "sim-config.yaml", "vis-config.yaml" are copied.
//   - If a copy fails, an error message is printed.
//   - This function is portable and keeps output organized for reproducibility.
// =====================================================================================================================
void simbridge::ConfigureSimulatorLog()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Get current local system time
    // ------------------------------------------------------------------------
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    struct tm tm_result;
    localtime_r(&t, &tm_result);

    // ------------------------------------------------------------------------
    // STEP 2 – Format date as "YYYY_MM_DD" and time as "HH_MM_SS"
    // ------------------------------------------------------------------------
    std::ostringstream dateStream, timeStream;
    dateStream << std::put_time(&tm_result, "%Y_%m_%d");
    timeStream << std::put_time(&tm_result, "%H_%M_%S");
    std::string dateStr = dateStream.str();
    std::string timeStr = timeStream.str();

    // ------------------------------------------------------------------------
    // STEP 3 – Assemble the relative log directory path
    // ------------------------------------------------------------------------
    this->log_dir = "../sim-log";
    this->log_dir /= this->active_platform;
    this->log_dir /= dateStr;
    this->log_dir /= "flight_run_" + timeStr;

    // ------------------------------------------------------------------------
    // STEP 4 – Create the directory tree recursively
    // ------------------------------------------------------------------------
    std::error_code ec;
    std::filesystem::create_directories(this->log_dir, ec);

    // ------------------------------------------------------------------------
    // STEP 5 – Report creation outcome to stdout or stderr
    // ------------------------------------------------------------------------
    if (ec) {
        _message_::SIMULATOR_ERROR("[SIMBRG]: Error creating directories: ", ec.message());
        return;
    } else {
        _message_::SIMULATOR_INFO("[SIMBRG]: Created log directory: " + this->log_dir.string());
    }

    // ------------------------------------------------------------------------
    // STEP 6 – Make sure log/config directory exists
    // ------------------------------------------------------------------------
    std::filesystem::path dst_config = this->log_dir / "config";
    std::error_code ec_config;
    std::filesystem::create_directories(dst_config, ec_config);

    // ------------------------------------------------------------------------
    // STEP 7 – Copy specific config files to log/config
    // ------------------------------------------------------------------------
    const char* files_to_copy[] = {"phy-config.yaml", "sim-config.yaml", "vis-config.yaml"};
    std::filesystem::path src_config_dir = "../config";
    for (const auto& fname : files_to_copy) {
        std::filesystem::path src = src_config_dir / fname;
        std::filesystem::path dst = dst_config / fname;
        std::error_code copy_ec;
        std::filesystem::copy_file(
            src, dst,
            std::filesystem::copy_options::overwrite_existing,
            copy_ec
        );
        if (copy_ec) {
            _message_::SIMULATOR_ERROR(
                "[SIMBRG]: Error copying config file: " + src.string() + " -> " + dst.string(),
                copy_ec.message()
            );
        } else {
            _message_::SIMULATOR_INFO("[SIMBRG]: Copied: " + src.string() + " to " + dst.string());
        }
    }
}


void simbridge::UpdateVisualizationSystem()
{
    // If the visualization system is active
    if (this->m_sys.GetVisConfig().enable_vis)
    {
        // Being the scene
        this->m_sys.GetVisionSystem().BeginScene();

        // Render the scene
        this->m_sys.GetVisionSystem().Render();

        // Render the NED inertial frame of reference if requested
        if (this->m_sys.GetVisConfig().render_ned_frame) {
            this->m_sys.GetVisionSystem().RenderFrame(uav->GetInertialNEDFrame(), 1);
        }

        // Render the NED body frame of reference if requested
        if (this->m_sys.GetVisConfig().render_body_frame) {
            this->m_sys.GetVisionSystem().RenderFrame(uav->GetUAVChassis().body->GetFrameRefToAbs(), 0.6);
        }

        // Render all the propeller frame of reference if requested
        if (this->m_sys.GetVisConfig().render_prop_frames) {
            for (int idx = 1; idx <= this->uav->GetPropCount(); ++idx) {
                this->m_sys.GetVisionSystem().RenderFrame(uav->GetUAVProp(idx).body->GetFrameRefToAbs(), 0.3);
            }
        }

        // Render all the COG frames of reference if requested
        if (this->m_sys.GetVisConfig().render_all_COG_frames) {
            this->m_sys.GetVisionSystem().RenderCOGFrames(1);
        }

        // Enable shadow rendering if requested
        if (this->m_sys.GetVisConfig().render_shadows) {
            // Get all the bodies from the simulator
            auto bodies = this->m_sys.GetPhysicsSystem().GetBodies(); 
            // Enable shadows for all the bodies in the physics system
            for (auto& body : bodies) { this->m_sys.GetVisionSystem().EnableShadows(body); }
        }

        // IF moving camera is enabled them,
        if (!this->m_sys.GetVisConfig().enable_static_cam)
        {
            this->m_sys.GetCamera()->Update(this->m_sys.GetPhyConfig().StepSize);
            this->m_sys.GetVisionSystem().GetActiveCamera()->setPosition(irr::core::vector3dfCH(this->m_sys.GetCamera()->GetCameraPos()));
            this->m_sys.GetVisionSystem().GetActiveCamera()->setTarget(irr::core::vector3dfCH(this->m_sys.GetCamera()->GetTargetPos()));
        }

        // End the scene
        this->m_sys.GetVisionSystem().EndScene();


    }
}


}   // namespace _bridge_

}   // namespace _acsl_