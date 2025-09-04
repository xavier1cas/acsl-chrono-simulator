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
//   4. Extract the simulation mode (software/hardware-in-the-loop) and debugging settings.
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
    // STEP 4 – Extract simulation mode and debuging settings
    // ------------------------------------------------------------------------
    this->efsl = config_file["mode"]["enable_flightstack_loop"].as_bool();
    this->log2file = config_file["debug"]["log_physics"].as_bool();
    this->log2terminal = config_file["debug"]["terminal"].as_bool();

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
// UpdateVisualizationSystem()
//
// Purpose:
//   Updates and renders the visualization for the simulator according to visualization config settings.
//   Handles scene setup, frame and propeller rendering, COG visualization, shadows, and camera movement.
//   Ensures every requested visual component is displayed and synchronized for the current simulation step.
//
// Workflow:
//   1. Check if visualization is enabled in the simsystem configuration.
//   2. Begin the visualization scene for this simulation step.
//   3. Render the full scene (all core visuals).
//   4. If enabled, render the inertial NED frame visualization.
//   5. If enabled, render the UAV body frame visualization.
//   6. If enabled, render frames for all UAV propellers.
//   7. If enabled, render all center-of-gravity (COG) frames.
//   8. If enabled, activate shadow rendering for all physics bodies.
//   9. If moving camera is enabled, update camera position and target for live-following the vehicle.
//  10. End the visualization scene, finalizing render output for the step.
//
// Notes:
//   - All rendering options are enabled or disabled via the visualization config (vis-config.yaml).
//   - All visuals are triggered in sequence within a single scene per time step.
//   - Camera and shadow updates provide enhanced realism and real-time following in visuals.
//   - Visualization may be completely skipped if not enabled, preserving compute resources.
// =====================================================================================================================
void simbridge::UpdateVisualizationSystem()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Check if visualization is enabled for this simulator run.
    // ------------------------------------------------------------------------
    if (this->m_sys.GetVisConfig().enable_vis)
    {
        // --------------------------------------------------------------------
        // STEP 2 – Begin a new scene to set up rendering for this step.
        // --------------------------------------------------------------------
        this->m_sys.GetVisionSystem().BeginScene();

        // --------------------------------------------------------------------
        // STEP 3 – Render the main simulator scene visuals.
        // --------------------------------------------------------------------
        this->m_sys.GetVisionSystem().Render();

        // --------------------------------------------------------------------
        // STEP 4 – If enabled, render the inertial NED frame in the visualization.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_ned_frame) {
            this->m_sys.GetVisionSystem().RenderFrame(uav->GetInertialNEDFrame(), 1);
        }

        // --------------------------------------------------------------------
        // STEP 5 – If enabled, render the body frame of the UAV.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_body_frame) {
            this->m_sys.GetVisionSystem().RenderFrame(uav->GetUAVChassis().body->GetFrameRefToAbs(), 0.6);
        }

        // --------------------------------------------------------------------
        // STEP 6 – If enabled, render frames for all UAV propellers.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_prop_frames) {
            for (int idx = 1; idx <= this->uav->GetPropCount(); ++idx) {
                this->m_sys.GetVisionSystem().RenderFrame(uav->GetUAVProp(idx).body->GetFrameRefToAbs(), 0.3);
            }
        }

        // --------------------------------------------------------------------
        // STEP 7 – If enabled, render all center-of-gravity (COG) frames.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_all_COG_frames) {
            this->m_sys.GetVisionSystem().RenderCOGFrames(1);
        }

        // --------------------------------------------------------------------
        // STEP 8 – If enabled, activate shadow rendering for all physics bodies.
        // --------------------------------------------------------------------
        if (this->m_sys.GetVisConfig().render_shadows) {
            // Get all bodies from the simulator and activate shadow rendering.
            auto bodies = this->m_sys.GetPhysicsSystem().GetBodies(); 
            for (auto& body : bodies) {
                this->m_sys.GetVisionSystem().EnableShadows(body);
            }
        }

        // --------------------------------------------------------------------
        // STEP 9 – If the moving camera is enabled, update its position and target for real-time following.
        // --------------------------------------------------------------------
        if (!this->m_sys.GetVisConfig().enable_static_cam)
        {
            this->m_sys.GetCamera()->Update(this->m_sys.GetPhyConfig().StepSize);
            this->m_sys.GetVisionSystem().GetActiveCamera()->setPosition(
                irr::core::vector3dfCH(this->m_sys.GetCamera()->GetCameraPos()));
            this->m_sys.GetVisionSystem().GetActiveCamera()->setTarget(
                irr::core::vector3dfCH(this->m_sys.GetCamera()->GetTargetPos()));
        }

        // --------------------------------------------------------------------
        // STEP 10 – End the scene; finalize rendering for this simulation step.
        // --------------------------------------------------------------------
        this->m_sys.GetVisionSystem().EndScene();
    }
}


// =====================================================================================================================
// UpdatePhysicsSystem()
//
// Purpose:
//   Advances the physics simulation for the current step, manages real-time synchronization,
//   retrieves the UAV state, and prints color-coded state information to the terminal (if enabled).
//
// Workflow:
//   1. Perform one physics simulation time step using Chrono with the configured step size.
//   2. Maintain soft real-time pacing by spinning until step duration matches target (avoids CPU overrun).
//   3. Query the UAV's current states (position, velocity, rotation, etc.).
//   4. If terminal logging is enabled, format and print the state block in color for operator readability.
//
// Notes:
//   - Real-time spinning helps match simulation wall-clock to desired step size, especially in live/interactive modes.
//   - State block includes simulation time, position, velocity, Euler angles, angular velocity, forces, and torques.
//   - Color-coding improves operational clarity in high-frequency terminal output scenarios.
//   - Updates are performed every simulation tick; rapid terminal output may occur if log2terminal is set true.
// =====================================================================================================================
void simbridge::UpdatePhysicsSystem()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Advance Chrono physics simulation by one step using config step size.
    // ------------------------------------------------------------------------
    this->m_sys.GetPhysicsSystem().DoStepDynamics(this->m_sys.GetPhyConfig().StepSize);

    // ------------------------------------------------------------------------
    // STEP 2 – Spin in place to maintain soft real-time pacing for this simulation tick.
    // ------------------------------------------------------------------------
    this->realtimer.Spin(this->m_sys.GetPhyConfig().StepSize);
    
    // ------------------------------------------------------------------------
    // STEP 3 – Extract current UAV state variables for reporting and logging.
    // ------------------------------------------------------------------------
    auto states = this->uav->GetUAVStateData();

    // ------------------------------------------------------------------------
    // STEP 4 – If terminal logging is enabled, print state block in color.
    // ------------------------------------------------------------------------
    if (this->log2terminal)
    {
        std::string color_label = "\033[38;5;214m";  // Bright Orange for labels
        std::string color_value = "\033[1;36m";      // Bright/Bold Cyan for values
        std::string color_reset = "\033[0m";         // Reset to default

        std::ostringstream msg;
        msg << "\n"
            << color_label << "SIMULATION TIME: " << color_value << states.time << " s\n" << color_reset
            << color_label << "UAV POSITION IN NED FRAME: " << color_value
            << states.pos.x() << ", "
            << states.pos.y() << ", "
            << states.pos.z() << "\n" << color_reset
            << color_label << "UAV VELOCITY IN NED FRAME: " << color_value
            << states.vel.x() << ", "
            << states.vel.y() << ", "
            << states.vel.z() << "\n" << color_reset
            << color_label << "UAV rotation in NED frame: " << color_value
            << _acsl_::_conversions_::rad2deg(states.eul.x()) << ", "
            << _acsl_::_conversions_::rad2deg(states.eul.y()) << ", "
            << _acsl_::_conversions_::rad2deg(states.eul.z()) << "\n" << color_reset
            << color_label << "UAV ANGULAR VELOCITY IN NED FRAME: " << color_value
            << states.ovel.x() << ", "
            << states.ovel.y() << ", "
            << states.ovel.z() << "\n" << color_reset
            << color_label << "UAV FORCES IN NED FRAME [I]: " << color_value
            << states.muI.x() << ", "
            << states.muI.y() << ", "
            << states.muI.z() << "\n" << color_reset
            << color_label << "UAV FORCES IN NED FRAME [J]: " << color_value
            << states.muJ.x() << ", "
            << states.muJ.y() << ", "
            << states.muJ.z() << "\n" << color_reset
            << color_label << "UAV TORQUES IN NED FRAME: " << color_value
            << states.tauJ.x() << ", "
            << states.tauJ.y() << ", "
            << states.tauJ.z() << color_reset;
        
        // Print the colorized output to the terminal.
        std::cout << msg.str() << std::endl;
    }

    // ------------------------------------------------------------------------
    // STEP 4 – If file logging is enabled, format and write to file.
    // ------------------------------------------------------------------------
    if (this->log2file) { this->LogData(); }
}



// =====================================================================================================================
// InitiateLogging()
//
// Purpose:
//   Initialize the Physics logger using Boost.Log with a formatted synchronous text sink targeting a file.
//   Sets logging attributes, ensures the log file is created and opened, and applies filtering to only capture the
//   physics data from the chrono system's log data.
//
// Workflow:
//   1. Construct the log file path and verify file can be opened for writing.
//   2. Add a "Tag" attribute to logger for log filtering.
//   3. Define and configure a synchronous text sink pointing to the file.
//   4. Set up formatting to include timestamp, thread/process IDs, tag, line ID, and the message.
//   5. Register the sink with Boost.Log's core.
//   6. Apply a filter to restrict logs by tag value.
//   7. Add Boost common attributes for completeness.
//   8. Return true on success; catch and report exceptions or file errors, returning false.
//
// Notes:
//   - Ensures file is writable before adding as stream.
//   - Reports detailed errors upon any failure.
//   - Ensures log attributes/sinks are not redundantly registered.
//   - Scalable; consider async sinks for high-frequency or multi-threaded logging.
// =====================================================================================================================
bool simbridge::InitiateLogging() {
    try {
        // --------------------------------------------------------------------
        // STEP 1 – Build log file path and check file accessibility.
        // --------------------------------------------------------------------
        std::stringstream log_ss;
        log_ss << this->m_logger.GetLogDirectory().string() << "/physics.log";
        std::string physics_log_filename = log_ss.str();

        // Prepare the log file stream
        auto file_stream = boost::make_shared<std::ofstream>(physics_log_filename);
        if (!file_stream->is_open()) {
            _message_::SIMULATOR_ERROR("[SIMBRG] FAILED TO OPEN LOG FILE: ", 
                                        physics_log_filename.c_str());
            return false;
        }

        // --------------------------------------------------------------------
        // STEP 2 – Add the "Tag" attribute for VIO log filtering, only if not already present.
        // --------------------------------------------------------------------
        if (!m_logger.GetPhysicsLogger().get_attributes().count("Tag")) {
            m_logger.GetPhysicsLogger().add_attribute("Tag", 
                        attrs::constant<std::string>("PhysicsTag"));
        }

        // --------------------------------------------------------------------
        // STEP 3 – Define and configure the synchronous sink with text backend.
        // --------------------------------------------------------------------
        typedef sinks::synchronous_sink<sinks::text_ostream_backend> text_sink;
        boost::shared_ptr<text_sink> physics_sink = boost::make_shared<text_sink>();
        
        // Unlike the flightstack, we aren't looking for the highest tier of 
        // performance, therfore you can set the logger to flush data to file
        // as soon as it gets it in the cache.
        physics_sink->locked_backend()->auto_flush(true);   

        // --------------------------------------------------------------------
        // STEP 4 – Add file stream to sink backend.
        // --------------------------------------------------------------------
        physics_sink->locked_backend()->add_stream(file_stream);

        // --------------------------------------------------------------------
        // STEP 5 – Set log message format for clarity and detail.
        // --------------------------------------------------------------------
        physics_sink->set_formatter(
            expr::stream
                << "[" << expr::format_date_time<boost::posix_time::ptime>("TimeStamp", "%Y-%m-%d %H:%M:%S.%f") << "] "
                << "[" << expr::attr<boost::log::attributes::current_thread_id::value_type>("ThreadID") << "] "
                << "[" << expr::attr<std::string>("Tag") << "] "
                << "[" << expr::attr<boost::log::attributes::current_process_id::value_type>("ProcessID") << "] "
                << "[" << expr::attr<unsigned int>("LineID") << "] "
                << expr::smessage
        );

        // --------------------------------------------------------------------
        // STEP 6 – Register the sink with the Boost.Log core.
        // --------------------------------------------------------------------
        logging::core::get()->add_sink(physics_sink);

        // --------------------------------------------------------------------
        // STEP 7 – Set filter to only accept logs with Tag == "VioTag".
        // --------------------------------------------------------------------
        physics_sink->set_filter(expr::has_attr("Tag") && expr::attr<std::string>("Tag") == "PhysicsTag");

        // --------------------------------------------------------------------
        // STEP 8 – Add common Boost.Log attributes for timestamps, IDs, etc.
        // --------------------------------------------------------------------
        logging::add_common_attributes();
        _message_::SIMULATOR_INFO("[SIMBRG] SIMULATOR PHYSICS LOG FILE OPENED");

        // --------------------------------------------------------------------
        // Return true if all logging setup steps were successful.
        // --------------------------------------------------------------------
        return true;
    } catch (const std::filesystem::filesystem_error& e) {
        _message_::SIMULATOR_ERROR("[SIMBRG] FILESYSTEM ERROR: ", e.what());
    } catch (const std::exception& e) {
        _message_::SIMULATOR_ERROR("[SIMBRG] EXCEPTION: ", e.what());
    }

    // ------------------------------------------------------------------------
    // Return false if any error or exception occurred.
    // ------------------------------------------------------------------------
    return false;
}

// =====================================================================================================================
// ConfigureHeaders()
//
// Purpose:
//   Build and write a CSV header row for all chassis and propeller states to the physics log.
//   Ensures that data columns in the log file are clearly labeled and aligned with each timestep's values.
//
// Workflow:
//   1. Define a complete, ordered list of state variable suffixes to describe all measured and derived fields.
//   2. Determine the current UAV's number of propellers (dynamic platform support).
//   3. Assemble header names for both the chassis (e.g., "chassis_x") and each propeller (e.g., "propeller_1_vx").
//   4. Concatenate all headers into a single comma-separated string for log file output.
//   5. Attach the "Tag" logging attribute as required by the Boost.Log sink filter to ensure this line is logged.
//   6. Write the header string using Boost.Log. Report to terminal on success.
//   7. Catch and report any exceptions that occur during log output.
//
// Notes:
//   - Guarantees all physics log columns are properly labeled for later analysis.
//   - Structure and naming align with simulation state output structures and variable names.
//   - Ensures compatibility with log sink filters (must have "PhysicsTag" attribute present).
//   - Fails gracefully, reporting header write errors to the terminal console.
// =====================================================================================================================
void simbridge::ConfigureHeaders()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Create array of suffixes for all UAV and propeller states.
    // ------------------------------------------------------------------------
    std::vector<std::string> state_suffixes = {
        "_t", "_x", "_y", "_z",
        "_vx", "_vy", "_vz",
        "_ax", "_ay", "_az",
        "_phi", "_theta", "_psi",
        "_q0", "_q1", "_q2", "_q3", "_q4",
        "_wx", "_wy", "_wz",
        "_alphx", "_alphy", "_alphz",
        "_muIx", "_muIy", "_muIz",
        "_muJx", "_muJy", "_muJz",
        "_tauJx", "_tauJy", "_tau_Jz"
    };

    // ------------------------------------------------------------------------
    // STEP 2 – Query live platform for current number of propellers
    // ------------------------------------------------------------------------
    int num_propellers = this->uav->GetPropCount();

    // ------------------------------------------------------------------------
    // STEP 3 – Assemble header names for chassis and all propellers
    // ------------------------------------------------------------------------
    std::vector<std::string> all_headers;
    for (const auto& suffix : state_suffixes) {
        all_headers.push_back("chassis" + suffix);
    }
    for (int idx = 1; idx <= num_propellers; ++idx) {
        for (const auto& suffix : state_suffixes) {
            all_headers.push_back("propeller_" + std::to_string(idx) + suffix);
        }
    }

    // ------------------------------------------------------------------------
    // STEP 4 – Concatenate all header names with a comma separator
    // ------------------------------------------------------------------------
    std::ostringstream oss;
    oss << ", "; // Leading delimiter for tag-data separation (if needed)
    for (const auto& name : all_headers) {
        oss << name << ", ";
    }

    try {
        // ------------------------------------------------------------------------
        // STEP 5 – Attach thread-local "Tag" so the logger filter lets 
        //          this log record through
        // ------------------------------------------------------------------------
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "PhysicsTag");

        // ------------------------------------------------------------------------
        // STEP 6 – Write headers to the physics log using Boost.Log macro
        // ------------------------------------------------------------------------
        BOOST_LOG(m_logger.GetPhysicsLogger()) << oss.str();
        _message_::SIMULATOR_INFO("[SIMBRG] WROTE PHYSICS LOG HEADER DATA");
    }
    catch (const std::exception& e) {
        // ------------------------------------------------------------------------
        // STEP 7 – Report errors to terminal logger if header write fails
        // ------------------------------------------------------------------------
        _message_::SIMULATOR_ERROR("[SIMBRG] FAILED TO WRITE PHYSICS LOG HEADERS", e.what());
    }
}


void simbridge::LogData()
{
    std::ostringstream oss;

    // Grab the states
    auto uav_states = this->uav->GetUAVStateData();

    // Add the uav states to the buffer
    oss << ", "
        << uav_states.time << ", "
        << uav_states.pos.x() << ", "
        << uav_states.pos.y() << ", "
        << uav_states.pos.z() << ", "
        << uav_states.vel.x() << ", "
        << uav_states.vel.y() << ", "
        << uav_states.vel.z() << ", "
        << uav_states.acc.x() << ", "
        << uav_states.acc.y() << ", "
        << uav_states.acc.z() << ", "
        << uav_states.eul.x() << ", "
        << uav_states.eul.y() << ", "
        << uav_states.eul.z() << ", "
        << uav_states.quat.e0() << ", "
        << uav_states.quat.e1() << ", "
        << uav_states.quat.e2() << ", "
        << uav_states.quat.e3() << ", "
        << uav_states.ovel.x() << ", "
        << uav_states.ovel.y() << ", "
        << uav_states.ovel.z() << ", "
        << uav_states.oacc.x() << ", "
        << uav_states.oacc.y() << ", "
        << uav_states.oacc.z() << ", "
        << uav_states.muI.x() << ", "
        << uav_states.muI.z() << ", "
        << uav_states.muI.y() << ", "
        << uav_states.muJ.x() << ", "
        << uav_states.muJ.z() << ", "
        << uav_states.muJ.y() << ", "
        << uav_states.tauJ.x() << ", "
        << uav_states.tauJ.y() << ", "
        << uav_states.tauJ.z() << ", ";

    // Iteratively grab the states of the propellers
    int nop = this->uav->GetPropCount();

    for (int i = 1; i <= nop; ++i)
    {
        // Get the states
        auto prop_states = this->uav->GetUAVPropStateData(i);
        
        // Add it to the buffer
        oss << prop_states.time << ", "
            << prop_states.pos.x() << ", "
            << prop_states.pos.y() << ", "
            << prop_states.pos.z() << ", "
            << prop_states.vel.x() << ", "
            << prop_states.vel.y() << ", "
            << prop_states.vel.z() << ", "
            << prop_states.acc.x() << ", "
            << prop_states.acc.y() << ", "
            << prop_states.acc.z() << ", "
            << prop_states.eul.x() << ", "
            << prop_states.eul.y() << ", "
            << prop_states.eul.z() << ", "
            << prop_states.quat.e0() << ", "
            << prop_states.quat.e1() << ", "
            << prop_states.quat.e2() << ", "
            << prop_states.quat.e3() << ", "
            << prop_states.ovel.x() << ", "
            << prop_states.ovel.y() << ", "
            << prop_states.ovel.z() << ", "
            << prop_states.oacc.x() << ", "
            << prop_states.oacc.y() << ", "
            << prop_states.oacc.z() << ", "
            << prop_states.muI.x() << ", "
            << prop_states.muI.z() << ", "
            << prop_states.muI.y() << ", "
            << prop_states.muJ.x() << ", "
            << prop_states.muJ.z() << ", "
            << prop_states.muJ.y() << ", "
            << prop_states.tauJ.x() << ", "
            << prop_states.tauJ.y() << ", "
            << prop_states.tauJ.z() << ", ";

    }

    try {
        // ------------------------------------------------------------------------
        // STEP 5 – Attach thread-local "Tag" so the logger filter lets 
        //          this log record through
        // ------------------------------------------------------------------------
        BOOST_LOG_SCOPED_THREAD_TAG("Tag", "PhysicsTag");

        // ------------------------------------------------------------------------
        // STEP 6 – Write headers to the physics log using Boost.Log macro
        // ------------------------------------------------------------------------
        BOOST_LOG(m_logger.GetPhysicsLogger()) << oss.str();
    }
    catch (const std::exception& e) {
        // ------------------------------------------------------------------------
        // STEP 7 – Report errors to terminal logger if header write fails
        // ------------------------------------------------------------------------
        _message_::SIMULATOR_ERROR("[SIMBRG] FAILED TO WRITE PHYSICS LOG DATA", e.what());
    }


}


}   // namespace _bridge_

}   // namespace _acsl_