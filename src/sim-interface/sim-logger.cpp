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
 * File:        sim-logger.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        August 26, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Implementation file for the common logger class that is then used to instantiate the logging
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-logger.hpp"

namespace _acsl_
{

namespace _logger_
{

// =====================================================================================================================
// ConfigureLogDirectory()
//
// Purpose:
//   Creates a directory structure for flight simulation logs, organizing output files by platform and run time.
//   Copies main config files needed for reproducibility into the log directory.
//
// Workflow:
//   1. Get current local system time to uniquely timestamp the log directory for archival.
//   2. Format date ("YYYY_MM_DD") and time ("HH_MM_SS") strings for use in folder names.
//   3. Read simulator config YAML, extract platform selections, and validate exclusive platform choice.
//   4. Assemble the full log directory: base path, platform folder, date folder, and flight run folder.
//   5. Create the directory tree recursively using std::filesystem and handle errors.
//   6. Print diagnostic info for success or errors during directory creation.
//   7. Ensure a "config" subdirectory exists under the new log directory for storing reproducibility-critical files.
//   8. Copy required config files from "../config" to the log's config subdirectory; log outcome of each copy.
//
// Notes:
//   - Only "phy-config.yaml", "sim-config.yaml", and "vis-config.yaml" are copied per run.
//   - Any file or directory creation issues are logged as errors and do not halt subsequent file copies.
//   - Layout enables full reproducibility and traceability for each simulation run.
//   - Uses std::filesystem for portability and robust error handling.
// =====================================================================================================================
void simlog::ConfigureLogDirectory()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Get current local system time for this simulator run
    // ------------------------------------------------------------------------
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    struct tm tm_result;
    localtime_r(&t, &tm_result);

    // ------------------------------------------------------------------------
    // STEP 2 – Format date and time strings for unique directory naming
    // ------------------------------------------------------------------------
    std::ostringstream dateStream, timeStream;
    dateStream << std::put_time(&tm_result, "%Y_%m_%d");
    timeStream << std::put_time(&tm_result, "%H_%M_%S");
    std::string dateStr = dateStream.str();
    std::string timeStr = timeStream.str();

    // ------------------------------------------------------------------------
    // STEP 3 – Read sim config file and extract platform selection from YAML
    //   - Open sim_config_filepath for reading
    //   - Populate available_uavs from YAML dynamically without hardcoding
    //   - Ensure only one platform is selected
    // ------------------------------------------------------------------------
    this->log_dir = "../sim-log";  // base directory
    std::ifstream ifs(this->sim_config_filepath);
    if (!ifs) {
        _message_::SIMULATOR_ERROR(
            "[SIMLOG]: COULD NOT OPEN SIMULATOR CONFIG FILE: " + this->sim_config_filepath
        );
        return;
    }
    fkyaml::node config_file = fkyaml::node::deserialize(ifs);
    for (auto& [name, ref] : available_uavs.asVectorRef()) {
        ref = config_file["platform"][name].as_bool();
    }
    this->active_platform = available_uavs.validateExclusiveSelection();
    this->log_dir /= this->active_platform;
    this->log_dir /= dateStr;
    this->log_dir /= "flight_run_" + timeStr;

    // ------------------------------------------------------------------------
    // STEP 4 – Create required log directory tree recursively; handle errors
    // ------------------------------------------------------------------------
    std::error_code ec;
    std::filesystem::create_directories(this->log_dir, ec);

    // ------------------------------------------------------------------------
    // STEP 5 – Report outcome of directory creation
    // ------------------------------------------------------------------------
    if (ec) {
        _message_::SIMULATOR_ERROR("[SIMLOG]: Error creating directories: ", ec.message());
        return;
    } else {
        _message_::SIMULATOR_INFO("[SIMLOG]: Created log directory: " + this->log_dir.string());
    }

    // ------------------------------------------------------------------------
    // STEP 6 – Ensure log/config subdirectory exists for config archival
    // ------------------------------------------------------------------------
    std::filesystem::path dst_config = this->log_dir / "config";
    std::error_code ec_config;
    std::filesystem::create_directories(dst_config, ec_config);

    // ------------------------------------------------------------------------
    // STEP 7 – Copy config files ("phy-config.yaml", "sim-config.yaml", "vis-config.yaml") to log/config subdir
    //   - Each file is copied with overwrite enabled
    //   - Every outcome is logged, errors do not halt processing
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
                "[SIMLOG]: Error copying config file: " + src.string() + " -> " + dst.string(),
                copy_ec.message()
            );
        } else {
            _message_::SIMULATOR_INFO("[SIMLOG]: Copied: " + src.string() + " to " + dst.string());
        }
    }
}



}   // namespace _logger_
    
}   // namespace _acsl_


