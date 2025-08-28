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
 * File:        sim-main.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        June 13, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Main include file for the acsl physics simulator
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-main.hpp"


// // Helper to pretty-print small values as zero
// double pretty(double x, double tol = 1e-10) {
//     return (std::abs(x) < tol) ? 0.0 : x;
// }

int main(int argc, char* argv[]) {

    // Set path to Chrono data directory
    chrono::SetChronoDataPath(CHRONO_DATA_DIR);

    // Create an instance of sim-bridge
    _acsl_::_bridge_::simbridge m_bridge;   

    chrono::ChRealtimeStepTimer realtime_timer;
    double step_size = m_bridge.GetSimSystem().GetPhyConfig().StepSize;
    
    realtime_timer.start();
    
    while (m_bridge.GetSimSystem().GetVisionSystem().Run()) {
				
        m_bridge.UpdateVisualizationSystem();

		m_bridge.GetSimSystem().GetPhysicsSystem().DoStepDynamics(step_size);

        // Spin in place to maintain soft real-time
        realtime_timer.Spin(step_size);
        
        // Get the UAV's states
        auto states = m_bridge.GetUAV()->GetUAVStateData();
        
        bool display_drone_data{true};

        if (display_drone_data)
        {
            std::string color_label = "\033[38;5;214m";  // Bright Orange
            std::string color_value = "\033[1;36m";      // Bright/Bold Cyan
            std::string color_reset = "\033[0m";         // Reset

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

            // Print the colorized output
            std::cout << msg.str() << std::endl;
        }

    }

    return 0;

}


