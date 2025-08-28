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


// Helper to pretty-print small values as zero
double pretty(double x, double tol = 1e-10) {
    return (std::abs(x) < tol) ? 0.0 : x;
}

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
       
        // After the loop, get the overall elapsed time (in seconds)
        double sim_time = m_bridge.GetSimSystem().GetPhysicsSystem().GetChTime();
        
        // Get the UAV's states
        auto states = m_bridge.GetUAV()->GetUAVStateData();

        auto forces = _acsl_::_transformations_::GetNEDPosFromChrono( m_bridge.GetUAV()->GetUAVChassis().body->GetAppliedForce() );
        auto torques = m_bridge.GetUAV()->GetUAVChassis().body->GetAppliedTorque() ;
        
        bool display_drone_data{false};

        if (display_drone_data)
        {
            std::cout << "Simulation time: " << states.time << " s, "  << std::endl;

            
            std::cout << "UAV position in NED frame: "
                    << pretty(states.pos.x(), 1e-10) << ", "
                    << pretty(states.pos.y(), 1e-10) << ", "
                    << pretty(states.pos.z(), 1e-10) << std::endl;
            
            std::cout << "UAV velocity in NED frame: "
                    << pretty(states.vel.x(), 1e-10) << ", "
                    << pretty(states.vel.y(), 1e-10) << ", "
                    << pretty(states.vel.z(), 1e-10) << std::endl;

            std::cout << "UAV rotation in NED frame (CHRONO): " 
                    << pretty(_acsl_::_conversions_::rad2deg(states.eul.x()), 1e-10) << ", "
                    << pretty(_acsl_::_conversions_::rad2deg(states.eul.y()), 1e-10) << ", "
                    << pretty(_acsl_::_conversions_::rad2deg(states.eul.z()), 1e-10) << std::endl;


            std::cout << "UAV angular velocity in NED frame: "
                    << pretty(states.ovel.x(), 1e-10) << ", "
                    << pretty(states.ovel.y(), 1e-10) << ", "
                    << pretty(states.ovel.z(), 1e-10) << std::endl;

            std::cout << "UAV FORCES IN NED FRAME: "
                      << pretty(forces.x(), 1e-10) << ", "
                      << pretty(forces.y(), 1e-10) << ", "
                      << pretty(forces.z(), 1e-10) << std::endl;

            std::cout << "UAV TORQUES IN NED FRAME: "
                      << pretty(torques.x(), 1e-10) << ", "
                      << pretty(torques.y(), 1e-10) << ", "
                      << pretty(torques.z(), 1e-10) << std::endl;
        }

    }

    return 0;

}


