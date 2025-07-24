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

int main(int argc, char* argv[]) {

    // Set path to Chrono data directory
    SetChronoDataPath(CHRONO_DATA_DIR);

    // // Create a Chrono physical system
    // ChSystemNSC sys;
    // sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // auto solver = std::make_shared<chrono::ChSolverPSOR>();
    // solver->SetMaxIterations(1000);
    // solver->EnableWarmStart(true);
    // chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
    // chrono::ChCollisionModel::SetDefaultSuggestedMargin(0.0005);
    // chrono::ChCollisionSystemBullet::SetContactBreakingThreshold(0.001);
    // sys.SetSolver(solver);

    // Create a chrono physical system
    _acsl_::_system_::simsystem m_sys;
    m_sys.SetupPhysicsSystem();
    m_sys.SetupVisualizationSystem();

    // Create a quaternion for +90 degrees about the x-axis
    // ChQuaternion<> RotChABSFrametoNEDFrame = QuatFromAngleX(CH_PI_2);
    
    // 1. Create the NED frame (rotated +90 deg about x)
    // auto NEDFrame = chrono_types::make_shared<ChBodyAuxRef>();  // Create an auxilliary frame
    // NEDFrame->SetFixed(true);                                   // fixed, so it cannot move
    // NEDFrame->SetPos(ChVector3d(0, 0, 0));                      // Set the position of this frame to be the origin
    // NEDFrame->SetRot(RotChABSFrametoNEDFrame);                  // Apply the rotation 
    // m_sys.GetPhysicsSystem().Add(NEDFrame);                     // Add the NED frame to the system
    // chrono::ChFrame<> NEDFRame_for_render(NEDFrame->GetPos(), NEDFrame->GetRot());

    // 2. Create the floor body in the NED frame
    // Specify size in NED coordinates (e.g., 10 x 0.5 x 10 in NED x, y, z)
    // double floor_length = 0.25; // NED x
    // double floor_thickness = 0.25; // NED y (vertical in NED)
    // double floor_width = 0.5; // NED z

    // auto floorBody = std::make_shared<ChBodyEasyBox>(
    //     floor_length, floor_thickness, floor_width, // size in NED frame
    //     3000, true, true
    // );

    // Set the floor's frame to match the NED frame
    // floorBody->SetPos(setPosNEDinGlobal(ChVector3d(0,0,-1))); // Place at NED origin, or set as needed
    // floorBody->SetRot(setRotNEDinGlobal()); // Align with NED axes
    // floorBody->SetFixed(false); // Not fixed; will be fixed by constraint
    // floorBody->GetVisualShape(0)->SetColor(ChColor(1.0f, 0.0f, 0.0f));
    // m_sys.GetSystem().Add(floorBody);
    

    // Apply a torque of (Tx, Ty, Tz) in the body frame
    // ChVector3d torque_local(0, 1, 0);
    

    // Apply a force of (Fx, Fy, Fz) in the body frame
    // ChVector3d force_local(0, 0, 0);
    


     // 5 - Simulation loop
    ChRealtimeStepTimer realtime_timer;
    double step_size = 5e-3;
    // bool display{false};

    // 6 - EXPT add in the solidworks model
    // Optional: if you have motion functions (motors), you can track them
    // std::unordered_map<std::string, std::shared_ptr<chrono::ChFunction>> motfun_map;


    realtime_timer.start();

    while (m_sys.GetVisionSystem().Run()) {
        // Render scene
        if(m_sys.GetVisConfig().enable_vis) {
            m_sys.GetVisionSystem().BeginScene();
            m_sys.GetVisionSystem().Render();
            // m_sys.GetVisionSystem()->RenderFrame(NEDFrame->GetFrameRefToAbs(), 1);
            // m_sys.GetVisionSystem()->RenderFrame(floorBody->GetFrameRefToAbs(), 1);
            m_sys.GetVisionSystem().EndScene();
            m_sys.GetVisionSystem().RenderCOGFrames();
            m_sys.GetVisionSystem().EnableBodyFrameDrawing(true);
        }

         // Perform the integration stpe
        m_sys.GetPhysicsSystem().DoStepDynamics(step_size);

        // Spin in place to maintain soft real-time
        realtime_timer.Spin(step_size);

        // ChFrame<> ned_abs = NEDFrame->GetFrameRefToAbs();
        // ChFrame<> floor_abs = floorBody->GetFrameRefToAbs();
        // ChFrame<> floor_in_ned = ned_abs.GetInverse() * floor_abs;
        // ChVector3d pos_in_ned = floor_in_ned.GetPos();
        // ChQuaterniond rot_in_ned = floor_in_ned.GetRot();
        // auto rpy = quaternionToEulerAnglesRPY(rot_in_ned);
        // ChVector3d w_body = floorBody->GetAngVelLocal();

        // ChVector3d v_abs = floorBody->GetPosDt();
        // ChMatrix33d R_ned = ned_abs.GetRotMat();
        // ChVector3d v_in_ned =  R_ned.transpose() * v_abs;

        // After the loop, get the overall elapsed time (in seconds)
        // double sim_time = m_sys.GetSystem().GetChTime();
        // floorBody->EmptyAccumulators();
        // floorBody->AccumulateForce(force_local, ChVector3d(0,0,0), true);
        // floorBody->AccumulateTorque(torque_local, true);

        // if (display)
        // {
        //     std::cout << "Simulation time: " << sim_time << " s, "  << std::endl;

            
        //     std::cout << "Floor position in NED frame: "
        //             << pretty(pos_in_ned.x(), 1e-10) << ", "
        //             << pretty(pos_in_ned.y(), 1e-10) << ", "
        //             << pretty(pos_in_ned.z(), 1e-10) << std::endl;
            
        //     std::cout << "Floor velocity in NED frame: "
        //             << pretty(v_in_ned.x(), 1e-10) << ", "
        //             << pretty(v_in_ned.y(), 1e-10) << ", "
        //             << pretty(v_in_ned.z(), 1e-10) << std::endl;

        //     std::cout << "Floor rotation in NED frame: " 
        //             << pretty(rad2deg(rpy[0]), 1e-10) << ", "
        //             << pretty(rad2deg(rpy[1]), 1e-10) << ", "
        //             << pretty(rad2deg(rpy[2]), 1e-10) << std::endl;

        //     std::cout << "Floor angular velocity in NED frame: "
        //             << pretty(w_body.x(), 1e-10) << ", "
        //             << pretty(w_body.y(), 1e-10) << ", "
        //             << pretty(w_body.z(), 1e-10) << std::endl;
        // }
    }

    return 0;

}


