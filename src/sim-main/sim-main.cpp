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
#include "chrono/physics/ChBodyEasy.h"

// Helper to pretty-print small values as zero
double pretty(double x, double tol = 1e-10) {
    return (std::abs(x) < tol) ? 0.0 : x;
}

int main(int argc, char* argv[]) {

    // Set path to Chrono data directory
    chrono::SetChronoDataPath(CHRONO_DATA_DIR);

    // // Create a Chrono physical system
    // ChSystemNSC sys;
    // sys.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);
    // auto solver = chrono_types::make_shared<chrono::ChSolverPSOR>();
    // solver->SetMaxIterations(1000);
    // solver->EnableWarmStart(true);
    // chrono::ChCollisionModel::SetDefaultSuggestedEnvelope(0.001);
    // chrono::ChCollisionModel::SetDefaultSuggestedMargin(0.0005);
    // chrono::ChCollisionSystemBullet::SetContactBreakingThreshold(0.001);
    // sys.SetSolver(solver);

    // Test create the sim-bridge
    _acsl_::_bridge_::simbridge m_bridge;
      
    // Create a quaternion for +90 degrees about the x-axis
    // chrono::ChQuaternion<> RotChABSFrametoNEDFrame = chrono::QuatFromAngleX(chrono::CH_PI_2);
    
    // 1. Create the NED frame (rotated +90 deg about x)
    // auto NEDFrame = chrono_types::make_shared<chrono::ChBodyAuxRef>();  // Create an auxilliary frame
    // NEDFrame->SetFixed(true);                                   // fixed, so it cannot move
    // NEDFrame->SetPos(chrono::ChVector3d(0, 0, 0));                      // Set the position of this frame to be the origin
    // NEDFrame->SetRot(RotChABSFrametoNEDFrame);                  // Apply the rotation 
    // m_sys.GetPhysicsSystem().Add(NEDFrame);                     // Add the NED frame to the system
    // chrono::ChFrame<> NEDFRame_for_render(NEDFrame->GetPos(), NEDFrame->GetRot());

    // 2. Create the floor body in the NED frame
    // Specify size in NED coordinates (e.g., 10 x 0.5 x 10 in NED x, y, z)
    double floor_length = 0.25; // NED x
    double floor_thickness = 0.25; // NED y (vertical in NED)
    double floor_width = 0.5; // NED z
    auto contact_material = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
    contact_material->SetFriction(1.0); 
    contact_material->SetDampingF(0.5);

    auto floorBody = chrono_types::make_shared<chrono::ChBodyEasyBox>(
                        floor_length, floor_thickness, floor_width, // size in NED frame
                        1000, true, true, contact_material);

    // Set the floor's frame to match the NED frame
	floorBody->SetName("spinny");
    floorBody->SetPos(_acsl_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(-2,-2,-1))); // Place at NED origin, or set as needed
    floorBody->SetRot(_acsl_::_transformations_::GetChronoOrientaitonFromNED()); // Align with NED axes
    floorBody->SetFixed(false); // Not fixed; will be fixed by constraint
    floorBody->GetVisualShape(0)->SetColor(chrono::ChColor(1.0f, 0.0f, 0.0f));
    m_bridge.GetSimSystem().GetPhysicsSystem().Add(floorBody);
    
    // Apply a torque of (Tx, Ty, Tz) in the body frame
    chrono::ChVector3d torque_local(0, 1, 0);
    
    // Apply a force of (Fx, Fy, Fz) in the body frame
    chrono::ChVector3d force_local(0, 0, 0);

	auto contact_material_ball = chrono_types::make_shared<chrono::ChContactMaterialNSC>();
	auto ballBody = chrono_types::make_shared<chrono::ChBodyEasySphere>(
	0.0254,                 // Radius
	7850,                   // Density
	true,                   // Visual asset
	true,                   // Enable collision
	contact_material_ball   // Material
	);
	ballBody->SetName("ball");
	ballBody->SetPos(_acsl_::_transformations_::GetChronoPosFromNED(chrono::ChVector3d(0,0,-0.7)));
	ballBody->SetFixed(false);
	ballBody->GetVisualShape(0)->SetTexture(chrono::GetChronoDataFile("textures/redwhite.png"));
	// m_bridge.GetSimSystem().GetPhysicsSystem().Add(ballBody);

    // Auxiliary marker (coordinate system feature)
    // Actuator 1 reference body
    auto aux_body_1 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    aux_body_1->SetName("actuator_1_auxref");
    aux_body_1->SetFrameRefToAbs(chrono::ChFrame<>(
                                 _acsl_::_transformations_::GetChronoPosFromNED(
                                  chrono::ChVector3d(0.184376362665965,0.308068237403556,0.152623963796844)), 
                                  _acsl_::_transformations_::GetChronoOrientaitonFromNED(
                                  chrono::ChQuaternion<>(1,0,0,0))));
    aux_body_1->SetFixed(false);
    // m_bridge.GetSimSystem().GetPhysicsSystem().Add(aux_body_1);

    // Actuator 2 reference body
    auto aux_body_2 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    aux_body_2->SetName("actuator_2_auxref");
    aux_body_2->SetFrameRefToAbs(chrono::ChFrame<>(
                                 _acsl_::_transformations_::GetChronoPosFromNED(
                                 chrono::ChVector3d(-0.00942816685338317,0.086637844787446,0.152623963796845)),
                                 _acsl_::_transformations_::GetChronoOrientaitonFromNED(
                                 chrono::ChQuaternion<>(1,0,0,0))));
    aux_body_2->SetFixed(false);
    // m_bridge.GetSimSystem().GetPhysicsSystem().Add(aux_body_2);

    // Actuator 3 reference body
    auto aux_body_3 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    aux_body_3->SetName("actuator_3_auxref");
    aux_body_3->SetFrameRefToAbs(chrono::ChFrame<>(
                                 _acsl_::_transformations_::GetChronoPosFromNED(
                                 chrono::ChVector3d(0.184376362665988,0.0866378447874975,0.152623963796846)), 
                                 _acsl_::_transformations_::GetChronoOrientaitonFromNED(
                                 chrono::ChQuaternion<>(1,0,0,0))));
    aux_body_3->SetFixed(false);
    // m_bridge.GetSimSystem().GetPhysicsSystem().Add(aux_body_3);

    // Actuator 4 reference body
    auto aux_body_4 = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    aux_body_4->SetName("actuator_4_auxref");
    aux_body_4->SetFrameRefToAbs(chrono::ChFrame<>(
                                 _acsl_::_transformations_::GetChronoPosFromNED(
                                 chrono::ChVector3d(-0.00942816685338455,0.308068237403483,0.152623963796843)), 
                                 _acsl_::_transformations_::GetChronoOrientaitonFromNED(
                                 chrono::ChQuaternion<>(1,0,0,0))));
    aux_body_4->SetFixed(false);
    // m_bridge.GetSimSystem().GetPhysicsSystem().Add(aux_body_4);

    // Setup the visualization system
    m_bridge.GetSimSystem().SetupVisualizationSystem();

     // 5 - Simulation loop
    chrono::ChRealtimeStepTimer realtime_timer;
    double step_size = m_bridge.GetSimSystem().GetPhyConfig().StepSize;
    bool display{false};
    bool display_drone{false};

    realtime_timer.start();
    
    while (m_bridge.GetSimSystem().GetVisionSystem().Run()) {
				
        // // Render scene
        // if(m_bridge.GetSimSystem().GetVisConfig().enable_vis) {
        //     m_bridge.GetSimSystem().GetVisionSystem().BeginScene();
        //     m_bridge.GetSimSystem().GetVisionSystem().Render();
        //     m_bridge.GetSimSystem().GetVisionSystem().RenderFrame(m_bridge.GetUAV()->GetInertialNEDFrame(), 1);
        //     m_bridge.GetSimSystem().GetVisionSystem().RenderFrame(floorBody->GetFrameRefToAbs(), 1);
        //     m_bridge.GetSimSystem().GetVisionSystem().EndScene();
        //     // m_bridge.GetSimSystem().GetVisionSystem().RenderCOGFrames();
        //     // m_sys.GetVisionSystem().EnableBodyFrameDrawing(true);

        //     // IF moving camera then,
        //     if (!m_bridge.GetSimSystem().GetVisConfig().enable_static_cam)
        //     {
        //         m_bridge.GetSimSystem().GetCamera()->Update(step_size);
        //         m_bridge.GetSimSystem().GetVisionSystem().GetActiveCamera()->setPosition(irr::core::vector3dfCH(m_bridge.GetSimSystem().GetCamera()->GetCameraPos()));
        //         m_bridge.GetSimSystem().GetVisionSystem().GetActiveCamera()->setTarget(irr::core::vector3dfCH(m_bridge.GetSimSystem().GetCamera()->GetTargetPos()));
        //     }
        // }

        m_bridge.UpdateVisualizationSystem();

        // Do the physics
		floorBody->EmptyAccumulators();
        floorBody->AccumulateForce(force_local, chrono::ChVector3d(0,0,0), true);
        // m_bridge.GetUAV()->GetUAVChassis().body->AccumulateForce(chrono::ChVector3d(0,0,-0.05), chrono::ChVector3d(0,0,0), true);
        floorBody->AccumulateTorque(torque_local, true);

		m_bridge.GetSimSystem().GetPhysicsSystem().DoStepDynamics(step_size);

        // Spin in place to maintain soft real-time
        realtime_timer.Spin(step_size);

        chrono::ChFrame<> ned_abs = m_bridge.GetUAV()->GetInertialNEDFrame();
        chrono::ChFrame<> floor_abs = floorBody->GetFrameRefToAbs();
        chrono::ChFrame<> floor_in_ned = ned_abs.GetInverse() * floor_abs;
        chrono::ChVector3d pos_in_ned = floor_in_ned.GetPos();
        chrono::ChQuaterniond rot_in_ned = floor_in_ned.GetRot();
        auto rpy_chrono = rot_in_ned.GetCardanAnglesXYZ();
        chrono::ChVector3d w_body = floorBody->GetAngVelLocal();

        chrono::ChVector3d v_abs = floorBody->GetPosDt();
        chrono::ChMatrix33d R_ned = ned_abs.GetRotMat();
        chrono::ChVector3d v_in_ned =  R_ned.transpose() * v_abs;

        // After the loop, get the overall elapsed time (in seconds)
        double sim_time = m_bridge.GetSimSystem().GetPhysicsSystem().GetChTime();
        
        // Get the UAV's states
        auto states = m_bridge.GetUAV()->GetUAVStateData();

        if (display)
        {
            std::cout << "Simulation time: " << sim_time << " s, "  << std::endl;

            
            std::cout << "Floor position in NED frame: "
                    << pretty(pos_in_ned.x(), 1e-10) << ", "
                    << pretty(pos_in_ned.y(), 1e-10) << ", "
                    << pretty(pos_in_ned.z(), 1e-10) << std::endl;
            
            std::cout << "Floor velocity in NED frame: "
                    << pretty(v_in_ned.x(), 1e-10) << ", "
                    << pretty(v_in_ned.y(), 1e-10) << ", "
                    << pretty(v_in_ned.z(), 1e-10) << std::endl;

            std::cout << "Floor rotation in NED frame (CHRONO): " 
                    << pretty(_acsl_::_conversions_::rad2deg(rpy_chrono.x()), 1e-10) << ", "
                    << pretty(_acsl_::_conversions_::rad2deg(rpy_chrono.y()), 1e-10) << ", "
                    << pretty(_acsl_::_conversions_::rad2deg(rpy_chrono.z()), 1e-10) << std::endl;


            std::cout << "Floor angular velocity in NED frame: "
                    << pretty(w_body.x(), 1e-10) << ", "
                    << pretty(w_body.y(), 1e-10) << ", "
                    << pretty(w_body.z(), 1e-10) << std::endl;
        }

        if (display_drone)
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
        }

    }

    return 0;

}


