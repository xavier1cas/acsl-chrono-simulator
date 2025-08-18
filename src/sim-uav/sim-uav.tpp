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
 * File:        sim-uav.tpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 15, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class file for the uav baseclass
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_UAV_TPP_
#define SIM_UAV_TPP_

#include "sim-uav.hpp"

namespace _acsl_
{

namespace _uav_
{

// =========================================================================================================
// SetupInertialNEDFrame()
//
// Purpose:
//   Set up the Inertial NED (North–East–Down) reference frame in the Chrono
//   global simulation space. This frame acts as a fixed reference, typically
//   for interpreting UAV states with respect to world coordinates.
//
// Workflow:
//   1. Create the reference body for the NED frame.
//   2. Make it fixed (immovable).
//   3. Position it at the world origin.
//   4. Orient it correctly so the Chrono world (ENU) frame aligns as expected
//      with the UAV's NED convention.
//   5. Add it to the simulation system.
//
// Notes:
//   - This should be called before any other UAV setup—other UAV elements may
//     require a valid inertial reference frame.
// =========================================================================================================
template <int nop>
void simuav<nop>::SetupInertialNEDFrame()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Create the Chrono body for the inertial NED frame
    //   - ChBodyAuxRef allows for a reference frame at an arbitrary pose
    // ------------------------------------------------------------------------
    InertialFrameNED = chrono_types::make_shared<chrono::ChBodyAuxRef>();
    InertialFrameNED->SetName("ned-frame");

    // ------------------------------------------------------------------------
    // STEP 2 – Mark this frame as fixed (does not move during simulation)
    // ------------------------------------------------------------------------
    InertialFrameNED->SetFixed(true);

    // ------------------------------------------------------------------------
    // STEP 3 – Place the frame at the world origin (0,0,0)
    // ------------------------------------------------------------------------
    InertialFrameNED->SetPos(chrono::ChVector3d(0, 0, 0));

    // ------------------------------------------------------------------------
    // STEP 4 – Orient the frame to define NED relative to Chrono global frame
    //   - Use conversion utility to get the appropriate quaternion orientation.
    //   - Ensures NED axes map correctly to Chrono frame (usually ENU by default)
    // ------------------------------------------------------------------------
    InertialFrameNED->SetRot(_transformations_::GetChronoOrientaitonFromNED());

    // ------------------------------------------------------------------------
    // STEP 5 – Register this reference frame with the physics system
    //   - Allows other objects (e.g., UAV, sensors) to use it as a stable anchor.
    // ------------------------------------------------------------------------
    m_physics_.Add(GetInertialNEDFrameAuxBody());
}


// =========================================================================================================
// GetInertialNEDFrame()
//
// Purpose:
//   Retrieve the Chrono (ChFrame<>) representation of the Inertial NED (North-East-Down)
//   frame, which is typically used for visualization or further transformation operations.
//
// Safety:
//   - Checks if the Inertial Frame NED body pointer is valid before accessing.
//   - If not set, triggers the simulator error message.
//
// Returns:
//   Chrono::ChFrame<> constructed using the current position and rotation of
//   the Inertial NED frame's ChBodyAuxRef object.
// =========================================================================================================
template <int nop>
chrono::ChFrame<> simuav<nop>::GetInertialNEDFrame()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Safety check: Ensure the Inertial Frame NED has been set up
    // ------------------------------------------------------------------------
    if (!InertialFrameNED) { _message_::SIMULATOR_ERROR("[SIMUAV]: INERTIAL FRAME NED IS NOT FOUND"); }

    // ------------------------------------------------------------------------
    // STEP 2 – Construct a ChFrame based on NED body's current position/rotation
    //   - Get position and orientation (quaternion) from the ChBodyAuxRef.
    //   - This frame can be used directly for visualization or reference transforms.
    // ------------------------------------------------------------------------
    chrono::ChFrame<> NEDFrame(
        InertialFrameNED->GetPos(),   // NED origin in Chrono world coordinates
        InertialFrameNED->GetRot()    // NED orientation as quaternion
    );

    // ------------------------------------------------------------------------
    // STEP 3 – Return the frame to the caller
    // ------------------------------------------------------------------------
    return NEDFrame;
}


// =========================================================================================================
// InitiateUAVChassis()
//
// Purpose:
//   Create and configure the UAV's chassis (main rigid body) and prepare it for
//   simulation by setting its physical properties and attaching its visual model.
//
// Notes:
//   - This function only sets up the chassis object in memory and stores it in
//     `bodylist`. It does NOT add it to the physics system — that is done later 
//     in InitiateUAV().
//   - All chassis configuration parameters (mass, inertia, COM, initial pose,
//     visual file name, etc.) must be set before calling this function.
// =========================================================================================================
template <int nop>
void simuav<nop>::InitiateUAVChassis()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Create the Chrono body for the chassis
    //   ChBodyAuxRef supports an auxiliary reference frame (useful for NED frames).
    // ------------------------------------------------------------------------
    chassis.body = chrono_types::make_shared<chrono::ChBodyAuxRef>();

    // ------------------------------------------------------------------------
    // STEP 2 – Apply basic identifying and initial state properties
    // ------------------------------------------------------------------------
    chassis.body->SetName("chassis");             // Set body name - Always constant
    chassis.body->SetPos(chassis.init_pos);       // Initial position (Chrono coords)
    chassis.body->SetRot(chassis.init_rot);       // Initial orientation (Chrono quat)
    chassis.body->SetMass(chassis.mass);          // Mass in kg
    chassis.body->SetInertiaXX(chassis.InertiaXX);// Principal moments of inertia
    chassis.body->SetInertiaXY(chassis.InertiaXY);// Products of inertia (off-diagonal terms)
    chassis.body->SetFrameCOMToRef(chassis.COM);  // Center of mass frame relative to reference

    // ------------------------------------------------------------------------
    // STEP 3 – Load and attach a visual mesh for the chassis
    // ------------------------------------------------------------------------
    // Load mesh geometry from Wavefront (.obj) file
    //   - First param: filepath = shapes_dir + chassis visual object file name
    //   - Second param: load normals
    //   - Third param: load UV texture coords
    auto trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(
        shapes_dir + chassis.vis_obj_name, true, true
    );

    // Create a Chrono visual shape from the mesh
    auto trimesh_shape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
    trimesh_shape->SetMesh(trimesh);
    trimesh_shape->SetMutable(false); // Make it immutable for performance

    // Add shape to body at zero offset with identity rotation
    chassis.body->AddVisualShape(
        trimesh_shape,
        chrono::ChFramed(
            chrono::ChVector3d(0, 0, 0),
            chrono::ChQuaternion<>(1, 0, 0, 0) // identity quaternion
        )
    );

    // ------------------------------------------------------------------------
    // STEP 4 – Register chassis body into the UAV's internal body list
    //   This list will later be added to the physics system in InitiateUAV()
    // ------------------------------------------------------------------------
    bodylist.push_back(chassis.body);
}


// =========================================================================================================
// InitiateUAV()
// 
// Purpose:
//   Finalize the UAV's addition to the Chrono simulation by registering all 
//   the rigid bodies and kinematic/dynamic links that make up the UAV 
//   into the physics system (`m_physics_`).
//
// Notes:
//   - This function should be called *after* the UAV components (chassis, 
//     propellers, sensors, etc.) have been fully created and added to 
//     `bodylist` and `linklist`.
//   - Once registered, Chrono will manage these objects during simulation 
//     (integration, collision detection, constraints).
// =========================================================================================================
template <int nop>
void simuav<nop>::InitiateUAV()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Add rigid bodies to the physics system
    //   - `bodylist` contains all the ChBodyAuxRef objects for this UAV.
    //   - Each is added to the ChSystemNSC instance so it participates in 
    //     physics simulation (forces, collisions, integration).
    // ------------------------------------------------------------------------
    for (auto& body : bodylist) {
        if (body) {
            _message_::SIMULATOR_INFO("[SIMUAV]: ADDING " + body->GetName() + " TO SYSTEM"); 
            m_physics_.Add(body);
        }
    }

    // ------------------------------------------------------------------------
    // STEP 2 – Add links (joints/constraints) to the physics system
    //   - `linklist` contains the joints/constraints (ChLinkBase objects) 
    //     that connect bodies (e.g., propeller hubs to chassis, fixed joints).
    //   - Adding these ensures that the kinematic/dynamic constraints are enforced.
    // ------------------------------------------------------------------------
    for (auto& link : linklist) {
        if (link) {
            m_physics_.Add(link);
        }
    }

    // UAV is now fully part of the simulation world
}


// =========================================================================================================
// GetUAVStateData()
// 
// Purpose:
//   Collect and return the UAV's state (position, velocity, acceleration,
//   orientation, angular velocity/acceleration) in the NED frame.
//
// Returns:
//   m_states struct containing the UAV's full state in NED coordinates.
// =========================================================================================================
template <int nop>
m_states simuav<nop>::GetUAVStateData()
{
    // Object to store and return the state data
    m_states m_state;

    // ------------------------------------------------------------------------
    // STEP 1 - Get frame transformations
    // ------------------------------------------------------------------------

    // NED frame relative to Chrono's absolute world frame
    chrono::ChFrame<> ned_abs = GetInertialNEDFrameAuxBody()->GetFrameRefToAbs();

    // UAV chassis frame relative to Chrono's absolute world frame
    chrono::ChFrame<> uav_abs = GetUAVChassis().body->GetFrameRefToAbs();

    // UAV chassis frame relative to the NED frame
    //   - Inverse transform of NED_abs gives Abs->NED conversion
    //   - Multiply by uav_abs to express UAV pose w.r.t NED origin
    chrono::ChFrame<> uav_ned = ned_abs.GetInverse() * uav_abs;

    // ------------------------------------------------------------------------
    // STEP 2 - Get linear kinematics in absolute frame
    // ------------------------------------------------------------------------

    // Linear velocity (absolute frame)
    chrono::ChVector3d uav_vel_abs = GetUAVChassis().body->GetPosDt();

    // Linear acceleration (absolute frame)
    chrono::ChVector3d uav_acc_abs = GetUAVChassis().body->GetPosDt2();

    // ------------------------------------------------------------------------
    // STEP 3 - Prepare NED rotation transformation
    // ------------------------------------------------------------------------

    // Rotation matrix of the NED frame relative to absolute frame
    //   - Used to convert vectors from Abs -> NED coordinates
    chrono::ChMatrix33d R_ned = ned_abs.GetRotMat();

    // ------------------------------------------------------------------------
    // STEP 4 - Fill state structure
    // ------------------------------------------------------------------------

    // --- Time Of the Simulation ---
    m_state.time = this->m_physics_.GetChTime();

    // --- Position in NED frame ---
    m_state.pos = uav_ned.GetPos();

    // --- Linear velocity in NED frame ---
    // Apply rotation transpose (i.e., inverse) to convert from Abs to NED
    m_state.vel = R_ned.transpose() * uav_vel_abs;

    // --- Linear acceleration in NED frame ---
    m_state.acc = R_ned.transpose() * uav_acc_abs;

    // --- Orientation in NED frame ---
    // Quaternion (scalar-last in Chrono convention)
    m_state.quat = uav_ned.GetRot();

    // Euler angles from quaternion (XYZ Cardan convention)
    m_state.eul = m_state.quat.GetCardanAnglesXYZ();

    // --- Angular velocity (in UAV local frame) ---
    m_state.ovel = GetUAVChassis().body->GetAngVelLocal();

    // --- Angular acceleration (in UAV local frame) ---
    m_state.oacc = GetUAVChassis().body->GetAngAccLocal();

    // Return the filled state struct
    return m_state;
}

}   // namespace _uav_

}   // namespace _acsl_

#endif  // SIM_UAV_TPP_