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
//     in AddUAVToSystem().
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
    // STEP 4 - Add all the collision shapes to the chassis (IF present)
    // ------------------------------------------------------------------------
    if (!this->chassis.collision.empty()) {
        // Create the collision model
        chassis.body->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

        // Iterate through the shapes and frames in the collision tuple and add it to the model
        for (const auto& [shape, frame] : this->chassis.collision) {
            chassis.body->GetCollisionModel()->AddShape(shape, frame);
        }

        // Set chassis body as collidable
        chassis.body->EnableCollision(true);
    }


    // ------------------------------------------------------------------------
    // STEP 5 – Register chassis body into the UAV's internal body list
    //   This list will later be added to the physics system in AddUAVToSystem()
    // ------------------------------------------------------------------------
    bodylist.push_back(chassis.body);
}


// =========================================================================================================
// CheckUAVPropRequest(idx)
//
// Purpose: 
//   Checks if the requested propeller is within range of the total number of propellers.
//   Total number of propellers = 1 - NOP
// Parameters:
//   idx - 1-based index of the propeller hub to access (first propeller is idx=1).
//         Must satisfy 1 <= idx <= nop.
// =========================================================================================================
template <int nop>
void simuav<nop>::CheckUAVPropRequest(size_t idx)
{
    // 1-based -> 0-based adjustment and bounds check
    if (idx == 0 || idx > nop) {
        std::ostringstream oss;
        oss << "[SIMUAV]: PROPELLER INDEX OUT OF RANGE. VALID RANGE: 1 TO " << nop << ", REQUESTED: " << idx;
        _message_::SIMULATOR_ERROR(oss.str());
    }
}


// =========================================================================================================
// GetUAVProp(idx)
// 
// Purpose:
//   Provides direct access to the physical and visualization properties of an individual
//   propeller hub of the UAV by returning a reference to its propstruct.
//   This enables external code to read or modify any propeller's data, supporting flexible
//   setup and configuration of UAV components.
//
// Parameters:
//   idx - 1-based index of the propeller hub to access (first propeller is idx=1).
//         Must satisfy 1 <= idx <= nop.
//
// Returns:
//   Reference to the propstruct instance corresponding to the requested propeller hub.
//
// Notes:
//   - The underlying std::array still uses zero-based indexing; input idx is decremented internally.
//   - Throws std::out_of_range if idx is not a valid 1-based propeller index.
//   - Enables modular, type-safe manipulation of individual UAV propellers for advanced
//     configuration, tuning, or real-time updates.
// =========================================================================================================
template <int nop>
propstruct& simuav<nop>::GetUAVProp(size_t idx)
{
    this->CheckUAVPropRequest(idx);         // Check the request
    return props[idx - 1];                  // Grant if within range
}


// =========================================================================================================
// ConfigureUAVPropInitPos(idx, pos)
//
// Purpose:
//   Set the initial position of the specified propeller for simulation.
// 
// Parameters:
//   idx - 1-based index of the propeller hub (1 <= idx <= nop).
//   pos - Initial position of the propeller (Chrono world coordinates).
// 
// Notes:
//   - This must be called before InitiateUAVProps().
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropInitPos(size_t idx, chrono::ChVector3d pos)
{
    this->CheckUAVPropRequest(idx);                                         // Check the id of the propeller
    props[idx - 1].init_pos = _transformations_::GetChronoPosFromNED(pos);         // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropInitRot(idx, rot)
//
// Purpose:
//   Set the initial orientation (quaternion) of the specified propeller.
// 
// Parameters:
//   idx - 1-based index of the propeller hub (1 <= idx <= nop).
//   rot - Initial orientation as Chrono quaternion.
// 
// Notes:
//   - This must be called before InitiateUAVProps().
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropInitRot(size_t idx, chrono::ChQuaternion<> rot)
{
    this->CheckUAVPropRequest(idx);                                         // Check the id of the propeller
    props[idx - 1].init_rot = _transformations_::GetChronoOrientaitonFromNED(rot); // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropMass(mass)
//
// Purpose:
//   Set the mass for all propellers in the UAV.
// 
// Parameters:
//   mass - Mass value to assign to each propeller (kg).
// 
// Notes:
//   - Sets mass uniformly for all propellers prior to simulation.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropMass(size_t idx, double mass)
{
    this->CheckUAVPropRequest(idx);                                         // Check the id of the propeller
    props[idx - 1].mass = mass;             // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropInertiaXX(idx, IXX)
//
// Purpose:
//   Set the principal moments of inertia (diagonal) for a specified propeller.
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   IXX - ChVector3d of inertia values.
// 
// Notes:
//   - Must be set before propeller is registered for simulation.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropInertiaXX(size_t idx, chrono::ChVector3d IXX)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx -1].InertiaXX = IXX;          // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropInertiaXY(idx, IXY)
//
// Purpose:
//   Set the products of inertia (off-diagonal) for a specified propeller.
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   IXY - ChVector3d of off-diagonal inertia values.
// 
// Notes:
//   - Must be set before physics registration.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropInertiaXY(size_t idx, chrono::ChVector3d IXY)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx -1].InertiaXY = IXY;          // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropCOM(idx, COM)
//
// Purpose:
//   Set the center of mass (frame) for a specified propeller.
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   COM - Chrono frame representing center of mass orientation and position.
// 
// Notes:
//   - Required before physics object creation.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropCOM(size_t idx, chrono::ChFrame<> COM)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx -1].COM = COM;                // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropOBJName(idx, name)
//
// Purpose:
//   Set the visual mesh filename for a specified propeller.
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   name - Wavefront .obj filename for propeller model.
// 
// Notes:
//   - Needed for visual mesh assignment during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropOBJName(size_t idx, std::string name)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx -1].vis_obj_name = name;      // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropCollisionShapes(idx, list)
//
// Purpose:
//   Assign one or more collision shapes to a specified propeller body.
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   list - Vector of CollisionShapeFrame objects describing the shapes.
// 
// Notes:
//   - Shapes should be set prior to physics engine registration.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropCollisionShapes(size_t idx, const std::vector<_acsl_::_uav_::CollisionShapeFrame>& list)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx - 1].collision = list;
}


// =========================================================================================================
// ConfigureUAVPropColor(idx, color)
//
// Purpose:
//   Set the prop color based on the predefined values in _acsl_::_uav_::_prop_color_
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   color - Color value for the prop
// 
// Notes:
//   - Needed for visual mesh assignment during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropColor(size_t idx, chrono::ChColor color)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx -1].color = color;            // Assign if within range
}


// =========================================================================================================
// ConfigureUAVPropOpacity(idx, val)
//
// Purpose:
//   Set the prop opacity based on the float value input.
// 
// Parameters:
//   idx - 1-based index (1 <= idx <= nop).
//   val - Opacity value from 0 - 1. [ 0 - fully transparent | 1 - fully opaque ]
// 
// Notes:
//   - Needed for visual mesh assignment during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVPropOpacity(size_t idx, float val)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    props[idx -1].opacity = val;            // Assign if within range
}


// =========================================================================================================
// InitiateUAVProps()
//
// Purpose:
//   Create and configure all UAV propeller bodies, preparing them for simulation.
// 
// Notes:
//   - This function registers all propellers in memory and stores them internally.
//   - Physical properties, geometry, and COM must be set before calling.
// =========================================================================================================
template <int nop>
void simuav<nop>::InitiateUAVProps()
{
    // Iterate through the total number of propellers
    for (int idx =0; idx < nop; ++idx)
    {
        // ------------------------------------------------------------------------
        // STEP 1 – Create the Chrono body for the propeller
        // ------------------------------------------------------------------------
        props[idx].body = chrono_types::make_shared<chrono::ChBodyAuxRef>();

        // ------------------------------------------------------------------------
        // STEP 2 – Apply identifying and initial state properties
        // ------------------------------------------------------------------------
        props[idx].body->SetName("propeller_" + std::to_string(idx +1));  // 1-based name
        props[idx].body->SetPos(props[idx].init_pos);
        props[idx].body->SetRot(props[idx].init_rot);
        props[idx].body->SetMass(props[idx].mass);
        props[idx].body->SetInertiaXX(props[idx].InertiaXX);
        props[idx].body->SetInertiaXY(props[idx].InertiaXY);
        props[idx].body->SetFrameCOMToRef(props[idx].COM);

        // ------------------------------------------------------------------------
        // STEP 3 – Load and attach a visual mesh for the propeller
        // ------------------------------------------------------------------------
        // Load mesh geometry from Wavefront (.obj) file
        //   - First param: filepath = shapes_dir + prop[idx] visual object file name
        //   - Second param: load normals
        //   - Third param: load UV texture coords
        auto trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(
            shapes_dir + props[idx].vis_obj_name, true, true
        );

        // Create a Chrono visual shape from the mesh
        auto trimesh_shape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
        trimesh_shape->SetMesh(trimesh);
        trimesh_shape->SetMutable(false); // Make it immutable for performance
        trimesh_shape->SetColor(props[idx].color);
        trimesh_shape->SetOpacity(props[idx].opacity);

        // Add shape to body at zero offset with identity rotation
        props[idx].body->AddVisualShape(
            trimesh_shape,
            chrono::ChFramed(
                chrono::ChVector3d(0, 0, 0),
                chrono::ChQuaternion<>(1, 0, 0, 0)  // identity quaternion
            )
        );

        // ------------------------------------------------------------------------
        // STEP 4 - Add all the collision shapes to the propeller (IF present)
        // ------------------------------------------------------------------------
        if (!this->props[idx].collision.empty()) {
            // Create the collision model
            props[idx].body->AddCollisionModel(chrono_types::make_shared<chrono::ChCollisionModel>());

            // Iterate through the shapes and frames in the collision tuple and add it to the model
            for (const auto& [shape, frame] : this->props[idx].collision) {
                props[idx].body->GetCollisionModel()->AddShape(shape, frame);
            }

            // Set propeller body as collidable
            props[idx].body->EnableCollision(true);
        }

        // ------------------------------------------------------------------------
        // STEP 5 – Register propeller body into the UAV's internal body list
        //   This list will later be added to the physics system in AddUAVToSystem()
        // ------------------------------------------------------------------------
        bodylist.push_back(props[idx].body);
    }
}


// =========================================================================================================
// LinkUAVBodies(link_data_vec)
//
// Purpose:
//   Constructs all Chrono link constraints between UAV bodies (e.g., chassis, propellers),
//   using the input vector of link property variants. Each link is created and stored
//   in the class's internal `linklist` for registration with the Chrono system later.
//
// Parameters:
//   link_data_vec - Vector of LinkData (variant property structs) specifying all
//                   constraint types and parameters between UAV bodies.
//
// Notes:
//   - Should be called after all bodies are constructed and stored in bodylist, 
//     but before AddUAVToSystem().
//   - The function does NOT add links to the Chrono system yet; it only prepares them.
//   - The input data is stored in the class member `links` for persistent access and future use.
// =========================================================================================================
template <int nop>
void simuav<nop>::LinkUAVBodies(const std::vector<LinkData>& link_data_vec)
{
    // Store a copy of the input link data in the class member for future/reference operations
    links = link_data_vec;

    // Iterate over all stored link property variants in the member vector
    for (const auto& link_data : links) {
        std::visit([&](const auto& prop) {
            std::shared_ptr<chrono::ChLinkBase> link;
            // Parallel
            if constexpr (std::is_same_v<std::decay_t<decltype(prop)>, LinkProperty<LinkType::Parallel>>) {
                link = chrono_types::make_shared<chrono::ChLinkMateParallel>();
                std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetFlipped(prop.flipped);
                std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->Initialize(prop.bodyA, prop.bodyB, false, prop.cA, prop.cB, prop.dA, prop.dB);
                std::dynamic_pointer_cast<chrono::ChLinkMateParallel>(link)->SetName(prop.name);
            }
            // Generic
            else if constexpr (std::is_same_v<std::decay_t<decltype(prop)>, LinkProperty<LinkType::Generic>>) {
                link = chrono_types::make_shared<chrono::ChLinkMateGeneric>();
                std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetConstrainedCoords(
                    prop.cx, prop.cy, prop.cz, prop.rx, prop.ry, prop.rz);
                std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->Initialize(
                    prop.bodyA, prop.bodyB, false, prop.cA, prop.cB, prop.dA, prop.dB);
                std::dynamic_pointer_cast<chrono::ChLinkMateGeneric>(link)->SetName(prop.name);
            }
            // DistanceZ
            else if constexpr (std::is_same_v<std::decay_t<decltype(prop)>, LinkProperty<LinkType::DistanceZ>>) {
                link = chrono_types::make_shared<chrono::ChLinkMateDistanceZ>();
                std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->Initialize(
                    prop.bodyA, prop.bodyB, false, prop.cA, prop.cB, prop.dB);
                std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetDistance(prop.distance);
                std::dynamic_pointer_cast<chrono::ChLinkMateDistanceZ>(link)->SetName(prop.name);
            }
            linklist.push_back(link);
        }, link_data);
    }
}

// =========================================================================================================
// GetUAVMotor(idx)
// 
// Purpose:
//   Provides direct access to the of an individual motor associated with it's respective
//   propeller hub of the UAV by returning a reference to its motorstruct.
//   This enables external code to read or modify any motors's data, supporting flexible
//   setup and configuration of UAV components.
//
// Parameters:
//   idx - 1-based index of the propeller hub to access (first motor is idx=1).
//         Must satisfy 1 <= idx <= nop.
//
// Returns:
//   Reference to the motorstruct instance corresponding to the requested propeller hub.
//
// Notes:
//   - The underlying std::array still uses zero-based indexing; input idx is decremented internally.
//   - Throws std::out_of_range if idx is not a valid 1-based propeller index.
//   - Enables modular, type-safe manipulation of individual UAV propellers for advanced
//     configuration, tuning, or real-time updates.
// =========================================================================================================
template <int nop>
motorstruct& simuav<nop>::GetUAVMotor(size_t idx)
{
    this->CheckUAVPropRequest(idx);         // Check the request
    return motors[idx - 1];                 // Grant if within range
}


// =========================================================================================================
// ConfigureUAVMotorSpinDir(idx, spin)
//
// Purpose:
//   Set the motor spin based on the int value input.
// 
// Parameters:
//   idx  - 1-based index (1 <= idx <= nop).
//   spin - Spin direction value -1 or 1. [ -1 - CW | 1 - CCW ]
// 
// Notes:
//   - Needed motor creation during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVMotorSpinDir(size_t idx, int spin)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    motors[idx -1].spin_dir = spin;         // Assign if within range
}


// =========================================================================================================
// ConfigureUAVMotorFrame(idx, frame)
//
// Purpose:
//   Set the motor frame based on the frame value input.
// 
// Parameters:
//   idx  - 1-based index (1 <= idx <= nop).
//   frame - Frame where the motor is located.
// 
// Notes:
//   - Needed motor creation during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVMotorFrame(size_t idx, chrono::ChFrame<> frame)
{
    this->CheckUAVPropRequest(idx);         // Check the id of the propeller
    motors[idx -1].frame = frame;           // Assign if within range
}


// =========================================================================================================
// ConfigureUAVMotorNorm2Newt(idx, vec)
//
// Purpose:
//   Set the motor polynomial vector for converting normalized thrust values [0-1] to Newtons.
//
// Parameters:
//   idx  - 1-based index (1 <= idx <= nop).
//   poly - Eigen::VectorXd containing polynomial coefficients.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVMotorNorm2Newt(size_t idx, Eigen::VectorXd& poly)
{
    this->CheckUAVPropRequest(idx);
    motors[idx - 1].norm2newt = poly;
}


// =========================================================================================================
// ConfigureUAVMotorNewt2Norm(idx, vec)
//
// Purpose:
//   Set the motor polynomial vector for converting Newtons to normalized thrust values [0-1].
//
// Parameters:
//   idx  - 1-based index (1 <= idx <= nop).
//   poly - Eigen::VectorXd containing polynomial coefficients.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVMotorNewt2Norm(size_t idx, Eigen::VectorXd& poly)
{
    this->CheckUAVPropRequest(idx);
    motors[idx - 1].newt2norm = poly;
}


// =========================================================================================================
// ConfigureUAVMotorNorm2RPS(idx, vec)
//
// Purpose:
//   Set the motor polynomial vector for converting normalized thrust values [0-1] to rotor speed in rad/s.
//
// Parameters:
//   idx  - 1-based index (1 <= idx <= nop).
//   poly - Eigen::VectorXd containing polynomial coefficients.
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVMotorNorm2RPS(size_t idx, Eigen::VectorXd& poly)
{
    this->CheckUAVPropRequest(idx);
    motors[idx - 1].norm2rps = poly;
}


// =========================================================================================================
// ConfigureUAVMotorCt(idx, ct)
//
// Purpose:
//   Set the motor torque constant that defines a linear realtionship between thrust and backtorque.
//
// Parameters:
//   idx  - 1-based index (1 <= idx <= nop).
//   ct - double value for the ct
// =========================================================================================================
template <int nop>
void simuav<nop>::ConfigureUAVMotorCt(size_t idx, double ct)
{
    this->CheckUAVPropRequest(idx);
    motors[idx - 1].ct = ct;
}


// =========================================================================================================
// InitiateUAVMotors()
//
// Purpose:
//   Create and configure all UAV motor bodies, preparing them for simulation.
// 
// Notes:
//   - This function registers all motors in memory and stores them internally.
//   - Spin direction and frame must be configured before the initiation
// =========================================================================================================
template <int nop>
void simuav<nop>::InitiateUAVMotors()
{
    // Iterate through the total number of propellers
    for (int idx = 0; idx < nop; ++idx)
    {
        // ------------------------------------------------------------------------
        // STEP 1 – Create the Chrono body for the motor
        // ------------------------------------------------------------------------
        motors[idx].motor = chrono_types::make_shared<chrono::ChLinkMotorRotationSpeed>();
        motors[idx].motor->SetName("motor_" + std::to_string(idx + 1));  // 1-based name

        // ------------------------------------------------------------------------
        // STEP 2 – Prepare transformed position and orientation for the motor frame
        // ------------------------------------------------------------------------
        // Convert NED frame to Chrono frame for both position and orientation
        chrono::ChVector3<> motor_pos = _transformations_::GetChronoPosFromNED(motors[idx].frame.GetPos());
        chrono::ChQuaternion<> base_rot = _transformations_::GetChronoOrientaitonFromNED(motors[idx].frame.GetRot());

        // 180-degree rotation about the X axis for CCW case
        chrono::ChQuaternion<> rot_180_x;
        rot_180_x = chrono::QuatFromAngleX(chrono::CH_PI);

        chrono::ChQuaternion<> motor_rot;
        if (motors[idx].spin_dir == _motor_dir_::CCW)
        {
            motor_rot = rot_180_x * base_rot;   // Flip for CCW
        }    
        else
        {
            motor_rot = base_rot;               // Use the base orientation for CW
        }

        motors[idx].frame.SetPos(motor_pos);
        motors[idx].frame.SetRot(motor_rot);

        // ------------------------------------------------------------------------
        // STEP 3 – Initialize the Chrono motor link
        // ------------------------------------------------------------------------
        motors[idx].motor->Initialize(
            this->GetUAVProp(idx + 1).body,    // 1-based indexing for the motors 
            this->GetUAVChassis().body,
            motors[idx].frame);
        
        // Set the spindle constraint to revolute
        motors[idx].motor->SetSpindleConstraint(chrono::ChLinkMotorRotation::SpindleConstraint::REVOLUTE);

        // ------------------------------------------------------------------------
        // STEP 4 – Set up the speed function and initial speed
        // ------------------------------------------------------------------------
        motors[idx].speed = chrono_types::make_shared<chrono::ChFunctionConst>();
        motors[idx].speed->SetConstant(0.0);   // Set the initial speed to 0 rpm
        motors[idx].motor->SetSpeedFunction(motors[idx].speed);

        // ------------------------------------------------------------------------
        // STEP 5 – Set up the thrust function and the initial thrust
        // ------------------------------------------------------------------------
        motors[idx].thrust = chrono_types::make_shared<chrono::ChForce>();      // Initiate the thrust force
        motors[idx].thrust->SetName("thrust_" + std::to_string(idx + 1));       // 1-based name
        motors[idx].thrust->SetBody(this->GetUAVChassis().body.get());          // Thrust is applied to the chassis
        motors[idx].thrust->SetMode(chrono::ChForce::ForceType::FORCE);         // It is of the type force
        motors[idx].thrust->SetFrame(chrono::ChForce::ReferenceFrame::BODY);    // It is applied in the body frame
        motors[idx].thrust->SetRelDir(chrono::ChVector3d(0,0,-1));              // It is applied in the -ve z (NED)
        
        // Iteratively compute where this thrust is applied wrt to the chassis and rotor hub.
        auto rel_coord = _transformations_::GetNEDPosFromChrono( this->GetUAVProp(idx + 1).body->GetPos() 
                                                                 - this->GetUAVChassis().body->GetPos() );
        // Point out the location in the chassis body frame (NED)
        motors[idx].thrust->SetVrelpoint(rel_coord);
        motors[idx].thrust->SetMforce(0);                                       // Apply an initial thrust
        GetUAVChassis().body->AddForce(motors[idx].thrust);                     // Add this thrust force to the body

        // ------------------------------------------------------------------------
        // STEP 5 – Set up the torque function and the initial thrust
        // ------------------------------------------------------------------------
        motors[idx].torque = chrono_types::make_shared<chrono::ChForce>();      // Initiate the torque force
        motors[idx].torque->SetName("torque_" + std::to_string(idx + 1));       // 1-basead name
        motors[idx].torque->SetBody(this->GetUAVChassis().body.get());          // Thrust is applied to the chassis
        motors[idx].torque->SetMode(chrono::ChForce::ForceType::TORQUE);        // It is of the type torque
        motors[idx].torque->SetFrame(chrono::ChForce::ReferenceFrame::BODY);    // It is applied in the body frame
        // Compute the axis around which the backtorque is applied.
        // CCW : -Z NED | CW : Z NED
        if (motors[idx].spin_dir == _motor_dir_::CW)
        {
            motors[idx].torque->SetRelDir(chrono::ChVector3d(0,0,-1));
        }    
        else
        {
            motors[idx].torque->SetRelDir(chrono::ChVector3d(0,0,1));
        }

        // Point out the location in the chassis body frame (NED)
        motors[idx].torque->SetVrelpoint(rel_coord);
        motors[idx].torque->SetMforce(0);                                       // Apply an initial thrust
        GetUAVChassis().body->AddForce(motors[idx].torque);                     // Add this torque force to the body

        _message_::SIMULATOR_INFO("[SIMUAV]: ADDING MOTOR AT: ", rel_coord );
        // ------------------------------------------------------------------------
        // STEP 7 – Register the motor link in the UAV's internal link list
        //   This list will later be added to the physics system in AddUAVToSystem()
        // ------------------------------------------------------------------------
        linklist.push_back(motors[idx].motor);
    }
}


// =========================================================================================================
// AddUAVToSystem()
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
void simuav<nop>::AddUAVToSystem()
{
    // ------------------------------------------------------------------------
    // STEP 1 – Add rigid bodies to the physics system
    //   - `bodylist` contains all the ChBodyAuxRef objects for this UAV.
    //   - Each is added to the ChSystemNSC instance so it participates in 
    //     physics simulation (forces, collisions, integration).
    // ------------------------------------------------------------------------
    for (auto& body : bodylist) {
        if (body) {
            _message_::SIMULATOR_INFO("[SIMUAV]: ADDING " + body->GetName() + " BODY TO SYSTEM"); 
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
            _message_::SIMULATOR_INFO("[SIMUAV]: ADDING " + link->GetName() + " LINK TO SYSTEM"); 
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
    //   - Used to convert vectors from Abs -> NED coordinates and vice-versa.
    chrono::ChMatrix33d R_ned = ned_abs.GetRotMat();

    // Rotation matrix of the drone relative to the NED frame
    //   - Used to convert vectors from I -> J coordinates and vice-versa.
    chrono::ChMatrix33d R_uav = uav_ned.GetRotMat();

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

    // --- Forces acting on the body in the Inertial frame (NED) ---
    m_state.muI = _transformations_::GetNEDPosFromChrono(GetUAVChassis().body->GetAppliedForce());

    // --- Forces acting on the body in the Body frame (NED) ---
    m_state.muJ =  R_uav.transpose() * m_state.muI;

    // --- Torque acting on the body in the Body frame (NED) ---
    m_state.tauJ = GetUAVChassis().body->GetAppliedTorque();

    // Return the filled state struct
    return m_state;
}

// =========================================================================================================
// GetUAVPropStateData()
// 
// Purpose:
//   Collect and return the UAV propeller's state (position, velocity, acceleration,
//   orientation, angular velocity/acceleration) in the NED frame.
//
// Returns:
//   m_states struct containing the UAV propeller's full state in NED coordinates.
// =========================================================================================================
template <int nop>
m_states simuav<nop>::GetUAVPropStateData(size_t idx)
{
    // Object to store and return the state data
    m_states m_state;

    // ------------------------------------------------------------------------
    // STEP 1 - Get frame transformations
    // ------------------------------------------------------------------------

    // NED frame relative to Chrono's absolute world frame
    chrono::ChFrame<> ned_abs = GetInertialNEDFrameAuxBody()->GetFrameRefToAbs();

    // UAV chassis frame relative to Chrono's absolute world frame
    chrono::ChFrame<> prop_abs = GetUAVProp(idx).body->GetFrameRefToAbs();

    // UAV chassis frame relative to the NED frame
    //   - Inverse transform of NED_abs gives Abs->NED conversion
    //   - Multiply by uav_abs to express UAV pose w.r.t NED origin
    chrono::ChFrame<> prop_ned = ned_abs.GetInverse() * prop_abs;

    // ------------------------------------------------------------------------
    // STEP 2 - Get linear kinematics in absolute frame
    // ------------------------------------------------------------------------

    // Linear velocity (absolute frame)
    chrono::ChVector3d prop_vel_abs = GetUAVProp(idx).body->GetPosDt();

    // Linear acceleration (absolute frame)
    chrono::ChVector3d prop_acc_abs = GetUAVProp(idx).body->GetPosDt2();

    // ------------------------------------------------------------------------
    // STEP 3 - Prepare NED rotation transformation
    // ------------------------------------------------------------------------

    // Rotation matrix of the NED frame relative to absolute frame
    //   - Used to convert vectors from Abs -> NED coordinates
    chrono::ChMatrix33d R_ned = ned_abs.GetRotMat();

    // Rotation matrix of the prop relative to the NED frame
    //   - Used to convert vectors from I -> J coordinates and vice-versa.
    chrono::ChMatrix33d R_prop = prop_ned.GetRotMat();

    // ------------------------------------------------------------------------
    // STEP 4 - Fill state structure
    // ------------------------------------------------------------------------

    // --- Time Of the Simulation ---
    m_state.time = this->m_physics_.GetChTime();

    // --- Position in NED frame ---
    m_state.pos = prop_ned.GetPos();

    // --- Linear velocity in NED frame ---
    // Apply rotation transpose (i.e., inverse) to convert from Abs to NED
    m_state.vel = R_ned.transpose() * prop_vel_abs;

    // --- Linear acceleration in NED frame ---
    m_state.acc = R_ned.transpose() * prop_acc_abs;

    // --- Orientation in NED frame ---
    // Quaternion (scalar-last in Chrono convention)
    m_state.quat = prop_ned.GetRot();

    // Euler angles from quaternion (XYZ Cardan convention)
    m_state.eul = m_state.quat.GetCardanAnglesXYZ();

    // --- Angular velocity (in UAV local frame) ---
    m_state.ovel = GetUAVProp(idx).body->GetAngVelLocal();

    // --- Angular acceleration (in UAV local frame) ---
    m_state.oacc = GetUAVProp(idx).body->GetAngAccLocal();

    // --- Forces acting on the body in the Inertial frame (NED) ---
    m_state.muI = _transformations_::GetNEDPosFromChrono(GetUAVProp(idx).body->GetAppliedForce());

    // --- Forces acting on the body in the Body frame (NED) ---
    m_state.muJ =  R_prop.transpose() * m_state.muI;

    // --- Torque acting on the body in the Body frame (NED) ---
    m_state.tauJ = GetUAVProp(idx).body->GetAppliedTorque();

    // Return the filled state struct
    return m_state;
}

// =========================================================================================================
// SetActuator(idx, thrust, torque, rpm)
// 
// Purpose:
//   Sets the motor thrust, backtorque and the rpm setpoints.
//
// Parameters:
//   idx    - 1-based index (1 <= idx <= nop).
//   thrust - thrust value in (N)
//   torque - troque value in (Nm)
//   rpm    - angular velocity setpoint of the rotors in (rad/s)
//   
// 
// Notes:
//   - Needed motor creation during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::SetActuator(size_t idx, double thrust, double torque, double rpm)
{
    // Set the thrust setpoint
    this->GetUAVMotor(idx).thrust->SetMforce(thrust);

    // Set the torque setpoint
    this->GetUAVMotor(idx).torque->SetMforce(torque);

    // Set the motor rpm
    this->GetUAVMotor(idx).speed->SetConstant(rpm);
}


// =========================================================================================================
// SetThrustSetPoint(idx, thrustSP)
// 
// Purpose:
//   Sets the motor thrust, backtorque and the rpm setpoints based on the normalized thrust setpoint input
//
// Parameters:
//   idx      - 1-based index (1 <= idx <= nop).
//   thrustSP - thrust value in normalized terms [0-1] (-)
// 
// Notes:
//   - Needed motor creation during simulation setup.
//   - Needed motor polynomial coefficient vectors to be configured during simulation setup.
// =========================================================================================================
template <int nop>
void simuav<nop>::SetThrustSetPoint(size_t idx, double thrustSP)
{
    // Check if the vectors are not empty
    if (motors[idx - 1].norm2newt.size() == 0) 
    { _message_::SIMULATOR_ERROR("[SIMUAV]: THRUST NORMAL VALUE TO NEWTON POLYNOMIAL NOT CONFIGURED"); }
    if (motors[idx - 1].norm2rps.size() == 0)
    { _message_::SIMULATOR_ERROR("[SIMUAV]: THRUST NORMAL VALUE TO RPS FOR PROP SPEED NOT CONFIGURED"); }

    // If it passes the checks then >>
    // Cache the values
    auto thrust = _compute_::evaluatePolynomial(motors[idx - 1].norm2newt, thrustSP);
    auto torque = 0.15 * motors[idx - 1].ct * thrust;       // Cq = 0.15 * ct -> Cq is the backtorque constant
    auto rps = _compute_::evaluatePolynomial(motors[idx - 1].norm2rps, thrustSP);

    // Set the values
    this->SetActuator(idx, thrust, torque, rps);
}


}   // namespace _uav_

}   // namespace _acsl_

#endif  // SIM_UAV_TPP_