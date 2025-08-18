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
 * File:        sim-uav.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 15, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for the uav baseclass
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_UAV_HPP_
#define SIM_UAV_HPP_

// System includes
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <array>

// ACSL physics sim includes
#include "sim-helpers.hpp"
#include "sim-messages.hpp"
#include "fkYAML/node.hpp"

// Chrono includes
#include "chrono/physics/ChSystemNSC.h"
#include "chrono/geometry/ChTriangleMesh.h"
#include "chrono/geometry/ChTriangleMeshConnected.h"
#include "chrono/assets/ChVisualShapeTriangleMesh.h"

namespace _acsl_
{
    
namespace _uav_
{


// =====================================================================================================================
// Shared data structures
// =====================================================================================================================

// ----------------------------------------------------------------------------
// Structure: chassisstruct
//
// Purpose:
//   Holds the rigid body parameters and properties for the UAV's main chassis.
//
// Members:
//   body          - Shared pointer to Chrono body representing the chassis.
//   init_pos      - Initial position in Chrono coordinates.
//   init_rot      - Initial orientation quaternion in Chrono coordinates.
//   mass          - Total mass of the chassis body.
//   InertiaXX     - Principal moments of inertia.
//   InertiaXY     - Products of inertia.
//   COM           - Center of mass offset frame.
//   vis_obj_name  - Name of the visualization mesh file (.obj) for the chassis.
// ----------------------------------------------------------------------------
struct chassisstruct {
    std::shared_ptr<chrono::ChBodyAuxRef> body;
    chrono::ChVector3d init_pos;
    chrono::ChQuaternion<> init_rot;
    double mass;
    chrono::ChVector3d InertiaXX;
    chrono::ChVector3d InertiaXY;
    chrono::ChFramed COM;
    std::string vis_obj_name;
};

// ----------------------------------------------------------------------------
// Structure: propstruct
//
// Purpose:
//   Template container for storing a fixed-size array of Chrono body pointers
//   that represent the UAV's propeller hubs/assemblies.
//
// Template Parameters:
//   num - Number of propellers for this UAV type
// ----------------------------------------------------------------------------
template <int num>
struct propstruct {
    // Fixed-size array of propeller body pointers
    std::array<std::shared_ptr<chrono::ChBodyAuxRef>, num> body;
};

// ----------------------------------------------------------------------------
// Structure: m_states
//
// Purpose:
//   Stores the UAV's runtime state data relative to the NED frame, including
//   translation, rotation, and kinematic properties.
//
// Members:
//   time  - Simulation time stamp.
//   pos   - Position in NED coordinates.
//   vel   - Linear velocity in NED coordinates.
//   acc   - Linear acceleration in NED coordinates.
//   eul   - Euler angles (rotation) in NED coordinates.
//   quat  - Orientation quaternion in NED coordinates.
//   ovel  - Angular velocity in body frame (relative to NED).
//   oacc  - Angular acceleration in body frame (relative to NED).
// ----------------------------------------------------------------------------
struct m_states {
    double time;
    chrono::ChVector3d pos;
    chrono::ChVector3d vel;
    chrono::ChVector3d acc;
    chrono::ChVector3d eul;
    chrono::ChQuaterniond quat;
    chrono::ChVector3d ovel;
    chrono::ChVector3d oacc;
};


// =====================================================================================================================
// Non-templated polymorphic base interface
//
// Purpose:
//   Defines the full UAV API contract so that any platform type (concrete UAV
//   class) can be managed through a simuavbase pointer.
//
// Structure:
//   - Public section  : General UAV API accessible externally.
//   - Protected section: Setup/configuration API intended for derived classes.
//
// Notes:
//   - All methods are pure virtual (= 0), forcing implementation in derived types.
//   - This allows simbridge or other managers to store and manage UAVs
//     polymorphically.
// =====================================================================================================================
class simuavbase {
public:
    virtual ~simuavbase() = default;

    // ---------------- Public API ----------------

    // Set the human-readable platform name.
    virtual void SetUAVPlatformName(std::string name) = 0;

    // Retrieve the currently assigned UAV platform name.
    virtual std::string GetUAVPlatformName() const = 0;

    // Get the file system path string for the UAV's shapes directory.
    virtual std::string GetUAVShapesDir() const = 0;

    // Return the inertial NED frame as a Chrono ChFrame object.
    virtual chrono::ChFrame<> GetInertialNEDFrame() = 0;

    // Access the chassisstruct for modifying chassis parameters directly.
    virtual chassisstruct& GetUAVChassis() = 0;

    // Return all Chrono body pointers belonging to the UAV.
    virtual std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> GetUAVBodyList() = 0;

    // Return all Chrono link/joint pointers belonging to the UAV.
    virtual std::vector<std::shared_ptr<chrono::ChLinkBase>> GetUAVLinkList() = 0;

    // Finalize and add all UAV components into the Chrono physics system.
    virtual void InitiateUAV() = 0;

    // Retrieve the UAV's full state data in the NED frame.
    virtual m_states GetUAVStateData() = 0;

protected:
    // ---------------- Protected API ----------------

    // Set the path to the UAV's shapes directory.
    virtual void SetUAVShapesDir(std::string dir) = 0;

    // Get the inertial NED frame auxiliary body pointer.
    virtual std::shared_ptr<chrono::ChBodyAuxRef> GetInertialNEDFrameAuxBody() = 0;

    // Create and set up the inertial NED frame in the simulation.
    virtual void SetupInertialNEDFrame() = 0;

    // Configure the chassis initial position (converted from NED to Chrono coordinates).
    virtual void ConfigureUAVChassisInitPos(chrono::ChVector3d pos) = 0;

    // Configure the chassis initial orientation quaternion (from NED to Chrono coords).
    virtual void ConfigureUAVChassisInitRot(chrono::ChQuaternion<> rot) = 0;

    // Set the chassis mass (kg).
    virtual void ConfigureUAVChassisMass(double mass) = 0;

    // Set principal moments of inertia (Ixx, Iyy, Izz).
    virtual void ConfigureUAVChassisInertiaXX(chrono::ChVector3d IXX) = 0;

    // Set products of inertia (Ixy, Ixz, Iyz).
    virtual void ConfigureUAVChassisInertiaXY(chrono::ChVector3d IXY) = 0;

    // Set center-of-mass frame relative to chassis reference.
    virtual void ConfigureUAVChassisCOM(chrono::ChFrame<> COM) = 0;

    // Set chassis visualization mesh filename.
    virtual void ConfigureUAVChassisOBJName(std::string name) = 0;

    // Create, initialize, and register chassis body in internal body list.
    virtual void InitiateUAVChassis() = 0;

    // Get number of UAV propellers.
    virtual int GetPropCount() = 0;
};


// =====================================================================================================================
// Templated implementation of full API
//
// Purpose:
//   Provides a complete base UAV implementation that is parameterized by the
//   number of propellers (nop) and implements all functions from simuavbase.
//
// Notes:
//   - Implements both high-level control API (e.g., InitiateUAV) and
//     lower-level setup/configuration API from the base.
//   - Designed to be extended by platform-specific UAV classes that call
//     protected setup methods in their constructors.
// =====================================================================================================================
template <int nop>
class simuav : public simuavbase {
public:
    // Constructor: stores a reference to the Chrono simulation system.
    simuav(chrono::ChSystemNSC& sys) : m_physics_(sys) { }

    // ---------------- Public API overrides ----------------

    void SetUAVPlatformName(std::string name) override { this->name_ = name; }
    std::string GetUAVPlatformName() const override { return name_; }
    std::string GetUAVShapesDir() const override { return shapes_dir; }

    chrono::ChFrame<> GetInertialNEDFrame() override;
    chassisstruct& GetUAVChassis() override { return chassis; }
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> GetUAVBodyList() override { return bodylist; }
    std::vector<std::shared_ptr<chrono::ChLinkBase>> GetUAVLinkList() override { return linklist; }
    void InitiateUAV() override;
    m_states GetUAVStateData() override;

protected:
    // ---------------- Protected API overrides ----------------

    void SetUAVShapesDir(std::string dir) override { shapes_dir = dir; }
    std::shared_ptr<chrono::ChBodyAuxRef> GetInertialNEDFrameAuxBody() override { return InertialFrameNED; }
    void SetupInertialNEDFrame() override;

    void ConfigureUAVChassisInitPos(chrono::ChVector3d pos) override { 
         chassis.init_pos = _transformations_::GetChronoPosFromNED(pos); }
    void ConfigureUAVChassisInitRot(chrono::ChQuaternion<> rot) override { 
         chassis.init_rot = _transformations_::GetChronoOrientaitonFromNED(rot); }

    void ConfigureUAVChassisMass(double mass) override { chassis.mass = mass; }
    void ConfigureUAVChassisInertiaXX(chrono::ChVector3d IXX) override { chassis.InertiaXX = IXX; }
    void ConfigureUAVChassisInertiaXY(chrono::ChVector3d IXY) override { chassis.InertiaXY = IXY; }
    void ConfigureUAVChassisCOM(chrono::ChFrame<> COM) override { chassis.COM = COM; }
    void ConfigureUAVChassisOBJName(std::string name) override { chassis.vis_obj_name = name; }
    void InitiateUAVChassis() override;
    int GetPropCount() override { return nop; }

private:
    // ---------------- Internal state ----------------

    chrono::ChSystemNSC& m_physics_;                             // Reference to Chrono physics system
    std::string name_;                                           // UAV name
    std::string shapes_dir;                                      // Path to visual shape files (.obj)
    std::shared_ptr<chrono::ChBodyAuxRef> InertialFrameNED;      // Inertial NED frame body
    chassisstruct chassis;                                       // Chassis data
    propstruct<nop> props;                                       // Propeller data
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> bodylist; // All body pointers for registration
    std::vector<std::shared_ptr<chrono::ChLinkBase>> linklist;   // All link pointers for registration
};

}   // namespace _uav_

}   // namespace _acsl_

// ---------------------------------------------------------------------------
// Implementation of template methods is in the separate .tpp file
// ---------------------------------------------------------------------------
#include "sim-uav.tpp"


#endif  // SIM_UAV_HPP_