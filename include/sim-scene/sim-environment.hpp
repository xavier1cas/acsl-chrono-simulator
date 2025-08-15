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
 * File:        sim-environment.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 15, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for the environment baseclass
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_ENVIRONMENT_HPP_
#define SIM_ENVIRONMENT_HPP_

// System includes
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
#include "chrono/physics/ChBodyEasy.h"

namespace _acsl_
{

namespace _environment_
{

// =====================================================================================================================
// Shared data structures
// =====================================================================================================================
// ----------------------------------------------------------------------------
// Structure: environmentstruct
//
// Purpose:
//   Template container for storing a fixed-size array of the rigid body 
//   parameters and properties for the environment.
//
// Template Parameters:
//   num - Number of bodies in the environment with the parameters 
//
// Members:
//   body          - Shared pointer to Chrono body representing the environment.
//   init_pos      - Initial position in Chrono coordinates.
//   init_rot      - Initial orientation quaternion in Chrono coordinates.
//   mass          - Total mass of the body.
//   InertiaXX     - Principal moments of inertia.
//   InertiaXY     - Products of inertia.
//   COM           - Center of mass offset frame.
//   vis_obj_name  - Name of the visualization mesh file (.obj) for the chassis.
//
// Notes:
//   - The default constructor initializes all members to safe defaults:
//       - body        = nullptr
//       - init_pos    = (0,0,0)
//       - init_rot    = identity quaternion
//       - mass        = 0.0
//       - InertiaXX   = (0,0,0)
//       - InertiaXY   = (0,0,0)
//       - COM         = zero translation, identity rotation
//       - vis_obj_name = "NONE"
// ----------------------------------------------------------------------------
template <int num>
struct environmentstruct {
    std::array<std::shared_ptr<chrono::ChBodyAuxRef>, num> body;
    std::array<chrono::ChVector3d, num> init_pos;
    std::array<chrono::ChQuaternion<>, num> init_rot;
    std::array<double, num> mass;
    std::array<chrono::ChVector3d, num> InertiaXX;
    std::array<chrono::ChVector3d, num> InertiaXY;
    std::array<chrono::ChFramed, num> COM;
    std::array<std::string, num> vis_obj_name;

    // ------------------------------------------------------------------------
    // Default constructor — initializes all members to safe default values
    // ------------------------------------------------------------------------
    environmentstruct() {
        for (int i = 0; i < num; ++i) {
            body[i]       = nullptr;                               // No Chrono body
            init_pos[i]   = chrono::ChVector3d(0, 0, 0);           // Zero position
            init_rot[i]   = chrono::ChQuaternion<>(1, 0, 0, 0);    // Identity rotation
            mass[i]       = 0.0;                                   // Zero mass
            InertiaXX[i]  = chrono::ChVector3d(0, 0, 0);           // Zero principal inertia
            InertiaXY[i]  = chrono::ChVector3d(0, 0, 0);           // Zero products of inertia
            COM[i]        = chrono::ChFramed(                      // COM = origin, identity
                               chrono::ChVector3d(0, 0, 0),
                               chrono::ChQuaternion<>(1, 0, 0, 0)
                           );
            vis_obj_name[i] = "NONE";                              // No mesh
        }
    }
};


// =====================================================================================================================
// Non-templated polymorphic base interface
//
// Purpose:
//   Defines the full ENVIRONMENT API contract so that any locale type (concrete ENVIRONMENT
//   class) can be managed through a simenvbase pointer.
//
// Structure:
//   - Public section  : General ENVIRONMENT API accessible externally.
//   - Protected section: Setup/configuration API intended for derived classes.
//
// Notes:
//   - All methods are pure virtual (= 0), forcing implementation in derived types.
//   - This allows simbridge or other managers to store and manage ENVIRONMENTs
//     polymorphically.
// =====================================================================================================================
class simenvbase {
public:
    virtual ~simenvbase() = default;

    // ---------------- Public API ----------------
    
    // Set the human-readable environment name.
    virtual void SetEnvName(std::string name) = 0;

    // Retrieve the currently assigened environment name.
    virtual std::string GetEnvName() const = 0;

    // Get the file system path string for the environment's shapes directory.
    virtual std::string GetEnvShapesdir() const = 0;

    // Return all Chrono body pointers belonging to the environment.
    virtual std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> GetEnvBodyList() = 0;

    // Return all Chrono link/joint pointers belonging to the environment.
    virtual std::vector<std::shared_ptr<chrono::ChLinkBase>> GetEnvLinkList() = 0;

    // Finalize and all the environment components inot the Chrono physics system.
    virtual void InitiateEnv() = 0;

protected:
    // ---------------- Protected API ----------------

    // Set the path to the environment's shapes directory.
    virtual void SetEnvShapesDir(std::string dir) = 0;

    // Configure the environment body initial position (converted from NED to Chrono coordinates).
    virtual void ConfigureEnvInitPosList(const std::vector<chrono::ChVector3d>& positions) = 0;

    // Configure the environment initial orientation quaternion (from NED to Chrono coords).
    virtual void ConfigureEnvInitRotList(const std::vector<chrono::ChQuaternion<>>& rotations) = 0;

    // Set the environemnt object masses (Kg)
    virtual void ConfigureEnvMassList(const std::vector<double>& mass) = 0;

    // Set principal moments of inertia (Ixx, Iyy, Izz)
    virtual void ConfigureEnvInertiaXXList(const std::vector<chrono::ChVector3d>& IXX) = 0;

    // Set products of inertia (Ixy, Ixz, Iyz).
    virtual void ConfigureEnvInertiaXYList(const std::vector<chrono::ChVector3d>& IXY) = 0;

    // Set center-of-mass frame relatively for each body.
    virtual void ConfigureEnvCOMList(const std::vector<chrono::ChFrame<>>& COM) = 0;

    // Set the environment visualization mesh filename.
    virtual void ConfigureEnvOBJNameList(const std::vector<std::string>& names) = 0;

    // Get the number of bodies in the environment
    virtual int GetEnvBodyCount() = 0;

};

// =====================================================================================================================
// Implementation of full API
//
// Purpose:
//   Provides a complete base environment implementation and implements all functions from simenvbase.
//
// Notes:
//   - Implements both high-level control API (e.g., InitiateEnv) and
//     lower-level setup/configuration API from the base.
//   - Designed to be extended by platform-specific Env classes that call
//     protected setup methods in their constructors.
// =====================================================================================================================
template <int num>
class simenv : public simenvbase {
public:
    // Constructor: stores a reference tot he Chrono simulation system.
    simenv(chrono::ChSystemNSC& sys) : m_physics_(sys) { }

    // ---------------- Public API overrides ----------------
    
    void SetEnvName(std::string name) override { this->name_ = name; };
    std::string GetEnvName() const override { return name_; }
    std::string GetEnvShapesdir() const override { return shapes_dir; }
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> GetEnvBodyList() override { return bodylist; }
    std::vector<std::shared_ptr<chrono::ChLinkBase>> GetEnvLinkList() override { return linklist; }
    void InitiateEnv() override;

protected:
    // ---------------- Protected API overrides ----------------
    void SetEnvShapesDir(std::string dir) override { shapes_dir = dir; }
    void ConfigureEnvInitPosList(const std::vector<chrono::ChVector3d>& positions) override;
    void ConfigureEnvInitRotList(const std::vector<chrono::ChQuaternion<>>& rotations) override;
    void ConfigureEnvMassList(const std::vector<double>& mass) override;
    void ConfigureEnvInertiaXXList(const std::vector<chrono::ChVector3d>& IXX) override;
    void ConfigureEnvInertiaXYList(const std::vector<chrono::ChVector3d>& IXY) override;
    void ConfigureEnvCOMList(const std::vector<chrono::ChFrame<>>& COM) override;
    void ConfigureEnvOBJNameList(const std::vector<std::string>& names) override;
    int GetEnvBodyCount() override { return num; }

private:
    // ---------------- Internal state ----------------

    chrono::ChSystemNSC& m_physics_;                             // Reference to Chrono physics system
    std::string name_;                                           // Environment name
    std::string shapes_dir;                                      // Path to visual shape files (.obj)
    environmentstruct<num> envbodies;                            // Environment body data
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> bodylist; // All body pointers for registration
    std::vector<std::shared_ptr<chrono::ChLinkBase>> linklist;   // All link pointers for registration
};

}   // namespace _environment_

}   // namespace _acsl_

// ---------------------------------------------------------------------------
// Implementation of template methods is in the separate .tpp file
// ---------------------------------------------------------------------------
#include "sim-environment.tpp"

#endif // SIM_ENVIRONMENT_HPP_

