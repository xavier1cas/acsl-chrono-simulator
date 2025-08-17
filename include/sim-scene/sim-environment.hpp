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
#include <limits>

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
//   Template container for storing fixed-size arrays of rigid body parameters and
//   properties for both auxiliary (ChBodyAuxRef) and default (ChBody) environment bodies.
//   Includes methods to query how many bodies are active (non-null) of each type
//   AND whether all auxiliary body properties are fully initialized.
//
// Template Parameters:
//   nxb - Number of slots allocated for auxiliary bodies (ChBodyAuxRef) in the environment.
//   nxd - Number of slots allocated for default bodies (ChBody or derived) in the environment.
//
// Members:
//   auxbody       - Shared pointers to Chrono Auxiliary reference bodies (ChBodyAuxRef), array size nxb.
//   defbody       - Shared pointers to Chrono Default bodies (ChBody, e.g., ChBodyEasyBox), array size nxd.
//   init_pos      - Initial positions in Chrono coordinates (for auxbody), array size nxb.
//   init_rot      - Initial orientation quaternions in Chrono coordinates (for auxbody), array size nxb.
//   mass          - Total mass of the auxiliary body, array size nxb.
//   InertiaXX     - Principal moments of inertia for auxiliary bodies, array size nxb.
//   InertiaXY     - Products of inertia for auxiliary bodies, array size nxb.
//   COM           - Center of mass offset frame for auxiliary bodies, array size nxb.
//   vis_obj_name  - Name of the visualization mesh file (.obj) for the auxiliary body, array size nxb.
//
// Notes:
//   - The default constructor initializes all members to safe (detectable) defaults:
//       NaN for numeric types, "NONE" for mesh name, nullptr for pointers.
//   - Helper methods are provided to count how many slots are actually assigned (non-null),
//     and to check if all auxiliary body properties are non-default (initialized).
// ----------------------------------------------------------------------------
template <int nxb, int nxd>
struct environmentstruct {
    std::array<std::shared_ptr<chrono::ChBodyAuxRef>, nxb> auxbody;   // Auxiliary bodies
    std::array<std::shared_ptr<chrono::ChBody>, nxd> defbody;         // Default bodies

    // Properties for auxiliary bodies (size nxb)
    std::array<chrono::ChVector3d, nxb> init_pos;
    std::array<chrono::ChQuaternion<>, nxb> init_rot;
    std::array<double, nxb> mass;
    std::array<chrono::ChVector3d, nxb> InertiaXX;
    std::array<chrono::ChVector3d, nxb> InertiaXY;
    std::array<chrono::ChFramed, nxb> COM;
    std::array<std::string, nxb> vis_obj_name;

    // ------------------------------------------------------------------------
    // Default constructor — initializes all members to detectable defaults
    // ------------------------------------------------------------------------
    environmentstruct() {
        for (int i = 0; i < nxb; ++i) {
            auxbody[i]     = nullptr;
            init_pos[i]    = chrono::ChVector3d(
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN()
            );
            init_rot[i]    = chrono::ChQuaternion<>(
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN()
            );
            mass[i]        = std::numeric_limits<double>::quiet_NaN();
            InertiaXX[i]   = chrono::ChVector3d(
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN()
            );
            InertiaXY[i]   = chrono::ChVector3d(
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN()
            );
            COM[i]         = chrono::ChFramed(
                chrono::ChVector3d(
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()
                ),
                chrono::ChQuaternion<>(
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN(),
                    std::numeric_limits<double>::quiet_NaN()
                )
            );
            vis_obj_name[i] = "NONE";
        }
        for (int i = 0; i < nxd; ++i) {
            defbody[i]     = nullptr;
        }
    }

    // ------------------------------------------------------------------------
    // Returns the number of non-null auxiliary bodies currently assigned
    // ------------------------------------------------------------------------
    int countAuxBodies() const {
        int count = 0;
        for (const auto& ptr : auxbody) {
            if (ptr) ++count;
        }
        return count;
    }

    // ------------------------------------------------------------------------
    // Returns the number of non-null default bodies currently assigned
    // ------------------------------------------------------------------------
    int countDefBodies() const {
        int count = 0;
        for (const auto& ptr : defbody) {
            if (ptr) ++count;
        }
        return count;
    }

    // ------------------------------------------------------------------------
    // Returns the total number of non-null bodies (auxiliary + default)
    // ------------------------------------------------------------------------
    int countTotalBodies() const {
        return countAuxBodies() + countDefBodies();
    }

    // ------------------------------------------------------------------------
    // Returns true if all property fields for auxiliary bodies are non-default
    // (ignores whether auxbody pointer is nullptr or not)
    //
    // If an error_report pointer is given, it will be filled with a message
    // describing the first uninitialized slot found.
    // ------------------------------------------------------------------------
    bool allAuxBodyPropsInitialized(std::string* error_report = nullptr) const {
        for (int i = 0; i < nxb; ++i) {
            bool valid = true;
            // Check position vector
            const auto& pos = init_pos[i];
            if (std::isnan(pos.x()) || std::isnan(pos.y()) || std::isnan(pos.z()))
            { valid = false; }

            // Check orientation quaternion
            const auto& rot = init_rot[i];
            if (std::isnan(rot.e0()) || std::isnan(rot.e1()) || std::isnan(rot.e2()) || std::isnan(rot.e3()))
            { valid = false; }

            // Check mass
            if (std::isnan(mass[i]))
            { valid = false; }

            // Check diagonal inertia
            const auto& ixx = InertiaXX[i];
            if (std::isnan(ixx.x()) || std::isnan(ixx.y()) || std::isnan(ixx.z()))
            { valid = false; }

            // Check off-diagonal inertia
            const auto& ixy = InertiaXY[i];
            if (std::isnan(ixy.x()) || std::isnan(ixy.y()) || std::isnan(ixy.z()))
            { valid = false; }

            // COM frame: position
            const auto& cpos = COM[i].GetPos();
            if (std::isnan(cpos.x()) || std::isnan(cpos.y()) || std::isnan(cpos.z()))
            { valid = false; }

            // COM frame: orientation
            const auto& crot = COM[i].GetRot();
            if (std::isnan(crot.e0()) || std::isnan(crot.e1()) || std::isnan(crot.e2()) || std::isnan(crot.e3()))
            { valid = false; }

            // Optionally: check mesh name if you want
            // if (vis_obj_name[i] == "NONE") valid = false;

            if (!valid) {
                if (error_report) {
                    *error_report = "[SIMENV]: AUX BODY " + std::to_string(i) + " HAS DEFAULT OR UNINITIALIZED PROPERTY FIELDS!";
                }
                return false;
            }
        }
        return true;
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

    // Return all Chrono Aux body pointers belonging to the environment.
    virtual std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> GetEnvAuxBodyList() = 0;

    // Return all Chrono Default body pointers belonging to the environment.
    virtual std::vector<std::shared_ptr<chrono::ChBody>> GetEnvDefBodyList() = 0;

    // Return all Chrono link/joint pointers belonging to the environment.
    virtual std::vector<std::shared_ptr<chrono::ChLinkBase>> GetEnvLinkList() = 0;

    // Finalize and all the environment components inot the Chrono physics system.
    virtual void InitiateEnv() = 0;

protected:
    // ---------------- Protected API ----------------

    // -----------------------------------------------------------------------------
    // Configuration interface for environment auxiliary reference bodies
    //
    // These virtual methods are used to configure all properties of the environment's 
    // auxiliary reference bodies (e.g., ChBodyAuxRef). Each function operates on vectors
    // that specify the value for every auxiliary body in the environment.
    //
    // Usage:
    //   - These setters should be called by environment/platform constructors or setup
    //     functions prior to simulation startup.
    //   - Vectors must have length equal to the number of auxiliary bodies (num).
    //
    // Properties configurable for auxiliary reference bodies:
    //   * File system path for visual shape assets (OBJ files)
    //   * Initial positions and orientations (in Chrono coordinates, usually NED converted)
    //   * Masses and inertia (diagonal and off-diagonal terms)
    //   * Center-of-mass frame (relative to body reference frame)
    //   * Mesh filenames for visualization/perception
    // -----------------------------------------------------------------------------
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

    // -----------------------------------------------------------------------------
    // Configuration interface for environment default ("def") reference bodies
    //
    // This virtual method is used to configure the set of environment bodies of type
    // Chrono::ChBody (or derived), distinct from auxiliary reference bodies.
    // The function operates on a vector containing all def bodies for the environment.
    //
    // Usage:
    //   - This setter should be called by environment/platform constructors or setup
    //     functions before simulation begins.
    //   - The input vector must have length equal to the number of default bodies (num).
    //
    // Properties configurable for default bodies:
    //   * Instantiation and assignment of Chrono::ChBody objects representing standard
    //     rigid bodies in the simulation (not equipped with auxiliary reference features).
    //   * Enables flexible integration of standard objects (e.g., floors, obstacles, etc,.)
    //     alongside auxiliary reference bodies.
    // -----------------------------------------------------------------------------
    // Set the simple default bodies created by ChEasy body to this
    virtual void ConfigureEnvDefBodyList(const std::vector<std::shared_ptr<chrono::ChBody>>& bodies) = 0;

    // Get the number of bodies in the environment
    virtual int GetEnvBodyCount() = 0;

};

// =====================================================================================================================
// Class: simenv<nxb, nxd>
//
// Purpose:
//   Provides a complete and extensible environment implementation capable of managing both auxiliary
//   reference bodies (e.g., Chrono::ChBodyAuxRef) and default bodies (e.g., Chrono::ChBody).
//   Implements the full polymorphic API contract defined by simenvbase, including both high-level control
//   and low-level configuration routines.
//
// Key Features:
//   - Supports separate fixed-size arrays for auxiliary and default bodies via independent
//     template parameters (nxb for aux bodies, nxd for default bodies).
//   - Seamlessly integrates with Chrono physics system via reference to ChSystemNSC.
//   - Vector-based bulk configuration and initialization of body properties (position, mass, inertia, mesh filenames, etc).
//   - Explicit distinction between auxiliary bodies (full property set, visualization mesh, COM, etc) and default bodies.
//   - Ready to be extended by derived platform-specific environment classes, facilitating
//     custom environment setup, configuration, and registration flows.
//
// Usage:
//   - Construct with a reference to the Chrono physics system.
//   - Use configuration methods to fill out body arrays and all desired properties before
//     calling InitiateEnv() to finalize and register all objects with simulation.
//
// Notes:
//   - Follows strict separation of aux and default bodies for clarity and robustness.
//   - All API override methods, body registration, and property assignment workflows
//     are documented and maintained for transparency and error detection.
//   - All internal state is encapsulated for simulation-safe lifetime management.
// =====================================================================================================================
template <int nxb, int nxd>
class simenv : public simenvbase {
public:
    simenv(chrono::ChSystemNSC& sys) : m_physics_(sys) {}

    // Public API overrides
    void SetEnvName(std::string name) override { this->name_ = name; }
    std::string GetEnvName() const override { return name_; }
    std::string GetEnvShapesdir() const override { return shapes_dir; }
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> GetEnvAuxBodyList() override { return auxbodylist; }
    std::vector<std::shared_ptr<chrono::ChBody>> GetEnvDefBodyList() override { return defbodylist; }
    std::vector<std::shared_ptr<chrono::ChLinkBase>> GetEnvLinkList() override { return linklist; }
    void InitiateEnv() override;

protected:
    // Protected API overrides
    void SetEnvShapesDir(std::string dir) override { shapes_dir = dir; }
    void ConfigureEnvInitPosList(const std::vector<chrono::ChVector3d>& positions) override;
    void ConfigureEnvInitRotList(const std::vector<chrono::ChQuaternion<>>& rotations) override;
    void ConfigureEnvMassList(const std::vector<double>& mass) override;
    void ConfigureEnvInertiaXXList(const std::vector<chrono::ChVector3d>& IXX) override;
    void ConfigureEnvInertiaXYList(const std::vector<chrono::ChVector3d>& IXY) override;
    void ConfigureEnvCOMList(const std::vector<chrono::ChFrame<>>& COM) override;
    void ConfigureEnvOBJNameList(const std::vector<std::string>& names) override;
    void ConfigureEnvDefBodyList(const std::vector<std::shared_ptr<chrono::ChBody>>& bodies) override;
    int GetEnvBodyCount() override { return envbodies.countTotalBodies(); }

private:
    chrono::ChSystemNSC& m_physics_;
    std::string name_;
    std::string shapes_dir;
    environmentstruct<nxb, nxd> envbodies;
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> auxbodylist;
    std::vector<std::shared_ptr<chrono::ChBody>> defbodylist;
    std::vector<std::shared_ptr<chrono::ChLinkBase>> linklist;
};

}   // namespace _environment_

}   // namespace _acsl_

// ---------------------------------------------------------------------------
// Implementation of template methods is in the separate .tpp file
// ---------------------------------------------------------------------------
#include "sim-environment.tpp"

#endif // SIM_ENVIRONMENT_HPP_

