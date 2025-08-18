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
 * File:        sim-environment.tpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 15, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Class file for the environment baseclass
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_ENVIRONMENT_TPP_
#define SIM_ENVIRONMENT_TPP_

#include "sim-environment.hpp"

namespace _acsl_
{

namespace _environment_
{

// =========================================================================================================
// ConfigureEnvInitPosList()
//
// Purpose:
//   Configure initial position states for all auxiliary reference bodies in the simulation.
//   The input positions are provided as NED coordinates and transformed to Chrono coordinate frame.
//
// Workflow:
//   1. For each auxiliary body, perform a coordinate transformation from NED to Chrono coordinates.
//   2. Assign the transformed positions to the fixed-size auxbody position array.
//
// Notes:
//   - Utilizes the _transformations_ utility namespace for coordinate conversion.
//   - Vectors must have length equal to nxb (number of aux bodies).
// =========================================================================================================
template <int nxb, int nxd>
void simenv<nxb, nxd>::ConfigureEnvInitPosList(const std::vector<chrono::ChVector3d>& positions) {
    for (int i = 0; i < nxb; ++i) {
        envbodies.init_pos[i] = _transformations_::GetChronoPosFromNED(positions[i]);
    }
}

// =========================================================================================================
// ConfigureEnvInitRotList()
//
// Purpose:
//   Configure initial orientation quaternions for all auxiliary reference bodies in the simulation.
//   Input rotations are expressed in NED frame and converted to Chrono’s orientation conventions.
//
// Workflow:
//   1. For each auxbody, convert quaternion from NED to Chrono frame using transformation utilities.
//   2. Store the converted quaternions in the fixed-size auxbody rotation array.
//
// Notes:
//   - Uses the _transformations_ namespace functions for conversion.
//   - Vectors must have length equal to nxb.
// =========================================================================================================
template <int nxb, int nxd>
void simenv<nxb, nxd>::ConfigureEnvInitRotList(const std::vector<chrono::ChQuaternion<>>& rotations) {
    for (int i = 0; i < nxb; ++i) {
        envbodies.init_rot[i] = _transformations_::GetChronoOrientaitonFromNED(rotations[i]);
    }
}

// =========================================================================================================
// ConfigureEnvMassList()
//
// Purpose:
//   Assign the mass (in kilograms) for each auxiliary reference body.
//
// Workflow:
//   1. For each auxbody, assign each mass value provided (must be non-negative) to the auxbody mass array.
//
// Notes:
//   - Mass values must be positive and in SI units (kg).
//   - Vector length must be nxb.
// =========================================================================================================
template <int nxb, int nxd>
void simenv<nxb, nxd>::ConfigureEnvMassList(const std::vector<double>& mass) {
    for (int i = 0; i < nxb; ++i) {
        if (mass[i] < 0) {
            _message_::SIMULATOR_ERROR("[SIMENV]: MASS OF AUX BODY NUMBER " + std::to_string(i) + " IS NEGATIVE!");
        } else {
            envbodies.mass[i] = mass[i];
        }
    }
}

// =========================================================================================================
// ConfigureEnvInertiaXXList()
//
// Purpose:
//   Set the principal moments of inertia (Ixx, Iyy, Izz) for each auxiliary reference body.
//
// Workflow:
//   1. For each auxbody, assign (Ixx, Iyy, Izz) triple to the auxbody inertiaXX array.
//
// Notes:
//   - Vector length must be nxb.
// =========================================================================================================
template <int nxb, int nxd>
void simenv<nxb, nxd>::ConfigureEnvInertiaXXList(const std::vector<chrono::ChVector3d>& IXX) {
    for (int i = 0; i < nxb; ++i) {
        envbodies.InertiaXX[i] = IXX[i];
    }
}

// =========================================================================================================
// ConfigureEnvInertiaXYList()
//
// Purpose:
//   Set the products of inertia (Ixy, Ixz, Iyz) for each auxiliary reference body.
//
// Workflow:
//   1. For each auxbody, assign (Ixy, Ixz, Iyz) triple to the auxbody inertiaXY array.
//
// Notes:
//   - Vector length must be nxb.
// =========================================================================================================
template <int nxb, int nxd>
void simenv<nxb, nxd>::ConfigureEnvInertiaXYList(const std::vector<chrono::ChVector3d>& IXY) {
    for (int i = 0; i < nxb; ++i) {
        envbodies.InertiaXY[i] = IXY[i];
    }
}

// =========================================================================================================
// ConfigureEnvCOMList()
//
// Purpose:
//   Set the center-of-mass (COM) reference frame for each auxiliary reference body.
//
// Workflow:
//   1. For each auxbody, assign the COM frame (position+rotation) to the auxbody COM array.
//
// Notes:
//   - Vector length must be nxb.
// =========================================================================================================
template <int nxb, int nxd>
void simenv<nxb, nxd>::ConfigureEnvCOMList(const std::vector<chrono::ChFrame<>>& COM) {
    for (int i = 0; i < nxb; ++i) {
        envbodies.COM[i] = COM[i];
    }
}

// =========================================================================================================
// ConfigureEnvOBJNameList()
//
// Purpose:
//   Set the visualization mesh file names (.obj) for all auxiliary reference bodies.
//
// Workflow:
//   1. For each auxbody, assign file name to vis_obj_name array.
//   2. If name is empty string, assign "NONE" placeholder from constructor.
//
// Notes:
//   - "NONE" will be skipped by InitiateEnv() mesh loading.
//   - Vector length must be nxb.
// =========================================================================================================
template <int nxb, int nxd>
void simenv<nxb, nxd>::ConfigureEnvOBJNameList(const std::vector<std::string>& names) {
    for (int i = 0; i < nxb; ++i) {
        envbodies.vis_obj_name[i] = names[i].empty() ? "NONE" : names[i];
    }
}

// =========================================================================================================
// ConfigureEnvDefBodyList()
//
// Purpose:
//   Set and assign the Chrono default rigid bodies (e.g., ChBody, ChBodyEasyBox) for the environment.
//   Populates only the defbody array, not auxbody properties.
//
// Workflow:
//   1. For each defbody, assign the Chrono body pointer provided.
//
// Notes:
//   - Vector length must be nxd.
// =========================================================================================================
template <int nxb, int nxd>
void simenv<nxb, nxd>::ConfigureEnvDefBodyList(const std::vector<std::shared_ptr<chrono::ChBody>>& bodies) {
    for (int i = 0; i < nxd; ++i) {
        envbodies.defbody[i] = bodies[i];
    }
}


// =========================================================================================================
// InitiateEnv()
//
// Purpose:
//   Initializes all environment bodies: sets up auxiliary reference bodies (e.g., ChBodyAuxRef), normal
//   reference bodies (e.g., ChBody or derived), their physical properties, visualization meshes, and
//   registers everything into the Chrono simulation system. Handles both body types distinctly and
//   ensures robust error and info reporting.
//
// Workflow:
//   0. Validate that all auxiliary body properties (positions, rotations, mass, inertia, COM, etc.)
//      are filled (i.e., not left at defaults/NaN) using allAuxBodyPropsInitialized().
//      - If any slot is uninitialized, aborts registration and reports the first uninitialized slot.
//   1. For each auxiliary reference body (envbodies.auxbody; size nxb):
//       a. Set metadata and all physical properties.
//       c. Attach a visualization mesh if specified (vis_obj_name != "NONE").
//       d. Add the body to auxbodylist for later registration.
//   2. For each normal reference body (envbodies.defbody; size nxd):
//       a. Validate the body pointer; log an error and skip if null.
//       b. Add the body to defbodylist for later registration.
//   3. Register all valid auxiliary reference bodies, normal reference bodies, and links to the Chrono physics system.
//
// Notes:
//   - Skips visualization mesh attachment if vis_obj_name[i] == "NONE".
//   - Reports errors for null body pointers and/or uninitialized aux body properties.
//   - Handles both auxbody and defbody types independently, so you can combine multiple body types in the same environment.
//   - This function must be called before the simulation starts.
// =========================================================================================================
template <int nxb, int nxd>
void simenv<nxb, nxd>::InitiateEnv() {

    // ------------------------------------------------------------------------------
    // STEP 0: Check that all auxbody properties have been set (not left at NaN/defaults)
    // ------------------------------------------------------------------------------
    std::string prop_error;
    if (!envbodies.allAuxBodyPropsInitialized(&prop_error)) {
        _message_::SIMULATOR_ERROR(prop_error);
        return;
    }

    // ------------------------------------------------------------------------------
    // STEP 1: Handle all auxiliary reference bodies (size nxb)
    // ------------------------------------------------------------------------------
    for (int i = 0; i < nxb; ++i) {
        envbodies.auxbody[i] = chrono_types::make_shared<chrono::ChBodyAuxRef>();
        envbodies.auxbody[i]->SetName("env_auxbody_" + std::to_string(i));
        envbodies.auxbody[i]->SetPos(envbodies.init_pos[i]);
        envbodies.auxbody[i]->SetRot(envbodies.init_rot[i]);
        envbodies.auxbody[i]->SetFixed(true);
        envbodies.auxbody[i]->SetMass(envbodies.mass[i]);
        envbodies.auxbody[i]->SetInertiaXX(envbodies.InertiaXX[i]);
        envbodies.auxbody[i]->SetInertiaXY(envbodies.InertiaXY[i]);
        envbodies.auxbody[i]->SetFrameCOMToRef(envbodies.COM[i]);

        if (envbodies.vis_obj_name[i] != "NONE") {
            auto trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(
                shapes_dir + envbodies.vis_obj_name[i], true, true
            );
            auto trimesh_shape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetMutable(false);
            envbodies.auxbody[i]->AddVisualShape(
                trimesh_shape,
                chrono::ChFramed(
                    chrono::ChVector3d(0, 0, 0),
                    chrono::ChQuaternion<>(1, 0, 0, 0)
                )
            );
        } else {
            _message_::SIMULATOR_INFO("NO VISUALIZATION MESH PROVIDED FOR ENV_AUX_BODY_" + std::to_string(i)
                                      + " | SKIPPING MESH ATTACHMENT.");
        }

        auxbodylist.push_back(envbodies.auxbody[i]);
    }

    // ------------------------------------------------------------------------------
    // STEP 2: Handle all normal reference bodies (size nxd)
    // ------------------------------------------------------------------------------
    for (int i = 0; i < nxd; ++i) {
        if (!envbodies.defbody[i]) {
            _message_::SIMULATOR_INFO("[SIMENV]: BODY POINTER IS NULL FOR ENV_DEF_BODY_" + std::to_string(i)
                                      + " | CANNOT INITIALIZE OR REGISTER THIS BODY.");
            continue;
        }
        envbodies.defbody[i]->SetName("env_defbody_" + std::to_string(i));
        defbodylist.push_back(envbodies.defbody[i]);
    }

    // ------------------------------------------------------------------------------
    // STEP 3: Final registration — push all valid bodies & links to Chrono physics system.
    // ------------------------------------------------------------------------------
    for (auto& body : auxbodylist) {
        if (body) {
            _message_::SIMULATOR_INFO("[SIMENV]: ADDING " + body->GetName() + " TO SYSTEM"); 
            m_physics_.Add(body);
        }
    }
    for (auto& link : linklist) {
        if (link) {
            m_physics_.Add(link);
        }
    }
    for (auto& body : defbodylist) {
        if (body) {
            _message_::SIMULATOR_INFO("[SIMENV]: ADDING " + body->GetName() + " TO SYSTEM");
            m_physics_.Add(body);
        }
    }

    // All environment bodies and links are now registered with the physics system.
}


}   // namespace _environment_

}   // namespace _acsl_

#endif  // SIM_ENVIRONMENT_TPP_