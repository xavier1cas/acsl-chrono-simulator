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
//   Configure initial position states for all environment bodies in the simulation.
//   The input positions are provided as NED coordinates and transformed to Chrono coordinate frame.
//
// Workflow:
//   1. Validate that the input vector size matches the expected number of bodies.
//   2. For each body, perform a coordinate transformation from NED to Chrono coordinates.
//   3. Assign the transformed positions to the internal fixed-size position array.
//
// Notes:
//   - Throws SIM_ERROR if the vector size does not match expected.
//   - Utilizes the _transformations_ utility namespace for coordinate conversion.
// =========================================================================================================
template <int num>
void simenv<num>::ConfigureEnvInitPosList(const std::vector<chrono::ChVector3d>& positions) {
    if (positions.size() != num) {
        _message_::SIMULATOR_ERROR("[SIMENV]: POSITIONS VECTOR SIZE MISMATCH - EXPECTED " + std::to_string(num));
    }
    for (int i = 0; i < num; ++i) {
        envbodies.init_pos[i] = _transformations_::GetChronoPosFromNED(positions[i]);
    }
}


// =========================================================================================================
// ConfigureEnvInitRotList()
//
// Purpose:
//   Configure initial orientation quaternions for all environment bodies in the simulation.
//   Input rotations are expressed in NED frame and converted to Chrono’s orientation conventions.
//
// Workflow:
//   1. Check that the input rotation vector size matches the number of environment bodies.
//   2. Convert each rotation quaternion from NED to Chrono frame using transformation utilities.
//   3. Store the converted quaternions in the internal array of initial rotations.
//
// Notes:
//   - Throws SIM_ERROR if vector size mismatches.
//   - Uses the _transformations_ namespace functions for conversion.
// =========================================================================================================
template <int num>
void simenv<num>::ConfigureEnvInitRotList(const std::vector<chrono::ChQuaternion<>>& rotations) {
    if (rotations.size() != num) {
        _message_::SIMULATOR_ERROR("[SIMENV]: ROTATIONS VECTOR SIZE MISMATCH - EXPECTED " + std::to_string(num));
    }
    for (int i = 0; i < num; ++i) {
        envbodies.init_rot[i] = _transformations_::GetChronoOrientaitonFromNED(rotations[i]);
    }
}


// =========================================================================================================
// ConfigureEnvMassList()
//
// Purpose:
//   Assign the mass (in kilograms) for each environment body.
//
// Workflow:
//   1. Validate that the size of the input mass vector matches the fixed number of environment bodies.
//   2. Assign each mass value to the corresponding entry in the mass array.
//
// Notes:
//   - Mass values must be positive and in SI units (kg).
//   - Throws SIM_ERROR if vector size mismatches.
// =========================================================================================================
template <int num>
void simenv<num>::ConfigureEnvMassList(const std::vector<double>& mass) {
    if (mass.size() != num) {
        _message_::SIMULATOR_ERROR("[SIMENV]: MASS VECTOR SIZE MISMATCH - EXPECTED " + std::to_string(num));
    }
    for (int i = 0; i < num; ++i) {
        envbodies.mass[i] = mass[i];
    }
}


// =========================================================================================================
// ConfigureEnvInertiaXXList()
//
// Purpose:
//   Set the principal moments of inertia (Ixx, Iyy, Izz) for each environment body (kg·m²).
//
// Workflow:
//   1. Validate that the number of vectors matches the body count.
//   2. Assign each (Ixx, Iyy, Izz) triple to inertiaXX array.
//
// Notes:
//   - Throws SIM_ERROR if vector size mismatches.
// =========================================================================================================
template <int num>
void simenv<num>::ConfigureEnvInertiaXXList(const std::vector<chrono::ChVector3d>& IXX) {
    if (IXX.size() != num) {
        _message_::SIMULATOR_ERROR("[SIMENV]: INERTIAXX VECTOR SIZE MISMATCH - EXPECTED " + std::to_string(num));
    }
    for (int i = 0; i < num; ++i) {
        envbodies.InertiaXX[i] = IXX[i];
        InertialFrameNED->GetRot()    // NED orientation as quaternion
    );

    }
}


// =========================================================================================================
// ConfigureEnvInertiaXYList()
//
// Purpose:
//   Set the products of inertia (Ixy, Ixz, Iyz) for each environment body.
//
// Workflow:
//   1. Validate that the vector size matches the body count.
//   2. Assign (Ixy, Ixz, Iyz) triple to inertiaXY array.
//
// Notes:
//   - Throws SIM_ERROR if vector size mismatches.
// =========================================================================================================
template <int num>
void simenv<num>::ConfigureEnvInertiaXYList(const std::vector<chrono::ChVector3d>& IXY) {
    if (IXY.size() != num) {
        _message_::SIMULATOR_ERROR("[SIMENV]: INERTIAXY VECTOR SIZE MISMATCH - EXPECTED " + std::to_string(num));
    }
    for (int i = 0; i < num; ++i) {
        envbodies.InertiaXY[i] = IXY[i];
    }
}


// =========================================================================================================
// ConfigureEnvCOMList()
//
// Purpose:
//   Set the center-of-mass (COM) reference frame for each environment body (offset from ref frame).
//
// Workflow:
//   1. Validate that the COM frame vector matches the body count.
//   2. Assign each COM frame to the COM array.
//
// Notes:
//   - Throws SIM_ERROR if vector size mismatches.
// =========================================================================================================
template <int num>
void simenv<num>::ConfigureEnvCOMList(const std::vector<chrono::ChFrame<>>& COM) {
    if (COM.size() != num) {
        _message_::SIMULATOR_ERROR("[SIMENV]: COM VECTOR SIZE MISMATCH - EXPECTED " + std::to_string(num));
    }
    for (int i = 0; i < num; ++i) {
        envbodies.COM[i] = COM[i];
    }
}


// =========================================================================================================
// ConfigureEnvOBJNameList()
//
// Purpose:
//   Set the visualization mesh file names (.obj) for all environment bodies.
//
// Workflow:
//   1. Validate that the names vector matches the body count.
//   2. Assign file name to vis_obj_name array.
//   3. If name is empty string, assign "NONE" placeholder from constructor.
//
// Notes:
//   - "NONE" will be skipped by InitiateEnv() mesh loading.
//   - Throws SIM_ERROR if vector size mismatches.
// =========================================================================================================
template <int num>
void simenv<num>::ConfigureEnvOBJNameList(const std::vector<std::string>& names) {
    if (names.size() != num) {
        _message_::SIMULATOR_ERROR("[SIMENV]: OBJ NAME VECTOR SIZE MISMATCH - EXPECTED " + std::to_string(num));
    }
    for (int i = 0; i < num; ++i) {
        envbodies.vis_obj_name[i] = names[i].empty() ? "NONE" : names[i];
    }
}


// =========================================================================================================
// InitiateEnv()
//
// Purpose:
//   Initializes all environment bodies: allocates Chrono body objects, sets up basic physical and geometric
//   properties, attaches visualization meshes (if specified), and registers them with the internal body list.
//   Finalizes registration by adding all environment bodies and links to the Chrono simulation system.
//
// Workflow:
//   For each environment body:
//     1. If the body pointer is null, throw a SIMULATOR_ERROR and skip further processing.
//     2. Set metadata and initial conditions.
//     3. Optionally attach a mesh if vis_obj_name != "NONE".
//     4. Register the body in the internal list.
//     5. Add all registered bodies and links to the physics system (m_physics_).
//
// Notes:
//   - Skips mesh if vis_obj_name[i] == "NONE" and reports this via SIMULATOR_INFO.
//   - Throws SIMULATOR_ERROR if body pointer is null—body is not registered.
//   - This routine must be called before simulation starts.
// =========================================================================================================
template <int num>
void simenv<num>::InitiateEnv() {
    for (int i = 0; i < num; ++i) {
        // ------------------------------------------------------------------------
        // STEP 1 – Check for valid body pointer
        // ------------------------------------------------------------------------
        if (!envbodies.body[i]) {
            _message_::SIMULATOR_ERROR("[SIMENV]: BODY POINTER IS NULL FOR ENV_BODY_" + std::to_string(i)
                                                + " | CANNOT INITIALIZE OR REGISTER THIS BODY.");
            continue; // skip to next body
        }

        // ------------------------------------------------------------------------
        // STEP 2 – Set all metadata and physical properties
        // ------------------------------------------------------------------------
        envbodies.body[i]->SetName("env_body_" + std::to_string(i));
        envbodies.body[i]->SetPos(envbodies.init_pos[i]);
        envbodies.body[i]->SetRot(envbodies.init_rot[i]);
        envbodies.body[i]->SetFixed(true);
        envbodies.body[i]->SetMass(envbodies.mass[i]);
        envbodies.body[i]->SetInertiaXX(envbodies.InertiaXX[i]);
        envbodies.body[i]->SetInertiaXY(envbodies.InertiaXY[i]);
        envbodies.body[i]->SetFrameCOMToRef(envbodies.COM[i]);

        // ------------------------------------------------------------------------
        // STEP 3 – Attach visual mesh if specified
        // ------------------------------------------------------------------------
        if (envbodies.vis_obj_name[i] != "NONE") {
            auto trimesh = chrono::ChTriangleMeshConnected::CreateFromWavefrontFile(
                shapes_dir + envbodies.vis_obj_name[i], true, true
            );
            auto trimesh_shape = chrono_types::make_shared<chrono::ChVisualShapeTriangleMesh>();
            trimesh_shape->SetMesh(trimesh);
            trimesh_shape->SetMutable(false);
            envbodies.body[i]->AddVisualShape(
                trimesh_shape,
                chrono::ChFramed(
                    chrono::ChVector3d(0, 0, 0),
                    chrono::ChQuaternion<>(1, 0, 0, 0)
                )
            );
        } else {
            _message_::SIMULATOR_INFO("NO VISUALIZATION MESH PROVIDED FOR ENV_BODY_" + std::to_string(i) +
                                      " | SKIPPING MESH ATTACHMENT.");
        }

        // ------------------------------------------------------------------------
        // STEP 4 – Register body in internal list
        // ------------------------------------------------------------------------
        bodylist.push_back(envbodies.body[i]);
    }

    // ================================================================================
    // STEP 5 – Final registration: add all environment bodies and links to physics system
    //   Mirrors UAV workflow: ensure all are fully integrated into the Chrono simulation
    // ================================================================================
    for (auto& body : bodylist) {
        m_physics_.Add(body);
    }
    for (auto& link : linklist) {
        m_physics_.Add(link);
    }
    // Environment is now fully part of the simulation world
}




}   // namespace _environment_

}   // namespace _acsl_

#endif  // SIM_ENVIRONMENT_TPP_