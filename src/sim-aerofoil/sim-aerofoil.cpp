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
 * File:        sim-aerofoil.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        December 13, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description:
 *   Implementation of the simairfoil base class, including:
 *     - Constructor-based loading of airfoil models from AirFoilType.
 *     - Generic evaluation of CL, CD, and CM using CoeffModel.
 *     - Support for a "NONE" airfoil type that disables aerodynamic modeling.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-aerofoil.hpp"

namespace _acsl_
{
namespace _uav_
{
namespace _aerofoil_
{

// =====================================================================================================================
// simairfoil::simairfoil(type)
//
// Purpose:
//   Constructor that delegates to load_from_enum() to configure the internal
//   coefficient models based on the specified AirFoilType.
// =====================================================================================================================
simairfoil::simairfoil(AirFoilType type)
{
  this->load_from_enum(type);
}

// =====================================================================================================================
// simairfoil::ComputeCL(alpha)
//
// Purpose:
//   Evaluate CL(alpha) using the stored CL model. Returns 0.0 if no airfoil
//   has been loaded (AirFoilType::NONE).
// =====================================================================================================================
double simairfoil::ComputeCL(double alpha) 
{
    if (!HasAeroFoil()) { return 0.0; }
    return eval(cl_model_, alpha);
}


// =====================================================================================================================
// simairfoil::ComputeCD(alpha)
//
// Purpose:
//   Evaluate CD(alpha) using the stored CD model. Returns 0.0 if no airfoil
//   has been loaded (AirFoilType::NONE).
// =====================================================================================================================
double simairfoil::ComputeCD(double alpha) 
{
    if (!this->HasAeroFoil()) { return 0.0; }
    return eval(cd_model_, alpha);
}


// =====================================================================================================================
// simairfoil::ComputeCM(alpha)
//
// Purpose:
//   Evaluate CM(alpha) using the stored CM model. Returns 0.0 if no airfoil
//   has been loaded (AirfoilType::NONE).
// =====================================================================================================================
double simairfoil::ComputeCM(double alpha) 
{
    if (!this->HasAeroFoil()) { return 0.0; }
    return eval(cm_model_, alpha);
}


// =====================================================================================================================
// simairfoil::eval(model, x)
//
// Purpose:
//   Generic evaluation of y(x) = (A * B(x))^T * V(x) for a given CoeffModel.
//
// Implementation details:
//   - Builds B(x) such that B_k(x) = tanh(B_scale(k)*x + B_offset(k)).
//   - Builds V(x) such that V_i(x) = x^(V_exponents(i)).
//   - Computes W = A * B(x), then returns y(x) = W^T * V(x).
// =====================================================================================================================
double simairfoil::eval(const CoeffModel& m, double x)
{
    const int nB = static_cast<int>(m.B_scale.size());
    const int nV = static_cast<int>(m.V_exponents.size());

    Eigen::VectorXd B(nB);
    Eigen::VectorXd V(nV);

    // Build B(x): tanh basis functions
    for (int k = 0; k < nB; ++k) {
        double t = m.B_scale(k) * x + m.B_offset(k);
        B(k) = std::tanh(t);
    }

    // Build V(x): polynomial basis functions
    for (int i = 0; i < nV; ++i) {
        int p = m.V_exponents(i);
        V(i) = std::pow(x, p);
    }

    // Combine into y(x) = (A * B)^T * V
    Eigen::VectorXd W = m.A * B;
    return W.dot(V);
}


// =====================================================================================================================
// simairfoil::load_from_enum(type)
//
// Purpose:
//   Load the CL, CD, and CM coefficient models based on the specified airfoil
//   type. If AirfoilType::NONE is selected, no models are loaded and all
//   coefficient evaluation functions will return zero.
// =====================================================================================================================
void simairfoil::load_from_enum(AirFoilType type)
{
    // Default: no airfoil loaded
    this->SetAeroFoilAbsent();

    // If no airfoil is requested, leave all models in their default-constructed
    // state and return immediately.
    if (type == AirFoilType::NONE) { return; }

    // Temporary coefficient models that will be loaded from the airfoil data
    // module and then assigned to the simairfoil instance.
    CoeffModel cl, cd, cm;

    switch (type) {
    
    // NACA0012
    case AirFoilType::NACA0012:
        // Load NACA0012 dataset into cl, cd, cm
        ::_acsl_::_uav_::_aerofoil_::loadNACA0012(cl, cd, cm);
        // Set the aerofoil is present
        this->SetAeroFoilPresent();
        break;
      
    // SYNTAX TO ADD
    // case AirFoilType::TYPE:
    //  // Load TYPE dataset into cl, cd, cm
    //  ::_acsl_::_uav_::_aerofoil_::loadTYPE(cl, cd, cm);
    //  // Set the aerofoil is present
    //  this->AeroFoilPresent();
    //  break;

    default:
        ::_acsl_::_message_::SIMULATOR_ERROR("[SIMUAV]: simairfoil::load_from_enum: UNSUPPORTED AirFoilType");
    }

    if (this->HasAeroFoil()) {
        this->SetCLModel(cl);
        this->SetCDModel(cd);
        this->SetCMModel(cm);
    }
}


}   // namespace _aerofoil_
}   // namespace _uav_
}   // namespace _acsl_

