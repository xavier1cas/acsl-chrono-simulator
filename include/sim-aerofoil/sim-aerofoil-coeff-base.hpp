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
 * File:        sim-aerofoil-coeff-base.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        December 13, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for the common aerofoil coefficient base.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/
#ifndef SIM_AEROFOIL_COEFF_BASE_HPP_
#define SIM_AEROFOIL_COEFF_BASE_HPP_

// Eigen Includes
#include <Eigen/Dense>

namespace _acsl_ 
{
namespace _uav_ 
{
namespace _aerofoil_ 
{

// =====================================================================================================================
// Struct: CoeffModel
//
// Purpose:
//   Stores all parameters required to evaluate a single aerodynamic coefficient
//   curve (e.g., CL, CD, or CM) as a function of a scalar input x
//   (typically angle of attack).
//
// Members:
//   A           - Coefficient matrix used to combine tanh-based basis functions.
//   B_scale     - Scale factors for the tanh arguments; defines B_k(x) = tanh(scale_k * x + offset_k).
//   B_offset    - Offset terms for the tanh arguments.
//   V_exponents - Polynomial exponents used to construct the V(x) basis,
//                 where V_i(x) = x^(exponent_i).
//
// Description:
//   This shared data structure stores aerodynamic coefficient models (CL, CD, CM)
//   used by the generic SimAirfoil base class and by airfoil-specific data
//   headers (e.g., NACA0012.hpp).
//
//   The model represents a scalar function:
//
//       y(x) = (A * B(x))^T * V(x)
//
//   where:
//       B_k(x) = tanh(B_scale(k) * x + B_offset(k))
//       V_i(x) = x^(V_exponents(i))
//
//   The numerical values of A, B_scale, B_offset, and V_exponents
//   are defined in airfoil-specific data modules.
// =====================================================================================================================
struct CoeffModel {
    Eigen::MatrixXd A;       // [nV x nB]
    Eigen::VectorXd B_scale; // size nB
    Eigen::VectorXd B_offset;
    Eigen::VectorXi V_exponents; // size nV
};

}   // namespace _aerofoil_
}   // namespace _uav_
}   // namespace _acsl_

#endif  // SIM_AEROFOIL_COEFF_BASE_HPP_