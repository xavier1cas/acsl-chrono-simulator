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
 * File:        sim-helpers.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 23, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for the common helper functions that are needed by the simulator
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_HELPERS_HPP_
#define SIM_HELPERS_HPP_

// System includes
#include <cmath>

// Chrono includes
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChRotation.h"

namespace _acsl_
{

// All the transformation functions
namespace _transformations_
{

// Functin to set the position in the Chrono global frame based on the NED positon
chrono::ChVector3d GetChronoPosFromNED(const chrono::ChVector3d& nedpos);

// Function to set the rotation in the Chrono global frame based on the NED rotation
// If no input is given it gives the rotation to go from Chrono Frame to NED frame
chrono::ChQuaternion<> GetChronoOrientaitonFromNED(const chrono::ChQuaternion<>& q_ned = chrono::ChQuaternion<>(1, 0, 0, 0));

// Converts from Chrono's global/world frame to NED position
chrono::ChVector3d GetNEDPosFromChrono(const chrono::ChVector3d& pos_chrono);

// Converts from Chrono's global frame to NED quaternion
// If no input is given it gives the rotation to go from NED frame to Chrono Frame
chrono::ChQuaternion<> GetNEDOrientationFromChrono(const chrono::ChQuaternion<>& q_chrono = chrono::ChQuaternion<>(1, 0, 0, 0));


}   // namespace _transformations_

// All the unit conversions
namespace _conversions_
{

// Converts radians to degrees
double rad2deg(double rad);

}   // namespace _conversions_

// All the computation function
namespace _compute_
{

// Function to evaluate a polynomial given its coefficients and a value at which to evaluate the polynomial
// Example usage: double value = 3.0;
//                Eigen::VectorXd coeffs(3);
//                coeffs << 1, -2, 1; // Represents the polynomial x^2 - 2x + 1
//                double result = evaluatePolynomial(coeffs, value);
double evaluatePolynomial(const Eigen::VectorXd& coefficients, double value);

}   // namespace _compute_

// All the serialization functions
namespace _serialize_
{

// Helper that serializes *any* UAV state struct to CSV-compatible values
template<typename State>
void SerializeStateData(std::ostringstream& oss, const State& s) {
    oss << s.time << ", "
        << s.pos.x() << ", " << s.pos.y() << ", " << s.pos.z() << ", "
        << s.vel.x() << ", " << s.vel.y() << ", " << s.vel.z() << ", "
        << s.acc.x() << ", " << s.acc.y() << ", " << s.acc.z() << ", "
        << s.eul.x() << ", " << s.eul.y() << ", " << s.eul.z() << ", "
        << s.quat.e0() << ", " << s.quat.e1() << ", " << s.quat.e2() << ", " << s.quat.e3() << ", "
        << s.ovel.x() << ", " << s.ovel.y() << ", " << s.ovel.z() << ", "
        << s.oacc.x() << ", " << s.oacc.y() << ", " << s.oacc.z() << ", "
        << s.muI.x() << ", " << s.muI.y() << ", " << s.muI.z() << ", "
        << s.muJ.x() << ", " << s.muJ.y() << ", " << s.muJ.z() << ", "
        << s.tauJ.x() << ", " << s.tauJ.y() << ", " << s.tauJ.z() << ", ";
}


}   // namespace _serialize_

}   // namespace _acsl_

#endif // SIM_HELPERS_HPP_