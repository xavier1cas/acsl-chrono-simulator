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

// ===============================
// System includes
// ===============================
#include <cmath>

// ===============================
// Chrono includes
// ===============================
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChRotation.h"

// ===============================
// RapidCSV include
// ===============================
#include "rapidcsv.h"


// ============================================================================================================
// Helper Function Declaration: _acsl_ helper namespaces
// Define the functions for sim-helpers.hpp --> Do not include any templated functions. 
//                                          --> Avoid data type ambiguity as much as possible. 
//                                          --> Use doubles religiously.
// ============================================================================================================
namespace _acsl_
{

// ============================================================================================================
// Namespace: _transformations_
// Description: Functions for coordinate and orientation conversions 
//              between NED (North-East-Down) and Chrono's global frame
// ============================================================================================================
namespace _transformations_
{

/**
 * @brief Converts NED position vector to Chrono global frame vector.
 * @param nedpos Position in North-East-Down frame (chrono::ChVector3<>)
 * @return Equivalent position in Chrono's global frame (chrono::ChVector3<>)
 */
chrono::ChVector3<> GetChronoPosFromNED(const chrono::ChVector3<>& nedpos);           

/**
 * @brief Converts NED quaternion to Chrono orientation.
 * @param q_ned Quaternion in NED frame (chrono::ChQuaternion<>)
 * @return Corresponding orientation in Chrono global frame (chrono::ChQuaternion<>)
 */
chrono::ChQuaternion<> GetChronoOrientaitonFromNED(const chrono::ChQuaternion<>& q_ned = chrono::ChQuaternion<>(1, 0, 0, 0));

/**
 * @brief Converts position from Chrono global frame to NED position.
 * @param pos_chrono Position in Chrono's global/world frame (chrono::ChVector3<>)
 * @return Equivalent position in the NED frame (chrono::ChVector3<>)
 */
chrono::ChVector3<> GetNEDPosFromChrono(const chrono::ChVector3<>& pos_chrono);   

/**
 * @brief Converts orientation from Chrono global world quaternion to NED quaternion.
 * @param q_chrono Quaternion in Chrono world/global frame (chrono::ChQuaternion<>)
 * @return Corresponding orientation in NED frame (chrono::ChQuaternion<>)
 */
chrono::ChQuaternion<> GetNEDOrientationFromChrono(const chrono::ChQuaternion<>& q_chrono = chrono::ChQuaternion<>(1, 0, 0, 0));

} // namespace _transformations_

// ============================================================================================================
// Namespace: _conversions_
// Description: Utilities for physical unit conversion.
// ============================================================================================================
namespace _conversions_
{

/**
 * @brief Converts radians to degrees.
 * @param rad Angle in radians
 * @return Equivalent angle in degrees
 */
double rad2deg(double rad);

} // namespace _conversions_

// ============================================================================================================
// Namespace: _compute_
// Description: General-purpose computation and math utilities
// ============================================================================================================
namespace _compute_
{

/**
 * @brief Evaluate a polynomial for a given value.
 *        Coefficients should be ordered from highest to lowest degree.
 * @param coefficients Eigen vector of polynomial coefficients
 * @param value Scalar at which to evaluate the polynomial
 * @return Result of polynomial evaluation (double)
 */
double evaluatePolynomial(const Eigen::VectorXd& coefficients, double value);

} // namespace _compute_

// ============================================================================================================
// Namespace: _serialize_
// Description: Serialization helpers for writing data to a file from the simulator
// ============================================================================================================
namespace _serialize_
{


} // namespace _serialize_

// ============================================================================================================
// Namespace: _deserialize_
// Description: Deserialization helpers for reading data from a file into the simulator environment 
// ============================================================================================================
namespace _deserialize_
{

/**
 * @brief Extracts a column of doubles from a RapidCSV document using a header label.
 * @param doc  rapidcsv::Document object
 * @param label String header of column to extract
 * @return std::vector<double> containing the extracted column data
 */
std::vector<double> GetCSVColumn(const rapidcsv::Document& doc, const std::string& label);

} // namespace _deserialize_

} // namespace _acsl_

#endif // SIM_HELPERS_HPP_
