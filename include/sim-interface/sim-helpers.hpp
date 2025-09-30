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
#include <vector>
#include <filesystem>
#include <string>
#include <stdexcept>

// ===============================
// Chrono includes
// ===============================
#include "chrono/core/ChQuaternion.h"
#include "chrono/core/ChVector3.h"
#include "chrono/core/ChRotation.h"
#include "chrono/geometry/ChLineNurbs.h"
#include "chrono/geometry/ChSurfaceNurbs.h"
#include "chrono/assets/ChVisualShapeLine.h"

// ===============================
// RapidCSV includes
// ===============================
#include "rapidcsv.h"

// ===============================
// JSON includes
// ===============================
#include <nlohmann/json.hpp>            

// ===============================
// Eigen includes
// ===============================
#include <Eigen/Dense>  

// ===============================
// Linterp inlcudes
// ===============================
#include "linterp.h"

// ===============================
// ACSL inlcudes
// ===============================
#include "sim-messages.hpp"
#include "sim-control-base.hpp"


// ============================================================================================================
// Helper Function Declaration: _acsl_ helper namespaces
// Define the functions for sim-helpers.hpp --> Do not include any templated functions. 
//                                          --> Avoid data type ambiguity as much as possible. 
//                                          --> Use doubles religiously.
// ============================================================================================================
namespace _shared_
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

/**
 * @brief Computes the 3-2-1 rotation matrix that transforms a vector from global to local coordinates.
 * 
 * The returned matrix is: R1ᵀ * R2ᵀ * R3ᵀ (roll, pitch, yaw).
 * 
 * @param roll  Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @param yaw   Yaw angle in radians.
 * @return Eigen::Matrix3d The global-to-local rotation matrix.
 */
Eigen::Matrix3d rotationMatrix321GlobalToLocal(double roll, double pitch, double yaw);

/**
 * @brief Computes the 3-2-1 rotation matrix that transforms a vector from local to global coordinates.
 * 
 * The returned matrix is: R3 * R2 * R1 (yaw, pitch, roll).
 * 
 * @param roll  Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @param yaw   Yaw angle in radians.
 * @return Eigen::Matrix3d The local-to-global rotation matrix.
 */
Eigen::Matrix3d rotationMatrix321LocalToGlobal(double roll, double pitch, double yaw);

/**
 * @brief Returns a 3x3 Jacobian matrix inverse for given roll and pitch angles.
 *
 * Exception is thrown if the pitch angle results in a singular configuration.
 * 
 * @param roll  Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @return Eigen::Matrix3d Inverse of the Euler angle rates Jacobian.
 * @throws std::invalid_argument if pitch near ±pi/2.
 */
Eigen::Matrix3d jacobianMatrixInverse(double roll, double pitch);

/**
 * @brief Computes the 3x3 Euler angle rates Jacobian matrix for given roll and pitch.
 * 
 * @param roll  Roll angle in radians.
 * @param pitch Pitch angle in radians.
 * @return Eigen::Matrix3d Euler angle rates Jacobian.
 */
Eigen::Matrix3d jacobianMatrix(double roll, double pitch);

/**
 * @brief Computes the time derivative of the Jacobian matrix for given roll, pitch, and their time derivatives.
 * 
 * @param roll      Roll angle in radians.
 * @param pitch     Pitch angle in radians.
 * @param roll_dot  Time derivative of roll (rad/s).
 * @param pitch_dot Time derivative of pitch (rad/s).
 * @return Eigen::Matrix3d Time derivative of the Jacobian matrix.
 */
Eigen::Matrix3d jacobianMatrixDerivative(
    double roll, double pitch, double roll_dot, double pitch_dot
);

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

/**
 * @brief Factory function for 1D linear interpolators. Given a grid (e.g., time) and its value
 *        vector (e.g., position), returns a fully constructed NDInterpolator_1_ML object ready
 *        for queries.
 * @param grid vector of double, the grid points (must be monotonically increasing)
 * @param values vector of double, values at each grid point (same size as grid)
 * @return NDInterpolator_1_ML object pointer configured for interpolation over the provided data.
 * @note Both grid and value vectors must remain valid for the lifetime of returned interpolator.
 *       Use with care if passing temporary/local vectors.       
 */
NDInterpolator_1_ML* create1DInterpolator(const std::vector<double>& grid, const std::vector<double>& values);

/**
 * @brief Finds the maximum value in a vector of doubles.
 * @param v Input vector of doubles.
 * @return Maximum value in the vector.
 * @note Behavior is undefined for empty vectors; add a check as needed.
 */
double vecmax(const std::vector<double>& v);

} // namespace _compute_

// ============================================================================================================
// Namespace: _serialize_
// Description: Serialization helpers for writing data to a file from the simulator
// ============================================================================================================
namespace _serialize_
{

/**
 * @brief Combines three parallel double vectors into a vector of ChVector3d for 3D point serialization.
 * @param xs Vector of X coordinates.
 * @param ys Vector of Y coordinates.
 * @param zs Vector of Z coordinates.
 * @return std::vector<chrono::ChVector3d> Combined 3D points.
 */
std::vector<chrono::ChVector3d> serialize2ChVector3d(
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    const std::vector<double>& zs);

/**
 * @brief Copies elements from an Eigen matrix/vector to a segment in a boost array.
 *
 * Assigns matrix/vector data from the Eigen object into the target boost::array starting at current_index.
 * Typically used for serializing state data before integration or output.
 *
 * @tparam T         Scalar type in the Eigen matrix/vector and boost array (e.g. double, float).
 * @tparam Rows      Number of rows in the Eigen matrix/vector.
 * @tparam Cols      Number of columns in the Eigen matrix/vector.
 * @tparam N         Total number of elements in the boost array.
 * @param entity         Source Eigen matrix or vector to copy from.
 * @param dxdt           Destination boost::array to receive the data.
 * @param current_index  Starting index for assignment into dxdt (incremented by number of elements copied).
 */
template<typename T, int Rows, int Cols, std::size_t N>
inline void assignElementsToDxdt(Eigen::Matrix<T, Rows, Cols>& entity, boost::array<T, N>& dxdt,
                                 int& current_index)
{
    std::size_t elements_to_copy = std::min<std::size_t>(entity.size(), dxdt.size() - current_index);
    std::copy(entity.data(), entity.data() + elements_to_copy, dxdt.begin() + current_index);
    current_index += elements_to_copy;
}


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

/**
 * @brief Checks that the file exists, throws an error if not.
 * @param filepath Path or filename to check.
 * @throws std::runtime_error if the file does not exist.
 */
void FileExists(const std::string& filepath);

/**
 * @brief Extract Eigen::MatrixXd from JSON array of arrays
 * @param jsonMatrix Matrix input that is in the json format
 * @param rows no of rows in the array
 * @param cols no of cols in the array
 */
Eigen::MatrixXd jsonToMatrixXd(const nlohmann::json& jsonMatrix, int rows, int cols);

/**
 * @brief Extract Eigen::MatrixXd from JSON with scaling
 * @param jsonMatrix Matrix input that is in the json format
 * @param rows no of rows in the array
 * @param cols no of cols in the array
 */
Eigen::MatrixXd jsonToScaledMatrixXd(const nlohmann::json& jsonMatrix, int rows, int cols);

/**
 * @brief Assigns elements from a boost array segment to an Eigen matrix or vector.
 *
 * Maps the relevant segment of the boost array starting at current_index into the target Eigen matrix/vector.
 * This is typically used for deserialization after integration or bulk loading states.
 *
 * @tparam T         Scalar type for Eigen matrix/vector and boost array (usually double or float).
 * @tparam Rows      Number of rows in target Eigen matrix/vector (static size).
 * @tparam Cols      Number of columns in target Eigen matrix/vector (static size).
 * @tparam N         Total number of elements in the boost array.
 * @param entity     Target Eigen matrix or vector for assignment (e.g. Eigen::Vector2d).
 * @param x          Source boost::array containing deserialized data.
 * @param current_index Starting index in the boost array where assignment should begin; incremented by matrix size.
 */
template<typename T, int Rows, int Cols, std::size_t N>
inline void assignElementsToMembers(Eigen::Matrix<T, Rows, Cols>& entity, boost::array<T, N>& x,
                                    int& current_index)
{
    // Map the segment of the boost array to the Eigen matrix
    Eigen::Map<Eigen::Matrix<T, Rows, Cols>>(entity.data(), Rows, Cols) = 
        Eigen::Map<Eigen::Matrix<T, Rows, Cols>>(x.data() + current_index, Rows, Cols);

    // Update the current index by the number of elements in the matrix
    current_index += Rows * Cols;
}

} // namespace _deserialize_

// ============================================================================================================
// Namespace: _visualize_
// Description: Helper functions for visualization in the irrlicht environment
// ============================================================================================================
namespace _visualize_
{

/**
 * @brief Creates a NURBS (Non-Uniform Rational B-Spline) visual line asset from a set of 3D control points.
 * 
 * This function generates a NURBS curve using the supplied control points and polynomial order,
 * encapsulates it as a Project Chrono ChVisualShapeLine asset, and sets the display color.
 *
 * @param controlpoints Vector of ChVector3d points defining the NURBS control polygon.
 * @param order         Degree of the NURBS curve (default: cubic, 3; must satisfy order < controlpoints.size()).
 * @param color         Color of the rendered line (default: black RGB).
 * @return std::shared_ptr<chrono::ChVisualShapeLine> Smart pointer to the created NURBS visual asset.
 */
std::shared_ptr<chrono::ChVisualShapeLine> createNurbsVisual(
    std::vector<chrono::ChVector3d>& controlpoints,
    int order = 3,
    const chrono::ChColor& color = chrono::ChColor(0.0f, 0.0f, 0.0f));
    
} // namespace _visualize_

} // namespace _shared_

#endif // SIM_HELPERS_HPP_
