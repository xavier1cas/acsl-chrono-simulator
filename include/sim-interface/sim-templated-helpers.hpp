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
 * File:        sim-templated-helpers.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 23, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for the common tempalted helper functions that are needed by the simulator
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_TEMPLATED_HELPERS_HPP_
#define SIM_TEMPLATED_HELPERS_HPP_

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

namespace _acsl_
{

namespace _control_
{

// =========================================================================================================
// RK4 State Array Type Alias
// ------------------------------------------------------------------------------------------
// Purpose:
//   - Defines a state array type for ODE integrators (e.g., RK4).
//   - Uses boost::array for fixed-size storage, template parameterized by scalar type and size.
//
// Usage Example:
//   rk4_array<double, 78> state_vector_double;  // 78-element double precision state vector
//   rk4_array<float, 100> state_vector_float;   // 100-element single precision state vector
//
// Notes:
//   - 'T' is the scalar type (e.g. float, double)
//   - 'N' is the dimensionality (number of states)
//   - Suitable for use as the main state variable in Runge-Kutta (RK4) ODE integration routines
// =========================================================================================================
template<typename T, std::size_t N>
using rk4_array = boost::array<T, N>;

// =========================================================================================================
// RK4 State Vector Type Alias
// ------------------------------------------------------------------------------------------
// Purpose:
//   - Defines a state vector type for ODE integrators (e.g., RK4).
//   - Uses boost::numeric::ublas::vector for dynamic-size storage, template parameterized by scalar type.
//
// Usage Example:
//   rk4_vector<double> state_vector_double;  // double precision state vector (size set at runtime)
//   rk4_vector<float>  state_vector_float;   // single precision state vector (size set at runtime)
//
// Notes:
//   - 'T' is the scalar type (e.g. float, double)
//   - Suitable for use as the main state variable in Runge-Kutta (RK4) ODE integration routines
//   - The vector size is set at runtime and can be changed for flexibility.
//   - Fully compatible with Boost.Odeint integrators and numeric algorithms.
// =========================================================================================================
template<typename T>
using rk4_vector = boost::numeric::ublas::vector<T>;

}   // namespace _control_

}   // namespace _acsl_

namespace _shared_
{

namespace _serialize_
{

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

// Inline template definition for logging Eigen vector or matrix
template <typename Derived>
inline void appendEigenData(std::ostringstream& oss, const Eigen::MatrixBase<Derived>& data) {
    for (int i = 0; i < data.size(); ++i) {
        oss << data(i) << ", ";
    }
}

// Inline template definition for logging the headers for Eigen vector or matrix
template <typename Derived>
inline void generateMatrixHeaders(std::ostringstream& oss, const std::string& matrixName, 
                                            const Eigen::MatrixBase<Derived>& matrix, const std::string& unit) {
    for (int col = 0; col < matrix.cols(); ++col) {             // Iterate over columns first
        for (int row = 0; row < matrix.rows(); ++row) {         // Then iterate over rows
            oss << matrixName << " index-(" << row  << "|" << col << ") " << unit << ", ";
        }
    }
}

} // namespace _serialize_


namespace _deserialize_
{

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

namespace _initiate_
{

/**
 * @brief Zero-initializes a fixed-size Eigen matrix.
 *
 * @tparam MatrixType  Matrix type supporting setZero() (e.g., Eigen::Matrix).
 * @param[in,out] matrix Reference to matrix to be set to zero.
 */
template <typename MatrixType>
void initMat(MatrixType& matrix) {
    matrix.setZero();
}

} // namespace _initiate_

} // namespace _shared_

#endif // SIM_TEMPLATED_HELPERS_HPP_
