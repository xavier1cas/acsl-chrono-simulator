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

namespace _compute_
{

/**
 * @brief Evaluates a polynomial at a given input value.
 *
 * @tparam VectorType Type of vector containing polynomial coefficients (e.g., Eigen::VectorXd).
 * @tparam ScalarType Type of input/output scalar (e.g., double, float).
 *
 * @param coefficients Vector of polynomial coefficients, ordered [highest degree ... lowest degree].
 * @param value Value at which to evaluate the polynomial.
 *
 * @return ScalarType The result of the polynomial evaluated at the input value.
 *
 * @details
 *   Evaluates the polynomial defined by the given coefficients at the specified value.
 *   The coefficient vector should be ordered from highest degree term to lowest.
 *   For example, for coefficients [a, b, c], the polynomial is \( a x^2 + b x + c \).
 *
 *   Uses Horner's rule, summing each term: result = c0 * x^n + c1 * x^{n-1} + ... + cn.
 *
 * @note Example usage:
 *   Eigen::VectorXd coeffs(3); coeffs << 1, -2, 1; // x^2 - 2x + 1
 *   double y = evaluatePolynomial(coeffs, 3.0);   // Evaluates to 4.0
 */
template <typename VectorType, typename ScalarType>
inline ScalarType evaluatePolynomial(const VectorType coefficients, ScalarType value) {
    ScalarType result = 0;
    for (int i = 0, deg = coefficients.size() - 1; deg >= 0; deg--, i++)
    {
        result += coefficients[i] * std::pow(value, deg);
    }
    return result;
}

/**
 * @brief Computes the 2D Euclidean norm of polynomial trajectories in the x and y dimensions.
 *
 * @tparam VectorType Type of vector containing polynomial coefficients (e.g., Eigen::VectorXd).
 * @tparam ScalarType Type of the scalar value (e.g., double, float).
 *
 * @param poly_coef_x Vector of coefficients for the x-dimension polynomial.
 * @param poly_coef_y Vector of coefficients for the y-dimension polynomial.
 * @param value Scalar value at which to evaluate the norm (typically a normalized or time-scaled parameter).
 *
 * @return ScalarType The computed Euclidean norm at the given evaluation point.
 *
 * @details
 *   This function evaluates two polynomials, one representing the x-dimension and one for the y-dimension,
 *   at a common input value, then computes their 2D Euclidean norm:
 *   @f[
 *       \text{norm}(t) = \sqrt{ (x(t))^2 + (y(t))^2 }
 *   @f]
 *
 *   The polynomials are evaluated using the `evaluatePolynomial()` helper, which assumes coefficient order
 *   from highest to lowest degree. For a coefficient vector
 *   `[a, b, c]`, the polynomial is \( a x^2 + b x + c \).
 *
 * @note Example usage:
 *   @code
 *   Eigen::VectorXd poly_coef_x(3), poly_coef_y(3);
 *   poly_coef_x << 1, -2, 1;  // Example: x(t) = t^2 - 2t + 1
 *   poly_coef_y << 2, 0, -1;  // Example: y(t) = 2t^2 - 1
 *   double t = 1.0;
 *   double result = norm2D<Eigen::VectorXd, double>(poly_coef_x, poly_coef_y, t);
 *   // result = sqrt( (x(t))^2 + (y(t))^2 )
 *   @endcode
 *
 * @see evaluatePolynomial
 */
template<typename VectorType, typename ScalarType>
inline ScalarType norm2D(const VectorType& poly_coef_x, const VectorType& poly_coef_y, ScalarType value)
{
    ScalarType norm_value = std::sqrt(
        std::pow(evaluatePolynomial(poly_coef_x, value), 2) +
        std::pow(evaluatePolynomial(poly_coef_y, value), 2)
    );
    return norm_value;
}

/**
 * @brief Computes the time-derivative of the 2D Euclidean norm of polynomial trajectories.
 *
 * @tparam VectorType Type of vector containing polynomial coefficients (e.g., Eigen::VectorXd).
 * @tparam ScalarType Type for the evaluation variable and resulting value (e.g., double).
 *
 * @param poly_coef_x Vector of coefficients for the x-dimension polynomial.
 * @param poly_coef_y Vector of coefficients for the y-dimension polynomial.
 * @param poly_coef_x_prime Vector of coefficients for the derivative of the x polynomial.
 * @param poly_coef_y_prime Vector of coefficients for the derivative of the y polynomial.
 * @param value Scalar value at which to evaluate the norm and its derivative.
 *
 * @return ScalarType Derivative of the 2D norm evaluated at the given value.
 *
 * @details
 *   This function computes the time-derivative of the 2D norm at a point, for polynomials
 *   representing x and y as functions of a scalar variable (e.g., time). It uses the closed-form:
 *   @f[
 *     \frac{d}{dt} \left( \sqrt{x(t)^2 + y(t)^2} \right)
 *     = \frac{x(t)x'(t) + y(t)y'(t)}{\sqrt{x(t)^2 + y(t)^2}}
 *   @f]
 *   Both the polynomials and their derivatives are evaluated at the specified value using
 *   `evaluatePolynomial()`. The result is the norm's instantaneous rate of change at that point.
 *
 * @note Example usage:
 *   @code
 *   Eigen::VectorXd poly_coef_x(3), poly_coef_y(3), poly_coef_x_prime(3), poly_coef_y_prime(3);
 *   poly_coef_x << 1, -2, 1;         // x(t) = t^2 - 2t + 1
 *   poly_coef_y << 2, 0, -1;         // y(t) = 2t^2 - 1
 *   poly_coef_x_prime << 2, -2, 0;   // x'(t) = 2t - 2
 *   poly_coef_y_prime << 4, 0, 0;    // y'(t) = 4t
 *   double t = 1.0;
 *   double d_norm = norm2Dderivative<Eigen::VectorXd, double>(
 *       poly_coef_x, poly_coef_y, poly_coef_x_prime, poly_coef_y_prime, t);
 *   @endcode
 *
 * @see norm2D, evaluatePolynomial
 */
template<typename VectorType, typename ScalarType>
inline ScalarType norm2Dderivative(const VectorType& poly_coef_x,
                                   const VectorType& poly_coef_y,
                                   const VectorType& poly_coef_x_prime,
                                   const VectorType& poly_coef_y_prime,
                                   ScalarType value)
{
    ScalarType norm_value = norm2D<VectorType, ScalarType>(poly_coef_x, poly_coef_y, value);
    ScalarType derivative_value = (
        evaluatePolynomial<VectorType, ScalarType>(poly_coef_x, value) *
        evaluatePolynomial<VectorType, ScalarType>(poly_coef_x_prime, value) +
        evaluatePolynomial<VectorType, ScalarType>(poly_coef_y, value) *
        evaluatePolynomial<VectorType, ScalarType>(poly_coef_y_prime, value)
    ) / norm_value;
    return derivative_value;
}

/**
 * @brief Compute the derivative of a piecewise polynomial coefficient matrix row-wise.
 *
 * Given a matrix whose rows represent polynomial coefficients for each piece of a trajectory,
 * returns a matrix whose rows are the coefficients of the derivatives of those polynomials.
 *
 * @tparam MatrixType Eigen-like matrix type (e.g., Eigen::MatrixXd)
 * @param poly_coeff_matrix_in Input matrix; each row is polynomial coefficients in descending exponent order.
 * @return MatrixType Output matrix; each row contains the derivative polynomial coefficients (one order lower).
 *
 * @details
 *   The input matrix is assumed to have each row as a set of polynomial coefficients (highest degree first).
 *   For a row like [a b c d] representing \( a x^3 + b x^2 + c x + d \), the result will be
 *   [3a 2b c], representing the derivative \( 3a x^2 + 2b x + c \).
 *
 * @see For example usage and mathematical background, see Eigen documentation.
 */
template<typename MatrixType>
inline MatrixType polyDerMatrix(const MatrixType& poly_coeff_matrix_in)
{
    // Get the size of the input matrix
    int rows = poly_coeff_matrix_in.rows();
    int cols = poly_coeff_matrix_in.cols();

    // Initialize the output matrix for the derivative
    MatrixType poly_coeff_matrix_out(rows, cols - 1);

    // Compute the derivative for each row of the input matrix
    for (int i = 0; i < rows; ++i) {
        for (int j = 0; j < cols - 1; ++j) {
            poly_coeff_matrix_out(i, j) = (cols - 1 - j) * poly_coeff_matrix_in(i, j);
        }
    }

    return poly_coeff_matrix_out;
}


}   // namespace _compute_

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

namespace _validate_
{
    
/**
 * @brief Checks if a scalar value is NaN.
 *
 * Throws a runtime error with the provided message if the input value is NaN.
 *
 * @tparam T        Scalar type (e.g., float, double).
 * @param value     The value to check for NaN.
 * @param message   The error message to include in the exception if value is NaN.
 * @throws std::runtime_error if value is NaN.
 */
template <typename T>
inline void VariableCheckNaN(T value, const char* message) {
    if (std::isnan(value)) {
        throw std::runtime_error(message);
    }
}


/**
 * @brief Checks if all elements in an Eigen matrix are finite (not NaN or inf).
 *
 * Throws a runtime error with the provided message if the matrix contains non-finite entries.
 *
 * @tparam Derived  Eigen matrix or vector type (supports allFinite()).
 * @param matrix    The Eigen matrix or vector to check.
 * @param message   The error message to include in the exception if check fails.
 * @throws std::runtime_error if matrix contains any NaN or Inf elements.
 */
template <typename Derived>
inline void MatrixCheckNaN(const Eigen::MatrixBase<Derived>& matrix, const char* message) {
    if (!matrix.allFinite()) {
        throw std::runtime_error(message);
    }
}


} // namespace _validate_

} // namespace _shared_

#endif // SIM_TEMPLATED_HELPERS_HPP_
