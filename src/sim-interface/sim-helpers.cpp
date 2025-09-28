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
 * File:        sim-helpers.cpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 23, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: cpp file for the common helper functions that are needed by the simulator
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#include "sim-helpers.hpp"

// ============================================================================================================
// Helper Function Implementation: _acsl_ helper namespaces
// Place this block in sim_helpers.cpp --> Do not include any templated functions. 
//                                     --> Avoid data type ambiguity as much as possible. 
//                                     --> Use doubles religiously.
// ============================================================================================================

namespace _shared_
{

// ============================================================================================================
// _transformations_ : Coordinate and orientation conversions between NED and Chrono frames
// ============================================================================================================
namespace _transformations_
{

// Converts vector from NED to Chrono global frame using a 90-degree rotation about X.
chrono::ChVector3<> GetChronoPosFromNED(const chrono::ChVector3<>& nedpos)
{
    // Chrono uses 90° rot about X to match NED <-> ENU
    chrono::ChQuaternion<> RotChABSFrametoNEDFrame = chrono::QuatFromAngleX(chrono::CH_PI_2);
    // Rotate the input vector to global ENU frame
    return RotChABSFrametoNEDFrame.Rotate(nedpos);
}

// Converts quaternion from NED to Chrono global frame.
chrono::ChQuaternion<> GetChronoOrientaitonFromNED(const chrono::ChQuaternion<>& q_ned)
{
    // 90° rotation about X to convert orientation (ENU <-> NED)
    chrono::ChQuaternion<> q_rot = chrono::QuatFromAngleX(chrono::CH_PI_2);
    chrono::ChQuaternion<> q_result;
    q_result.Cross(q_ned, q_rot);  // Quaternion multiplication: q_ned * q_rot
    return q_result;
}

// Converts vector from Chrono global frame to NED frame using inverse rotation.
chrono::ChVector3<> GetNEDPosFromChrono(const chrono::ChVector3<>& pos_chrono)
{
    // -90° rotation about X for inverse conversion
    chrono::ChQuaternion<> RotChABSFrametoNEDFrame = chrono::QuatFromAngleX(-chrono::CH_PI_2);
    return RotChABSFrametoNEDFrame.Rotate(pos_chrono);
}

// Converts quaternion from Chrono global frame to NED orientation.
chrono::ChQuaternion<> GetNEDOrientationFromChrono(const chrono::ChQuaternion<>& q_chrono)
{
    // Inverse 90° rotation about X for ENU->NED conversion
    chrono::ChQuaternion<> q_rot = chrono::QuatFromAngleX(-chrono::CH_PI_2);
    chrono::ChQuaternion<> q_result;
    q_result.Cross(q_chrono, q_rot);  // Quaternion multiplication: q_chrono * q_rot
    return q_result;
}

} // namespace _transformations_


// ============================================================================================================
// _conversions_ : Unit conversion utilities
// ============================================================================================================
namespace _conversions_
{

// Converts radians to degrees, works with any arithmetic double.
double rad2deg(double rad)
{
    // Use Chrono's value for pi for consistency with the simulation environment
    return rad * 180.0 / chrono::CH_PI;
}

} // namespace _conversions_


// ============================================================================================================
// _compute_ : General-purpose computation
// ============================================================================================================
namespace _compute_
{

// Evaluates a polynomial at a given value. Coefficients ordered [highest degree ... lowest].
double evaluatePolynomial(const Eigen::VectorXd& coefficients, double value)
{
    double result = 0.0;
    // Evaluate: result = c0*x^n + c1*x^(n-1) + ... + cn
    for (int i = 0, deg = coefficients.size() - 1; deg >= 0; deg--, i++) {
        result += coefficients[i] * std::pow(value, deg);
    }
    return result;
}

// Creates and returns a 1-dimensional interpolator
NDInterpolator_1_ML* create1DInterpolator(const std::vector<double>& grid, const std::vector<double>& values)
{
    // ---- linterp expects pointers, not references, so we cast away constness ----
    double* grids_begin[1] = { const_cast<double*>(&grid.front()) };
    int grid_len[1] = { static_cast<int>(grid.size()) };

    // ---- Create interpolator over range [begin, end). Use &front() + size() for safety ----
    return new NDInterpolator_1_ML(grids_begin, grid_len,
                                   const_cast<double*>(&values.front()),
                                   const_cast<double*>(&values.front()) + values.size());
}

// Function returns the max value in a vector array
double vecmax(const std::vector<double>& v)
{
    if (v.empty()) {
        ::_acsl_::_message_::SIMULATOR_ERROR("[SIMHLP]: VECMAX: INPUT VECTOR IS EMPTY");
    }
    
    return *std::max_element(v.begin(), v.end());
}

} // namespace _compute_


// ============================================================================================================
// _serialize_ : Serialize data to output from the simulator
// ============================================================================================================
namespace _serialize_
{

// Function takes in any 3 equal length vectors and returns a ChVector3d vector array with the data arranged 
// in the same order as the input.
std::vector<chrono::ChVector3d> serialize2ChVector3d(
    const std::vector<double>& xs,
    const std::vector<double>& ys,
    const std::vector<double>& zs)
{
    size_t N = xs.size();
    std::vector<chrono::ChVector3d> out;
    out.reserve(N);
    for (size_t i = 0; i < N; ++i) {
        out.emplace_back(xs[i], ys[i], zs[i]);
    }
    return out;
}

} // namespace _serialize_


// ============================================================================================================
// _deserialize_ : De-serialize data to read from file
// ============================================================================================================
namespace _deserialize_
{

// Given the column label, it extracts all the data as a vector
std::vector<double> GetCSVColumn(const rapidcsv::Document& doc, const std::string& label)
{
    // Quickly extract a single column from CSV into std::vector<T>
    return doc.GetColumn<double>(label);
}

// Function checks if the given file exists in the path
void FileExists(const std::string& filepath)
{
    if (!std::filesystem::exists(filepath)) {
        ::_acsl_::_message_::SIMULATOR_ERROR("[SIMHLP]: FILE DOES NOT EXIST" + filepath);
    }
}

} // namespace _deserialize_


// ============================================================================================================
// _visualize_ : Helper functions for visualization in the irrlicht environment
// ============================================================================================================
namespace _visualize_
{

// Creates a NURB line for visualization, with conversion from NED to Chrono coordinates
std::shared_ptr<chrono::ChVisualShapeLine> createNurbsVisual(
    std::vector<chrono::ChVector3d>& controlpoints,
    int order,
    const chrono::ChColor& color)
{
    // NOTE: No need to convert each NED point to Chrono's coordinate system as these
    //       assets will be introduced as a visualshape attached to the NED Frame's
    //       auxiliary body which is already pre-rotated.
    
    // Create and initialize the NURBS curve with converted points
    auto nurbs = chrono_types::make_shared<chrono::ChLineNurbs>();
    nurbs->Setup(order, controlpoints);

    // Wrap in visual asset
    auto nurbsasset = chrono_types::make_shared<chrono::ChVisualShapeLine>();
    nurbsasset->SetLineGeometry(nurbs);
    nurbsasset->SetColor(color);

    // Return the visual asset for adding to a body for visualization
    return nurbsasset;
}


} // namespace _visualize_


} // namespace _shared_