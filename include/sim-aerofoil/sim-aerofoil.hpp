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
 * File:        sim-aerofoil.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        December 13, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description:
 *   Generic airfoil base class providing a unified interface for aerodynamic
 *   coefficient evaluation (CL, CD, CM). The class loads airfoil-specific
 *   CoeffModel data based on an enum AirfoilType and exposes a simple api:
 *
 *       computeCL(alpha), computeCD(alpha), computeCM(alpha)
 *
 *   The numerical data for each airfoil is defined in separate headers
 *   (e.g., NACA0012.hpp), which only depend on aero_coeff_model.hpp and not
 *   on this class, thereby avoiding circular dependencies.
 *
 * Usage:
 *   - Construct with AirfoilType::NACA0012 to load that dataset.
 *   - Construct with AirfoilType::None to disable aerodynamic modeling.
 *
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_AIRFOIL_HPP_
#define SIM_AIRFOIL_HPP_

// System includes
#include <cmath>
#include <stdexcept>

// ACSL include
#include "sim-aerofoil-coeff-base.hpp"
#include "sim-messages.hpp"

// =====================================================================================================================
//                                           HOW TO ADD AEROFOILS TO SIMULATOR 
// =====================================================================================================================
// 1. Setup the .hpp file with the data in /include/sim-aerofoil/sim-aerofoils/
// 2. Include the .hpp file below
// 3. Include the name in AirFoilType
// 4. Modify load_from_enum() function in /src/sim-aerofoil/sim-aerofoil.cpp
// 5. You can now load it in and use all the computation functions in simaerofoil class
// =====================================================================================================================

// =====================================================================================================================
// ADD AEROFOILS HEADER FILES HERE THAT LOAD IN THE AEROMODULES FOR THE COEFFICIENTS FOR CL, CD, CM. SEE sim-aerofoils/
// =====================================================================================================================
#include "NACA0012.hpp"

namespace _acsl_
{
namespace _uav_
{
namespace _aerofoil_
{



// =====================================================================================================================
// ADD AEROFOILS ENUM TYPE HERE THAT ARE USED TO LOAD THE RIGHT MODULE IN FOR THE RIGHT AEROFOIL.
// 
// Enum: AirfoilType
//
// Purpose:
//   Enumerates the supported airfoil datasets that can be loaded into simairfoil.
//   The 'NONE' value disables airfoil-based aerodynamic modeling and causes the
//   coefficient evaluation functions to return zero.
// =====================================================================================================================
enum class AirFoilType {
  NONE,       // NO AIRFOIL DATA PRESENT
  NACA0012    
  // ADD FUTURE ENTIRES HERE: NACA<XXXX> ETC.
};

// =====================================================================================================================
// Class: simairfoil
//
// Purpose:
//   Base airfoil class used by the simulator to encapsulate aerodynamic
//   coefficient models for lift (CL), drag (CD), and pitching moment (CM).
//
//   This class:
//     - Stores three CoeffModel instances (CL, CD, CM).
//     - Provides evaluation methods computeCL/CD/CM(alpha).
//     - Loads airfoil-specific data based on AirfoilType via load_from_enum.
// =====================================================================================================================
class simairfoil
{
  public:
    // -----------------------------------------------------------------------------------------------------------------
    // Constructor: simairfoil(type)
    //
    // Purpose:
    //   Construct a simairfoil instance and load the corresponding airfoil
    //   coefficient models based on the given AirFoilType enum.
    //
    // Parameters:
    //   type - Desired airfoil dataset (None, NACA0012, etc.).
    // -----------------------------------------------------------------------------------------------------------------
    explicit simairfoil(AirFoilType type);

    // Evaluate CL(alpha); returns 0.0 if no airfoil is loaded.
    double ComputeCL(double alpha);

    // Evaluate CD(alpha); returns 0.0 if no airfoil is loaded.
    double ComputeCD(double alpha);

    // Evaluate CM(alpha); returns 0.0 if no airfoil is loaded.
    double ComputeCM(double alpha);

    // Getter function for the airfoil load boolean
    bool HasAeroFoil() { return this->has_airfoil_; }

  protected:
    // Protected setters for coefficient models (used by loaders, if needed).
    void SetCLModel(const CoeffModel& m) { cl_model_ = m; }
    void SetCDModel(const CoeffModel& m) { cd_model_ = m; }
    void SetCMModel(const CoeffModel& m) { cm_model_ = m; }

  private:
    CoeffModel cl_model_;
    CoeffModel cd_model_;
    CoeffModel cm_model_;

    // Setter function for the airfoil load boolean
    void SetAeroFoilPresent() { this->has_airfoil_ = true; }

    // Setter function for the airfoil load boolean
    void SetAeroFoilAbsent() { this->has_airfoil_ = false; }

    // Flag indicating whether an airfoil has been loaded (false for AirfoilType::None).
    bool has_airfoil_ = false;

    // Generic evaluation of y(x) = (A * B(x))^T * V(x) for a given model.
    static double eval(const CoeffModel& model, double x);

    // Load CL/CD/CM models from the requested AirfoilType.
    void load_from_enum(AirFoilType type);
};


}   // namespace _aerofoil_
}   // namespace _uav_
}   // namespace _acsl_

#endif  // SIM_AIRFOIL_HPP_

