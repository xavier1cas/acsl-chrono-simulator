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


 /**********************************************************************************************************************
 * File:        sim-flat.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        August 11, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Derived environment class for the simulated flat surface.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_FLAT_HPP_
#define SIM_FLAT_HPP_

#include "sim-environment.hpp"

namespace _acsl_
{

namespace _flat_
{

// ============================================================================
// Flat ground dimensions and properties
//
// Purpose:
//   - Define all constant geometric and physical parameters for the simulation
//     floor (the flat environment ground).
//   - These constants are deliberately hard-coded here, as the flat floor is
//     always present and never varied during simulation.
//
// Parameter units:
//   - All dimensions are in meters [m] and density in kg/m^3.
//
// Details:
//   - FLOOR_LENGTH        : Length of the floor plane along X (m).
//   - FLOOR_WIDTH         : Width of the floor plane along Y (m).
//   - FLOOR_HEIGHT        : Thickness of the floor box (m).
//   - FLOOR_DENSITY       : Rigid body density for the floor material (kg/m^3).
//   - FLOOR_VISIBILITY    : If true, the floor is rendered visually.
//   - FLOOR_COLLIDABLE    : If true, the floor participates in collision detection.
//   - FLOOR_PLACEMENT_X   : X-coordinate of floor center in world frame (m).
//   - FLOOR_PLACEMENT_Y   : Y-coordinate of floor center in world frame (m).
//   - FLOOR_PLACEMENT_Z   : Z-coordinate of floor center in world frame (m).
//   - FLOOR_FRICTION_COEFF: Friction coeff [-]
//   - FLOOR_DAMPING_FACTOR: Damping factor [-]
//
// Notes:
//   - These values are only altered by changing the source code; they are
//     NOT loaded from YAML or any external configuration.
// ============================================================================
// constexpr double FLOOR_LENGTH         = 50;
// constexpr double FLOOR_WIDTH          = 50;
constexpr double FLOOR_LENGTH         = 5;
constexpr double FLOOR_WIDTH          = 5;
constexpr double FLOOR_HEIGHT         = 0.1;
constexpr double FLOOR_DENSITY        = 1000;
constexpr bool   FLOOR_VISIBILITY     = true;
constexpr bool   FLOOR_COLLIDABLE     = true;
constexpr double FLOOR_PLACEMENT_X    = 0.0;
constexpr double FLOOR_PLACEMENT_Y    = 0.0;
// constexpr double FLOOR_PLACEMENT_Z    = 0.4;
constexpr double FLOOR_PLACEMENT_Z    = 1.0;
constexpr float  FLOOR_FRICTION_COEFF = 1.0;
constexpr float  FLOOR_DAMPING_FACTOR = 0.5;

// ============================================================================
// simflat class
// Inherits from the templated base environment base class, specifying num = 1,
// meaning this is a fixed 1 body scene of type ChBody.
// ============================================================================
class simflat : public _acsl_::_environment_::simenv<0,1>
{
public:
    // ------------------------------------------------------------------------
    // Constructor for simflat
    //
    // Parameters:
    //   sys - a reference to the Chrono physical system (ChSystemNSC) 
    //         where bodies, joints, and simulation settings live.
    //
    // This constructor *must* forward 'sys' to the base class constructor
    // for simenv<1>, because that class stores a reference to ChSystemNSC.
    // ------------------------------------------------------------------------
    simflat(chrono::ChSystemNSC& sys)
        : _acsl_::_environment_::simenv<0,1>(sys)   // <-- Call the base class constructor
    {
        // All initialization specific to 'simflat' goes here.
        // NOTE: THE ORDER OF THESE FUNCTIONS MATTER. YOU CAN IGNORE THE UNIQUE FNS
        //       IF YOU ARE TRYING TO REPLICATE THIS CODE FOR ANOTHER PLATFORM.    
        // 
        // These are:
        //   - Setting the name of the environment
        //   - Call the function to setup the bodies of the environment
        //   - Initiate the entire locale and add it to the physics system
        
        SetEnvName("flat-ground");                  // <- Baseclass function

        ConfigureFlatGround();                      // <- Compulsary derived class function

        InitiateEnv();                              // <- Baseclass function
    }

    // ------------------------------------------------------------------------
    // You need to add additional functions below that are unique to simflat.
    // These functions include some compulsary functions and some unique to 
    // the platform itself.
    // NOTE: YOU CAN CHANGE THE NAME OF THE COMPULSARY FUNCTIONS TO THE NAME 
    //       OF YOUR PARTICULAR PLATFORM BUT THEY SHOULD BE CLEARLY DEFINED 
    //       (IF YOU ARE TRYING TO REPLICATE THIS CLASS FOR ANOTHER LOCALE)
    // 
    // The comulsary functions are:
    //   - Function to setup the flat ground
    //
    // The unique functions are:
    //   -
    // ------------------------------------------------------------------------
    void ConfigureFlatGround();
};

}   // namespace _flat_

}   // namespace _acsl_

#endif // SIM_FLAT_HPP_