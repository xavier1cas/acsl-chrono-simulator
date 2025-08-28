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
 * File:        sim-locales.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        June 13, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header only file to include all the locales.
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_LOCALES_HPP_
#define SIM_LOCALES_HPP_

#include "sim-environment.tpp"

/* =====================================================================================================================
  sim-locales.hpp

  Purpose:
    - Central registry for all environment platform class includes.
    - This ensures that `sim-bridge.hpp` (and related simulation code) 
      can discover and use all available UAV platform definitions.

  How to add a new platform:
    1. Include your derived environment class header in the "Includes" section below.
    2. Add a new `bool <locale_name>{};` member to the `locales` struct.
       !! The member name **must exactly match** the name used in your YAML config. !!
    3. Update the `members()` helper inside the struct with the new entry  
       so that `asMap()` / `asVector()` can iterate it automatically.
    4. Update `sim-bridge.hpp` and `sim-bridge.cpp` if any locale‑specific
       logic is needed for initialization.
    5. Verify that the locale is listed in the YAML configuration file and  
       that it exists in the ACSL Physics Sim database with matching `.hpp`/`.cpp`
       implementation.

  Notes:
    - Keep includes grouped and sorted alphabetically for maintainability.
    - This file intentionally only contains header includes and the **locales** 
      struct — no functional logic belongs here.
==================================================================================================================== */

// -----------------------------------------------------------------------------
// Includes for different locales/scenes (add more as needed)
// -----------------------------------------------------------------------------
#include "sim-flat.hpp"
// #include "sim-mountain.hpp"
// #include "sim-arena.hpp"

namespace _acsl_
{

namespace _bridge_
{

// -----------------------------------------------------------------------------
// Structure: locales
//
// Holds boolean flags indicating which environment locales (scenes/platforms) are enabled (true)
// or disabled (false) for the current simulation run.
//
// How to add/remove a locale: see file-level doc block above.
// -----------------------------------------------------------------------------
struct locales {
    bool flat{};        // Flat surface
    // bool mountain{};  // Example: Mountain terrain
    // bool arena{};     // Example: Urban test arena

    // -------------------------------------------------------------------------
    // Helper: asVectorRef()
    //
    // Purpose:
    //   Returns all locale members as {name, reference} pairs for dynamic
    //   iteration (e.g., to set or check which locale(s) are active).
    //   - Use `auto&` for writable access.
    //   - Use `const auto&` for read-only access.
    //
    // Example usage:
    //   for (auto& [name, val] : locales.asVectorRef()) { val = true; }
    //   for (const auto& [name, val] : locales.asVectorRef()) { if (val) {...} }
    // -------------------------------------------------------------------------
    std::vector<std::pair<std::string, bool&>> asVectorRef() {
        return {
            {"flat", flat}
            // ,{"mountain", mountain}
            // ,{"arena", arena}
        };
    }

    // -------------------------------------------------------------------------
    // Factory: createSelectedLocale()
    //
    // Purpose:
    //   Instantiate the environment locale object corresponding to the active locale
    //   and return it as a polymorphic unique_ptr<simenvbase>.
    //
    // Parameters:
    //   sys - Reference to the Chrono physics system (ChSystemNSC) to be passed
    //         into the environment constructor.
    //
    // Returns:
    //   std::unique_ptr<_acsl_::_environment_::simenvbase> pointing to the constructed
    //   locale (scene) instance.
    //
    // Workflow:
    //   1. Validate that exactly one locale is selected (see validateExclusiveSelection()).
    //   2. Based on which boolean is true, construct the matching locale class.
    //   3. Return the base-class pointer (simenvbase) to the caller.
    //
    // Notes:
    //   - Requires that the derived locale class headers are included above.
    //   - Throws/logs an error if zero or more than one locales are selected.
    // -------------------------------------------------------------------------
    std::unique_ptr< ::_acsl_::_environment_::simenvbase >
    createSelectedLocale(chrono::ChSystemNSC& sys) const {
        std::string active = validateExclusiveSelection();
        if (active == "flat") {
            _message_::SIMULATOR_INFO("[SIMBRG]: ATTACHING FLAT SURFACE LOCALE TO THE PHYSICS SYSTEM");
            return std::unique_ptr< ::_acsl_::_environment_::simenvbase >(new ::_acsl_::_flat_::simflat(sys));
        }
        // else if (active == "mountain") {
        //     *message*::SIMULATOR_INFO("ATTACHING MOUNTAIN LOCALE TO THE PHYSICS SYSTEM");
        //     return std::unique_ptr< ::_acsl_::_environment_::simenvbase >(new ::_acsl_::_mountain_::simmountain(sys));
        // }
        // Add more else-if blocks for new scenes/locales here
        _message_::SIMULATOR_ERROR("[SIMBRG]: ACTIVE LOCALE NAME NOT RECOGNIZED: " + active);
        return nullptr;
    }

    // -------------------------------------------------------------------------
    // Helper: validateExclusiveSelection()
    //
    // Purpose:
    //   Ensures that exactly ONE locale is set to true, and returns its name.
    //   Throws or logs a SIMULATOR_ERROR if zero or more than one are enabled.
    //
    // Returns:
    //   Name (string) of the active locale.
    // -------------------------------------------------------------------------
    std::string validateExclusiveSelection() const {
        std::string active;
        int count = 0;
        for (const auto& [name, val] : const_cast<locales*>(this)->asVectorRef()) {
            if (val) {
                active = name;
                count++;
            }
        }
        if (count == 0) {
            _message_::SIMULATOR_ERROR("[SIMBRG]: NO ENVIRONMENT SCENE SELECTED — EXACTLY ONE MUST BE SET TO TRUE.");
        } else if (count > 1) {
            _message_::SIMULATOR_ERROR("[SIMBRG]: MULTIPLE ENVIRONMENT SCENES SELECTED — EXACTLY ONE MUST BE SET TO TRUE.");
        }
        return active;
    }
};

}   // namespace _bridge_

}   // namespace _acsl_

#endif // SIM_LOCALES_HPP_