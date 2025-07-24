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
 * File:        sim-uav.hpp
 * Author:      Giri Mugundan Kumar
 * Date:        July 15, 2025
 * For info:    Andrea L'Afflitto 
 *              a.lafflitto@vt.edu
 * 
 * Description: Header file for the uav baseclass
 * 
 * GitHub:    https://github.com/girimugundankumar/acsl-physics-sim.git
 **********************************************************************************************************************/

#ifndef SIM_UAV_HPP_
#define SIM_UAV_HPP_

// System inclues
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <array>


#include "sim-messages.hpp"
#include "fkYAML/node.hpp"
#include "chrono/physics/ChSystemNSC.h"

namespace _acsl_
{
namespace _uav_
{

// Templated function that takes in the number of propellers
template <int nop>
class simuav
{

public:
    // Constructor: Constructor that takes in the system and the name of the uav
    simuav(chrono::ChSystemNSC& sys, const std::string name) : m_physics_(sys), name_(name) {}

    // Getter function for the name of the UAV
    std::string GetUAVPlatformName() { return name_; }

    // Getter function for retreving the path to the shapes directory
    std::string GetUAVShapesDir() { return shapes_dir; }

protected:
    // Getter function for getting the number of propellers
    int GetPropCount() { return nop; }
    
    // Setter function for setting the path to the shapes directory
    void SetUAVShapesDir(std::string dir) { shapes_dir = dir; }   

    // Getter function for getting the chassis of the UAV
    std::shared_ptr<chrono::ChBodyAuxRef> GetUAVChassis() { return chassis; }

    // Getter function for returning the bodylist
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> GetUAVBodyList() { return bodylist; }
    
    // Getter function for returning the linklist
    std::vector<std::shared_ptr<chrono::ChLinkBase>> GetUAVLinkList() { return linklist; }

    // Function to setup the Inertial NED frame
    void SetupInertialNEDFrame();

private:
    // Pointer to the system object
    chrono::ChSystemNSC& m_physics_;

    // Name of the uav
    std::string name_;

    // String for the path to the .obj files
    std::string shapes_dir;

    // ChBodyAuxRef pointer to store the inertial frame
    std::shared_ptr<chrono::ChBodyAuxRef> InertialFrameNED;

    // ChBodyAuxRef pointer to store the chassis
    std::shared_ptr<chrono::ChBodyAuxRef> chassis;

    // ChBodyAuxRef Fixed-size array of pointers for props
    std::array<std::shared_ptr<chrono::ChBodyAuxRef>, nop> props;

    // UAV object bodylist
    std::vector<std::shared_ptr<chrono::ChBodyAuxRef>> bodylist;
    std::vector<std::shared_ptr<chrono::ChLinkBase>> linklist;

};

}   // namespace _uav_

}   // namespace _acsl_

#endif  // SIM_UAV_HPP_