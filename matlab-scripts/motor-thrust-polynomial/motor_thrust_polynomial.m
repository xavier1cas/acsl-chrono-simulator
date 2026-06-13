% =========================================================================
% Copyright (c) 2024 Giri M. Kumar, Mattia Gramuglia, Andrea L'Afflitto.
% All rights reserved.
% 
% Redistribution and use in source and binary forms, with or without 
% modification, are permitted provided that the following conditions 
% are met:
% 
% 1. Redistributions of source code must retain the above copyright 
% notice, this list of conditions and the following disclaimer.
% 
% 2. Redistributions in binary form must reproduce the above copyright 
% notice, this list of conditions and the following disclaimer in 
% the documentation and/or other materials provided with the 
% distribution.
% 
% 3. Neither the name of the copyright holder nor the names of its 
% contributors may be used to endorse or promote products derived 
% from this software without specific prior written permission.
%  
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
% "AS IS” AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
% FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
% COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
% INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
% BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
% LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
% CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT 
% LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
% ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
% POSSIBILITY OF SUCH DAMAGE.
% =========================================================================

% =========================================================================
% File:        motor_thrust_polynomial.m
% Author:      Giri Mugundan Kumar
% Date:        November 29, 2024
% For info:    Andrea L'Afflitto 
%              a.lafflitto@vt.edu
% 
% Description: Data processing code for the motors in the simulation and 
%              real world testing. Example script can be used to process 
%              any motor for the simulator as well as the flightstack.
% 
% GitHub:      https://github.com/girimugundankumar/acsl-physics-sim.git
% =========================================================================

%==========================================================================
clear all
close all
clc

% Some plotter settings
set(groot, 'defaultAxesTickLabelInterpreter','latex');
set(groot, 'defaultLegendInterpreter','latex');
parameters.font_size = 20;

% Add the folder paths
addpath("data/")
addpath("functions/")
%==========================================================================

%% Thrust Calibration Data Requirements and Test Procedure
%
% ---> NOTE <---
% The power levels 0.05 to 1.0 used below correspond to the normalized
% thrust input for the Pixhawk (throttle command in the range [0,1]).
% In this convention, 0.05–1.0 normalized thrust spans approximately
% 5–100% of the available throttle range for the motors.
%
% This script assumes that motor thrust calibration data are stored in the
% data/ directory as Excel workbooks (.xlsx), one workbook per motor.
% To add a new motor:
%   - Create a new .xlsx file in data/ following the format of the
%     existing example files.
%   - Use exactly four sheets per motor, corresponding to four
%     calibration runs.
%
% Each motor requires FOUR calibration runs. For each run, start with a
% freshly charged battery. The four runs may be performed back-to-back
% or separated in time, but every run must begin from a full charge to
% ensure consistent operating conditions.
%
%   • Run 1 (ascending):
%       - Start from 0.05 normalized thrust and increase to 1.0.
%       - Use increments of 0.05 in the normalized thrust command
%         (equivalently, from ~0 to 100% throttle).
%
%   • Run 2 (descending):
%       - Start from 1.0 normalized thrust and decrease to 0.05.
%       - Use decrements of 0.05 in the normalized thrust command.
%
%   • Run 3 (repeat ascending):
%       - Repeat the same procedure as Run 1 (0.05 → 1.0 in steps of 0.05).
%
%   • Run 4 (repeat descending):
%       - Repeat the same procedure as Run 2 (1.0 → 0.05 in steps of 0.05).
%
% If only a single run is available for initial testing, you may duplicate
% it into four sheets as a placeholder, but proper calibration should be
% based on four physically distinct runs as described above.

%% ===================================================== SET THE PARAMETERS
%-------------------------------------------------------------------------
% READ THE EXCEL SHEET - UNCOMMENT THE MOTOR YOU WANT FROM THE DATA FOLDER
%-------------------------------------------------------------------------
% Velox V2808 1300Kv Motor | DP 7x4x3 Triblade
% F = "velox_v2808_1300kv";
% parameters.D = 0.1778;      % Diameter of the propeller

% Antigravity MN5008 400Kv | T-motor 16x5.4" CF
% F = "antigravity_mn5008_400kv";
% parameters.D = 0.4064;      % Diameter of the propeller

% Tmotor P2306 V3 2550Kv Motor | Tmotor P49436-3
F = "p2306_v3_2550Kv";
parameters.D = 0.12446;      % Diameter of the propeller

%-------------------------------------------------------------------------
% SET THE PARAMETERS FOR PROCESSING - Usually need not be modified
%-------------------------------------------------------------------------
parameters.order_thr = 7;      % Order to process the polynomial for thrust
parameters.order_rps = 7;      % Order to process the polynomial for RPMs
parameters.rho = 1.225;        % The air density [Kg/m^3]

%% ======================================================= PROCESS THE DATA
%-------------------------------------------------------------------------
% USE THE PROCESS FUNCTION AND GET THE NECESSARY DATA
%-------------------------------------------------------------------------
data = processSheet(F,parameters);

%-------------------------------------------------------------------------
% USE THE WRITE FUNCTION TO WRITE OUT THE NECESSARY DATA
%-------------------------------------------------------------------------
writeCalibrationJson(F, data)
