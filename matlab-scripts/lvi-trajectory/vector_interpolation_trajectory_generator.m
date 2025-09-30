%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025 Giri M. Kumar, Andrea L'Afflitto. All rights reserved.                                                        
%                                                                             
% Redistribution and use in source and binary forms, with or without          
% modification, are permitted provided that the following conditions 
% are met: 
%                                                                             
% 1. Redistributions of source code must retain the above copyright notice,   
%    this list of conditions and the following disclaimer.                    
%                                                                             
% 2. Redistributions in binary form must reproduce the above copyright        
%    notice, this list of conditions and the following disclaimer in the      
%    documentation and/or other materials provided with the distribution.     
%                                                                             
% 3. Neither the name of the copyright holder nor the names of its            
%    contributors may be used to endorse or promote products derived from     
%    this software without specific prior written permission.                 
%                                                                             
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
% "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
% LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
% PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER 
% OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
% EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
% PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.                                                 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File:        vector_interpolation_trajectory_generator.m
% Author:      Giri Mugundan Kumar
% Date:        June 20, 2025
% For info:    Andrea L'Afflitto
%              a.lafflitto@vt.edu
% 
% Description: This live script is used to generate csv files for the 
%              vector-interpolation-trajectory module in acsl-physics-sim.
%              (Please note that this script does not work with any other
%               trajectory module in the simulator)
%
% Github: https://github.com/girimugundankumar/acsl-physics-sim.git
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%% NOTE -------------------------------------------------------------------
% - This script requires the Robotics System Toolbox
% - This script must be run from the directory containing it. 
% - Paths are hardcoded therefore the previous note is important!
% -------------------------------------------------------------------------

%% ------------------------------------------------- Process the input file

% Reset the workspace
clear all
close all
clc

% Add path to the functions
addpath('functions/');

% Add path to the yaml library
addpath(genpath('../../libraries/third-party/yamlmatlab/'));
addpath('../../libraries/third-party/yamlmatlab/');

% Inject the yaml file - Get the points, orientations, times and the name 
data = process_input_file('input/datafile.yaml');
% note: please note that the datafile name,location is hardcoded.

%% ------------------------------------------------- Compute the trajectory

[traj.time_ned,          ...
 traj.positions_ned,     ...
 traj.rotations_ned,     ...
 traj.euler_ned,         ...
 traj.quats_ned,         ...
 traj.quats_dot_ned,     ...
 traj.velocities_ned,    ...
 traj.accelerations_ned, ...
 traj.angular_vels_ned,  ...
 traj.angular_accs_ned] = compute_trajectory(data);

%% ------------------------------- Plot trajecotry and animate moving frame

animate_trajectory(traj);

%% ---- Plot position, velocity and acceleration components (translational)

plot_translational_data(traj);

%% ------------------------ Plot the Euler angles in NED frame (rotational)

plot_rotational_data(traj);

%% ------------------------------- Create the reference trajectory function

% -------------------------------------------------------------------------
% function: ref_traj_function
%
% Purpose:
%   This function provides a reference vector at any time t. 
%
% Fields:
%   pos_x        % [Nx1] X position in NED (meters)
%   pos_y        % [Nx1] Y position in NED (meters)
%   pos_z        % [Nx1] Z position in NED (meters)
%   vel_x        % [Nx1] X linear velocity (meters/second)
%   vel_y        % [Nx1] Y linear velocity (meters/second)
%   vel_z        % [Nx1] Z linear velocity (meters/second)
%   acc_x        % [Nx1] X linear acceleration (meters/second^2)
%   acc_y        % [Nx1] Y linear acceleration (meters/second^2)
%   acc_z        % [Nx1] Z linear acceleration (meters/second^2)
%   euler_roll   % [Nx1] Euler roll angle (radians)
%   euler_pitch  % [Nx1] Euler pitch angle (radians)
%   euler_yaw    % [Nx1] Euler yaw angle (radians)
%   euler_rate_r % [Nx1] Euler roll rate (radians/second)
%   euler_rate_p % [Nx1] Euler pitch rate (radians/second)
%   euler_rate_y % [Nx1] Euler yaw rate (radians/second)
%   euler_acc_r  % [Nx1] Euler roll acceleration (radians/second^2)
%   euler_acc_p  % [Nx1] Euler pitch acceleration (radians/second^2)
%   euler_acc_y  % [Nx1] Euler yaw acceleration (radians/second^2)
%   quat_w       % [Nx1] Quaternion W component
%   quat_x       % [Nx1] Quaternion X component
%   quat_y       % [Nx1] Quaternion Y component
%   quat_z       % [Nx1] Quaternion Z component
%   quat_rate_w  % [Nx1] Quaternion derivative W
%   quat_rate_x  % [Nx1] Quaternion derivative X
%   quat_rate_y  % [Nx1] Quaternion derivative Y
%   quat_rate_z  % [Nx1] Quaternion derivative Z
%   quat_acc_w   % [Nx1] Quaternion second derivative W
%   quat_acc_x   % [Nx1] Quaternion second derivative X
%   quat_acc_y   % [Nx1] Quaternion second derivative Y
%   quat_acc_z   % [Nx1] Quaternion second derivative Z
%   ang_vel_x    % [Nx1] X angular velocity (radians/second)
%   ang_vel_y    % [Nx1] Y angular velocity (radians/second)
%   ang_vel_z    % [Nx1] Z angular velocity (radians/second)
%   ang_acc_x    % [Nx1] X angular acceleration (radians/second^2)
%   ang_acc_y    % [Nx1] Y angular acceleration (radians/second^2)
%   ang_acc_z    % [Nx1] Z angular acceleration (radians/second^2)
% -------------------------------------------------------------------------

ref_traj_function = @(t) [
interp1(traj.time_ned, traj.positions_ned(:,1), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.positions_ned(:,2), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.positions_ned(:,3), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.velocities_ned(:,1), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.velocities_ned(:,2), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.velocities_ned(:,3), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.accelerations_ned(:,1), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.accelerations_ned(:,2), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.accelerations_ned(:,3), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.euler_ned(:,1), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.euler_ned(:,2), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.euler_ned(:,3), t, 'linear', 'extrap');
interp1(traj.time_ned, zeros(size(traj.time_ned)), t, 'linear', 'extrap');
interp1(traj.time_ned, zeros(size(traj.time_ned)), t, 'linear', 'extrap');
interp1(traj.time_ned, zeros(size(traj.time_ned)), t, 'linear', 'extrap');
interp1(traj.time_ned, zeros(size(traj.time_ned)), t, 'linear', 'extrap');
interp1(traj.time_ned, zeros(size(traj.time_ned)), t, 'linear', 'extrap');
interp1(traj.time_ned, zeros(size(traj.time_ned)), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.quats_ned(:,1), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.quats_ned(:,2), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.quats_ned(:,3), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.quats_ned(:,4), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.quats_dot_ned(:,1), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.quats_dot_ned(:,2), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.quats_dot_ned(:,3), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.quats_dot_ned(:,4), t, 'linear', 'extrap');
interp1(traj.time_ned, zeros(size(traj.time_ned)), t, 'linear', 'extrap');
interp1(traj.time_ned, zeros(size(traj.time_ned)), t, 'linear', 'extrap');
interp1(traj.time_ned, zeros(size(traj.time_ned)), t, 'linear', 'extrap');
interp1(traj.time_ned, zeros(size(traj.time_ned)), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.angular_vels_ned(:,1), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.angular_vels_ned(:,2), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.angular_vels_ned(:,3), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.angular_accs_ned(:,1), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.angular_accs_ned(:,2), t, 'linear', 'extrap');
interp1(traj.time_ned, traj.angular_accs_ned(:,3), t, 'linear', 'extrap');
];

%% -------------------------- Check reference trajectory interpolation data

compare_traj_interpolation(traj, ref_traj_function);

%% --------------- Export reference trajectory data and copy yaml file over

export_ref_traj_and_yaml(traj, data)
