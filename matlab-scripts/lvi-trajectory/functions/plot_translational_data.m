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
% File:        plot_translational_data.m
% Author:      Giri Mugundan Kumar
% Date:        June 20, 2025
% For info:    Andrea L'Afflitto
%              a.lafflitto@vt.edu
% 
% Description: Function reads in the traj vector and plots the
%              translational states of the generated trajectory
%
% Github: https://github.com/girimugundankumar/acsl-physics-sim.git
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = plot_translational_data(traj)
% =========================================================================
% Function: plot_translational_data
%
% Purpose:
%   Plots the time histories of the X, Y, Z components of position, velocity,
%   and acceleration for a trajectory in the NED (North-East-Down) frame.
%   Each component is shown with a separate color and labeled legend.
%
% Inputs:
%   traj - Struct containing:
%           .time_ned           - [Nx1] vector of sample times (seconds)
%           .positions_ned      - [Nx3] position components (meters)
%           .velocities_ned     - [Nx3] velocity components (m/s)
%           .accelerations_ned  - [Nx3] acceleration components (m/s^2)
% =========================================================================

% -------------------------------------------------------------------------
% Plot NED Position Components vs Time
% -------------------------------------------------------------------------
figure;
plot(traj.time_ned, traj.positions_ned(:,1), 'r', 'LineWidth', 1.5); 
hold on;
plot(traj.time_ned, traj.positions_ned(:,2), 'g', 'LineWidth', 1.5);
plot(traj.time_ned, traj.positions_ned(:,3), 'b', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Position (m)');
title('Position Components (NED)');
legend('X', 'Y', 'Z');
grid on;

% -------------------------------------------------------------------------
% Plot NED Velocity Components vs Time
% -------------------------------------------------------------------------
figure;
plot(traj.time_ned, traj.velocities_ned(:,1), 'r', 'LineWidth', 1.5); 
hold on;
plot(traj.time_ned, traj.velocities_ned(:,2), 'g', 'LineWidth', 1.5);
plot(traj.time_ned, traj.velocities_ned(:,3), 'b', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Velocity (m/s)');
title('Velocity Components (NED)');
legend('X', 'Y', 'Z');
grid on;

% -------------------------------------------------------------------------
% Plot NED Acceleration Components vs Time
% -------------------------------------------------------------------------
figure;
plot(traj.time_ned, traj.accelerations_ned(:,1), 'r', 'LineWidth', 1.5); 
hold on;
plot(traj.time_ned, traj.accelerations_ned(:,2), 'g', 'LineWidth', 1.5);
plot(traj.time_ned, traj.accelerations_ned(:,3), 'b', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Acceleration (m/s^2)');
title('Acceleration Components (NED)');
legend('X', 'Y', 'Z');
grid on;
end



