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
% File:        animate_trajectory.m
% Author:      Giri Mugundan Kumar
% Date:        June 20, 2025
% For info:    Andrea L'Afflitto
%              a.lafflitto@vt.edu
% 
% Description: Function reads in the traj vector and animates the generated
%              trajectory.
%
% Github: https://github.com/girimugundankumar/acsl-physics-sim.git
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = animate_trajectory(traj)
% =========================================================================
% Function: animate_trajectory
%
% Purpose:
%   Visualizes a 3D trajectory by plotting the path and animating a moving
%   coordinate frame along the trajectory using position and orientation 
%   data. Useful for debugging and illustrating UAV or robotic trajectory 
%   following.
%
% Inputs:
%   traj - Struct containing:
%          .positions_ned   - Nx3 matrix of position samples (in NED)
%          .rotations_ned   - 3x3xN array of rotation matrices (body-to-NED)
% =========================================================================

% -------------------------------------------------------------------------
% 1. Plot the full trajectory path in 3D
% -------------------------------------------------------------------------
figure;
plot3(traj.positions_ned(:,1), ...
      traj.positions_ned(:,2), ...
      traj.positions_ned(:,3), ...
      'k.-', 'LineWidth', 1.5, 'MarkerSize', 10);   % Plot trajectory
xlabel('X');
ylabel('Y');
zlabel('Z');
title('Waypoint Trajectory with Moving Frame Animation');
grid on;
axis equal;
hold on;

% Reverse Z and Y axes to match NED (North-East-Down) conventions
set(gca, 'ZDir', 'reverse');
set(gca, 'YDir', 'reverse');

% -------------------------------------------------------------------------
% 2. Initialize animated moving frame (quiver3 for body X, Y, Z axes)
% -------------------------------------------------------------------------
frameScale = 0.1;  % Scale for axis vectors

hX = quiver3(0, 0, 0, 0, 0, 0, 'r', 'LineWidth', 2, 'MaxHeadSize', 1);  % X axis (red)
hY = quiver3(0, 0, 0, 0, 0, 0, 'g', 'LineWidth', 2, 'MaxHeadSize', 1);  % Y axis (green)
hZ = quiver3(0, 0, 0, 0, 0, 0, 'b', 'LineWidth', 2, 'MaxHeadSize', 1);  % Z axis (blue)

legend('Trajectory', 'X axis', 'Y axis', 'Z axis');
view(3);

% Defensive: Immediately check if handles were created
if ~isvalid(hX) || ~isvalid(hY) || ~isvalid(hZ)
    error('Quiver axes could not be initialized — aborting animation.');
end

% -------------------------------------------------------------------------
% 3. Animation loop — for each time step, update the frame axes
% -------------------------------------------------------------------------
for i = 1:size(traj.positions_ned, 1)
    % Defensive: Check that handles are still valid on every frame!
    if ~isvalid(hX) || ~isvalid(hY) || ~isvalid(hZ)
        warning('Quiver object(s) deleted or figure closed — animation stopped.');
        break;
    end

    origin = traj.positions_ned(i, :);      % Current position sample [x y z]
    R = traj.rotations_ned(:, :, i);        % Current rotation matrix, body-to-NED

    % Update X, Y, Z axis arrows of frame using the current orientation
    set(hX, 'XData', origin(1), ...
            'YData', origin(2), ...
            'ZData', origin(3), ...
            'UData', frameScale * R(1, 1), ...
            'VData', frameScale * R(2, 1), ...
            'WData', frameScale * R(3, 1));
    set(hY, 'XData', origin(1), ...
            'YData', origin(2), ...
            'ZData', origin(3), ...
            'UData', frameScale * R(1, 2), ...
            'VData', frameScale * R(2, 2), ...
            'WData', frameScale * R(3, 2));
    set(hZ, 'XData', origin(1), ...
            'YData', origin(2), ...
            'ZData', origin(3), ...
            'UData', frameScale * R(1, 3), ...
            'VData', frameScale * R(2, 3), ...
            'WData', frameScale * R(3, 3));

    pause(0.05);  % Control animation speed
end
end
