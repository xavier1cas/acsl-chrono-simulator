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
% File:        compute_trajectory.m
% Author:      Giri Mugundan Kumar
% Date:        June 20, 2025
% For info:    Andrea L'Afflitto
%              a.lafflitto@vt.edu
% 
% Description: Function reads in the data vector and outputs the computed
%              trajectory.
%
% Github: https://github.com/girimugundankumar/acsl-physics-sim.git
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [time_ned, ...
          positions_ned, ...
          rotations_ned, ...
          euler_ned, ...
          quats_ned, ...
          quats_dot_ned, ...
          velocities_ned, ...
          accelerations_ned, ...
          angular_vels_ned, ...
          angular_accs_ned] = compute_trajectory(data)
% ========================================================================
% Function: compute_trajectory
%
% Purpose:
%   Generates interpolated trajectory data between a sequence of input
%   waypoints, including positions, orientations, linear and angular
%   velocities/accelerations, all referenced to the NED frame.
%
% Inputs:
%   data     - Structure containing:
%              - data.T    : {Nx4x4} cell array of SE(3) transform matrices
%                            representing consecutive waypoints in NED.
%              - data.tpts : 1xN vector of timestamps for each waypoint.
%
% Outputs:
%   time_ned          - Nx1 array: time (seconds).
%   positions_ned     - Nx3 array: Position (NED; meters).
%   rotations_ned     - 3x3xN array: Rotation matrices (NED).
%   euler_ned         - Nx3 array: Euler orientations (NED).
%   quats_ned         - Nx4 array: Quaternion orientations (NED).
%   quats_dot_ned     - Nx4 array: Quaternion first derivative (NED).
%   velocities_ned    - Nx3 array: Linear velocity (NED; m/s).
%   accelerations_ned - Nx3 array: Linear acceleration (NED; m/s^2).
%   angular_vels_ned  - Nx3 array: Angular velocity (NED; rad/s).
%   angular_accs_ned  - Nx3 array: Angular acceleration (NED; rad/s^2).
%
% Algorithm:
%   - For each consecutive pair of waypoints, computes SE(3) transforms
%     and derivatives using transformtraj (MATLAB Robotics Toolbox).
%   - Concatenates all segments and assembles final trajectory data.
%   - Output arrays are indexed so that sample i corresponds to time.
% ========================================================================

% Initialize storage for full trajectory segment outputs
allTforms = [];  % Transformation matrices
allVels   = [];  % 6D velocity vectors (angular;linear)
allAccs   = [];  % 6D acceleration vectors (angular;linear)
allTimes  = [];  % Time stamps

for i = 1:numel(data.T)-1
    % Get the time interval for this trajectory segment
    tInterval = [data.tpts(i), data.tpts(i+1)];
    % Generate evenly spaced sample times within segment (100 per segment)
    tvec = linspace(tInterval(1), tInterval(2), 100);
    % Compute transforms/velocities/accelerations using SE(3) interpolation
    [tforms, vels, accs] = transformtraj(data.T{i}, data.T{i+1}, tInterval, tvec);

    % Remove first sample point ("overlap") from all but the first segment
    allTforms = cat(3, allTforms, tforms(:,:,2:end));
    allVels   = [allVels, vels(:,2:end)];
    allAccs   = [allAccs, accs(:,2:end)];
    allTimes  = [allTimes, tvec(2:end)];
end

% Prepend the initial state for full trajectory arrays
allTforms = cat(3, data.T{1}, allTforms);
allVels   = [zeros(6,1), allVels];
allAccs   = [zeros(6,1), allAccs];
allTimes  = [data.tpts(1), allTimes];

% Extract the euler angles
allEul = zeros(size(allTforms, 3), 3);
for k = 1:size(allTforms, 3)
    allEul(k,:) = rotm2eul(allTforms(1:3,1:3,k), 'ZYX');
end

% Extract the quaternions from the rotation matrices (NED frame)
allQuat = zeros(size(allTforms, 3), 4);  % [w x y z]
for k = 1:size(allTforms, 3)
    allQuat(k,:) = rotm2quat(allTforms(1:3,1:3,k));
end

% Compute quaternion derivated given angular velocities (NED frame)
% angular_vels_ned: Nx3 matrix [wx wy wz]
allQuatsDot = zeros(size(allQuat)); % Nx4
for k = 1:size(allQuat,1)
    w = allQuat(k,1);
    x = allQuat(k,2);
    y = allQuat(k,3);
    z = allQuat(k,4);
    wx = allVels(1,k);
    wy = allVels(2,k);
    wz = allVels(3,k);

    allQuatsDot(k,1) = -0.5 * (x*wx + y*wy + z*wz);
    allQuatsDot(k,2) =  0.5 * (w*wx + y*wz - z*wy);
    allQuatsDot(k,3) =  0.5 * (w*wy - x*wz + z*wx);
    allQuatsDot(k,4) =  0.5 * (w*wz + x*wy - y*wx);
end

% Convert quaternions back to Euler angles (ZYX order)
eul_from_quat = zeros(size(allQuat,1),3); %Nx3
for k = 1:size(allQuat,1)
    eul_from_quat(k,:) = quat2eul(allQuat(k,:), 'ZYX');
end

% Compare with previously extracted Euler angles (allEul)
angle_diff = allEul - eul_from_quat;
angle_diff = mod(angle_diff + pi, 2*pi) - pi; % Normalize t [-pi,pi]

% Plot Euler angle errors over time
figure;
% Roll error
plot(allTimes, rad2deg(angle_diff(:,3)), 'r', 'LineWidth', 1.5); 
hold on;
% Pitch error
plot(allTimes, rad2deg(angle_diff(:,2)), 'g', 'LineWidth', 1.5);
% Yaw error
plot(allTimes, rad2deg(angle_diff(:,1)), 'b', 'LineWidth', 1.5);
xlabel('Time [s]');
ylabel('Euler Angle Error [deg]');
title('Euler Angle Error Between rotm2eul and quat2eul');
legend('Roll error', 'Pitch error', 'Yaw error');
grid on;

% (Optional) Display the maximum absolute difference for each component in degrees
max_diff = max(abs(angle_diff), [], 1);
disp('Maximum absolute difference between Euler angles [yaw pitch roll] (degrees):');
disp(rad2deg(max_diff));

% ------------------------------------------------------------------------
% Extract output trajectory data:
%   - Positions: translation part of SE(3)
%   - Rotations: rotation matrix part of SE(3)
%   - Velocities/Accelerations: angular and linear from twist/se(3) vectors
% ------------------------------------------------------------------------

time_ned            = allTimes(:,:);                  % Nx1
positions_ned       = squeeze(allTforms(1:3,4,:))';   % Nx3
rotations_ned       = allTforms(1:3,1:3,:);           % 3x3xN
euler_ned           = allEul(:,:);                    % Nx3
quats_ned           = allQuat(:,:);                   % Nx4
quats_dot_ned       = allQuatsDot(:,:);               % Nx4
velocities_ned      = allVels(4:6,:)';                % Nx3
accelerations_ned   = allAccs(4:6,:)';                % Nx3
angular_vels_ned    = allVels(1:3,:)';                % Nx3
angular_accs_ned    = allAccs(1:3,:)';                % Nx3


end


