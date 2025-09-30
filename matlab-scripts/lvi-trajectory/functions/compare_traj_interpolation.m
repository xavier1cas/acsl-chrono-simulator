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
% File:        compare_traj_interpolation.m
% Author:      Giri Mugundan Kumar
% Date:        June 20, 2025
% For info:    Andrea L'Afflitto
%              a.lafflitto@vt.edu
% 
% Description: Function that compares if the interpolation function works.
%
% Github: https://github.com/girimugundankumar/acsl-physics-sim.git
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function compare_traj_interpolation(traj, ref_traj_function)
% =========================================================================
% Function: compare_traj_interpolation
%
% Purpose:
%   Compares direct trajectory data in traj against its interpolated version from
%   ref_traj_function at each sample time, and visualizes the comparison and
%   maximum absolute error for each state variable component and group.
%
% Inputs:
%   traj              - Struct containing original trajectory data arrays.
%   ref_traj_function - Function handle for reference trajectory interpolation,
%                       returns 36x1 column vectors.
% =========================================================================

% Define the time vector to check (direct correspondence)
t_check = traj.time_ned;
N = numel(t_check);

% Preallocate arrays for output data (36 variable channels)
ref_vals = zeros(N, 36);
org_vals = zeros(N, 36);

% Loop through each time step (compare original data vs interpolated ref)
for k = 1:N
    t = t_check(k);
    ref_vals(k,:) = ref_traj_function(t)';
    org_vals(k,:) = [traj.positions_ned(k,1:3), ...
                     traj.velocities_ned(k,1:3), ...
                     traj.accelerations_ned(k,1:3), ...
                     traj.euler_ned(k,1:3), ...
                     zeros(1,3), ...
                     zeros(1,3), ...
                     traj.quats_ned(k,1:4), ...
                     traj.quats_dot_ned(k,1:4), ...
                     zeros(1,4), ...
                     traj.angular_vels_ned(k,1:3), ...
                     traj.angular_accs_ned(k,1:3)];
end

% Group names and indices for labeling and loop organization
groups = { {'x', 'y', 'z'}, ...
           {'vx', 'vy', 'vz'}, ...
           {'ax', 'ay', 'az'}, ...
           {'roll', 'pitch', 'yaw'}, ...
           {'roll_dot', 'pitch_dot', 'yaw_dot'}, ...
           {'roll_ddot', 'pitch_ddot', 'yaw_ddot'}, ...
           {'qw', 'qx', 'qy', 'qz'}, ...
           {'qw_dot', 'qx_dot', 'qy_dot', 'qz_dot'}, ...
           {'qw_ddot', 'qx_ddot', 'qy_ddot', 'qz_ddot'}, ...
           {'wx', 'wy', 'wz'}, ...
           {'alpha_x', 'alpha_y', 'alpha_z'} };

indices = { 1:3,4:6,7:9,10:12,13:15,16:18,19:22,23:26,27:30,31:33,34:36 };

colors = {'r', 'g', 'b', 'k'}; % Up to 4 subplots/group

% Loop through groups and plot each variable channel
for g = 1:length(groups)
    figure;
    group_names = groups{g};
    idx = indices{g};
    n_subplots = length(idx);
    for s = 1:n_subplots
        subplot(n_subplots,1,s);
        plot(t_check, org_vals(:,idx(s)), [colors{s} '-'], 'LineWidth', 1.5);
        hold on
        plot(t_check, ref_vals(:,idx(s)), [colors{s} '-.'], 'LineWidth', 1.5);
        xlabel('Time [s]', 'Interpreter', 'none');
        ylabel(group_names{s}, 'Interpreter', 'none');
        title([group_names{s}, ': Original (solid) vs Interpolated (dashed)'], 'Interpreter', 'none');
        legend({[group_names{s} ' original'], [group_names{s} ' interp']}, 'Interpreter', 'none');
        grid on;
    end
end


% Display the maximum absolute error for each variable
max_abs_error = max(abs(ref_vals - org_vals), [], 1);
disp('Maximum absolute error for each variable:');
for g = 1:length(groups)
    idx = indices{g};
    group_names = groups{g};
    for s = 1:length(idx)
        fprintf('%-12s: %.3e\n', group_names{s}, max_abs_error(idx(s)));
    end
end
end

