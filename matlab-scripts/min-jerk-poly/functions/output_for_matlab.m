%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025 Giri M. Kumar, Andrea L'Afflitto. All rights reserved.                                                        
%                                                                             
% Redistribution and use in source and binary forms, with or without          
% modification, are permitted provided that the following conditions are met: 
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
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE   
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE  
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE   
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR         
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF        
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS    
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN     
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)     
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE  
% POSSIBILITY OF SUCH DAMAGE.                                                 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File:       output_for_matlab.m                                                   
% Author:     Giri Mugundan Kumar
% Date:       June 20, 2025
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: This function is used to output the trajectory for matlab
%              use.
%
% GitHub: https://github.com/girimugundankumar/qrbp-unwinding-constraints.git   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [] = output_for_matlab(t, position, velocity, acceleration, ...
                                yaw, yaw_dot, yaw_dot_dot, traj_data)

% This function provides a reference vector at any time time, including:
% - Position (x, y, z)
% - Linear velocity (vx, vy, vz)
% - Linear acceleration (ax, ay, az)
% - Euler angles (roll, pitch, yaw)
% - Angular rates (roll_dot, pitch_dot, yaw_dot)
% - Angular rates dot (roll_ddot, pitch_ddot, yaw_ddot)
% - Quaternion (qw, qx, qy, qz)
% - Quaternion derivative (qw_dot, qx_dot, qy_dot, qz_dot)

ref_traj_function = @(time) [
    interp1(t', position.x', time, 'linear', 'extrap');         % x
    interp1(t', position.y', time, 'linear', 'extrap');        % y
    interp1(t', position.z', time, 'linear', 'extrap');        % z
    interp1(t', velocity.x', time, 'linear', 'extrap');         % vx
    interp1(t', velocity.y', time, 'linear', 'extrap');         % vy
    interp1(t', velocity.z', time, 'linear', 'extrap');         % vz
    interp1(t', acceleration.x', time, 'linear', 'extrap');     % ax
    interp1(t', acceleration.y', time, 'linear', 'extrap');     % ay
    interp1(t', acceleration.z', time, 'linear', 'extrap');     % az
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % roll
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % pitch
    interp1(t', yaw', time, 'linear', 'extrap');                % yaw
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % roll_dot
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % pitch_dot
    interp1(t', yaw_dot', time, 'linear', 'extrap');            % yaw_dot
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % roll_ddot
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % pitch_ddot
    interp1(t', yaw_dot_dot', time, 'linear', 'extrap');        % yaw_ddot
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % qw
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % qx
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % qy
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % qz
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % qw_dot
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % qx_dot
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % qy_dot
    interp1(t', zeros(size(t')), time, 'linear', 'extrap');     % qz_dot
];

% Define the time vector to check (use allTimes for direct comparison)
t_check = t(:)';

% Preallocate arrays for trajectory data (26 columns)
ref_vals = zeros(numel(t_check), 26); 
orig_vals = zeros(numel(t_check), 26);

% Loop through each time and compute both the function output and the original data
for i = 1:numel(t_check)
    time = t_check(i);
    % Evaluate the reference function at this time
    ref_vals(i,:) = ref_traj_function(time)';
    % Extract original data at this time index (no interpolation)
    orig_vals(i,:) = [
        position.x(1,i), ...
        position.y(1,i), ...
        position.z(1,i), ...
        velocity.x(1,i), ...
        velocity.y(1,i), ...
        velocity.z(1,i), ...
        acceleration.x(1,i), ...
        acceleration.y(1,i), ...
        acceleration.z(1,i), ...
        0, ...
        0, ...
        yaw(1,i), ...
        0, ...
        0, ...
        yaw_dot(1,i), ...
        0, ...
        0, ...
        yaw_dot_dot(1,i), ...
        0, ...
        0, ...
        0, ...
        0, ...
        0, ...
        0, ...
        0, ...
        0, ...
    ];
end

% Variable names for plotting
groups = { ...
    {'x', 'y', 'z'}, ...
    {'vx', 'vy', 'vz'}, ...
    {'ax', 'ay', 'az'}, ...
    {'roll', 'pitch', 'yaw'}, ...
    {'roll_dot', 'pitch_dot', 'yaw_dot'}, ...
    {'roll_ddot', 'pitch_ddot', 'yaw_ddot'}, ...
    {'qw', 'qx', 'qy', 'qz'}, ...
    {'qw_dot', 'qx_dot', 'qy_dot', 'qz_dot'} ...
};

% Indices for each group in the ref_vals/orig_vals
indices = { ...
    1:3, ...
    4:6, ...
    7:9, ...
    10:12, ...
    13:15, ...
    16:18, ...
    19:22, ...
    23:26 ...
};

% Optionally, display the maximum absolute error for each variable
max_abs_error = max(abs(ref_vals - orig_vals), [], 1);
disp('Maximum absolute error for each variable:');
for g = 1:length(groups)
    idx = indices{g};
    group_names = groups{g};
    for s = 1:length(idx)
        fprintf('%-10s: %.3e\n', group_names{s}, max_abs_error(idx(s)));
    end
end

% 1. Export the reference trajectory data (as before)
ref_traj_table = table( ...
    t', ...
    position.x', position.y', position.z', ...
    velocity.x', velocity.y', velocity.z', ...
    acceleration.x', acceleration.y', acceleration.z', ...
    zeros(size(t')), zeros(size(t')), yaw', ...
    zeros(size(t')), zeros(size(t')), yaw_dot', ...
    zeros(size(t')), zeros(size(t')), yaw_dot_dot', ...
    zeros(size(t')), zeros(size(t')), zeros(size(t')), zeros(size(t')), ...
    zeros(size(t')), zeros(size(t')), zeros(size(t')), zeros(size(t')), ...
    'VariableNames', { ...
        'time', ...
        'pos_x', 'pos_y', 'pos_z', ...
        'vel_x', 'vel_y', 'vel_z', ...
        'acc_x', 'acc_y', 'acc_z', ...
        'roll', 'pitch', 'yaw', ...
        'roll_dot', 'pitch_dot', 'yaw_dot', ...
        'roll_ddot', 'pitch_ddot', 'yaw_ddot', ...
        'quat_w', 'quat_x', 'quat_y', 'quat_z', ...
        'quat_dot_w', 'quat_dot_x', 'quat_dot_y', 'quat_dot_z' ...
    });

    % Extract the traj_name, codebase_name and the alpha factor
    traj_name = char(traj_data.name);
    codebase_name = char(traj_data.codebase);

    % Get the alpha factor as string and format for filename
    alpha_val = traj_data.alphaFactor;
    alpha_str = num2str(alpha_val);
    if ~contains(alpha_str, '.')
        alpha_str = [alpha_str 'p0'];
    else
        alpha_str = strrep(alpha_str, '.', 'p');
    end

    % Create the trajectory output name as a character array
    traj_output_name = [traj_name alpha_str];

    % Create folder path dynamically in pwd/output/cpp/traj_output_name
    folder_path = fullfile(pwd, 'output', codebase_name, traj_output_name);

    % Check if folder exists, if not create it
    if ~exist(folder_path, 'dir')
        mkdir(folder_path);
    end

    csv_filename = fullfile(folder_path, [traj_output_name, '.csv']);
    writetable(ref_traj_table, csv_filename);

    % Copy input/datafile.yaml to the new folder, using pwd for absolute path
    source_file = fullfile(pwd, 'input', 'datafile.yaml');
    dest_file = fullfile(folder_path, [traj_output_name, '.yaml']);

    if exist(source_file, 'file')
        copyfile(source_file, dest_file);
    else
        warning('Source YAML file does not exist: %s', source_file);
    end

    disp(['CSV file saved as: ' csv_filename]);
    disp(['YAML file copied to: ' dest_file]);

end

