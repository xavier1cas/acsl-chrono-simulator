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
% File:        process_input_file.m
% Author:      Giri Mugundan Kumar
% Date:        June 20, 2025
% For info:    Andrea L'Afflitto
%              a.lafflitto@vt.edu
% 
% Description: Function reads in the input datafile at input/datafile.yaml
%
% Github: https://github.com/girimugundankumar/acsl-physics-sim.git
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function data = process_input_file(yaml_filepath)
% =========================================================================
% Function: process_input_file
%
% Purpose:
%   Loads user trajectory data from a YAML file and parses it into a MATLAB 
%   struct suitable for trajectory generation and analysis.
%
% Inputs:
%   yaml_filepath : String path to the user-provided YAML file describing 
%                   the trajectory.
%
% Outputs:
%   data : Struct containing extracted and converted trajectory information
%       .name                   - Trajectory name (char or string)
%       .codebase               - Codebase descriptor (char or string)
%       .T                      - {Nx1} cell array of SE(3) transformation 
%                                 matrices for each waypoint
%       .segment_relative_times - [Nx1] vector of relative segment 
%                                 durations (seconds)
%       .tpts                   - [Nx1] vector of absolute time stamps for 
%                                 waypoints (seconds)
% =========================================================================

% -------------------------------------------------------------------------
% Read YAML data with SnakeYAML interface, output as a struct
% -------------------------------------------------------------------------
yamlData = yaml.ReadYaml(yaml_filepath);

% -------------------------------------------------------------------------
% Extract metadata fields from YAML (trajectory name, codebase type)
% -------------------------------------------------------------------------
data.name = yamlData.name;
data.codebase = yamlData.codebase;

% -------------------------------------------------------------------------
% Parse waypoints section:
%   - Each waypoint contains position (x, y, z), orientation (Euler angles)
%     and relative segment time.
%   - Store transformation matrices in data.T
%   - Store segment times in data.segment_relative_times
% -------------------------------------------------------------------------
wpts = yamlData.waypoints;
num_wpts = numel(wpts);

for i = 1:num_wpts
    % Build SE(3) transformation: translation * Euler-angles-to-rotation
    data.T{i} = trvec2tform([wpts{i}.x wpts{i}.y wpts{i}.z]) * ...
                eul2tform([deg2rad(wpts{i}.psi) ...
                           deg2rad(wpts{i}.theta) ...
                           deg2rad(wpts{i}.phi)]);

    % Store duration of segment following this waypoint
    data.segment_relative_times(i) = wpts{i}.time;
end

% -------------------------------------------------------------------------
% Compute absolute waypoint times by cumulative sum of segment durations
% -------------------------------------------------------------------------
segment_times = data.segment_relative_times;
absolute_times = cumsum(segment_times);
data.tpts = absolute_times;

% -------------------------------------------------------------------------
% Consistency check: Ensure time and waypoint array lengths match
% -------------------------------------------------------------------------
if length(data.tpts) ~= length(data.T)
    error('waypointTimes length must match number of waypoints');
end
end
