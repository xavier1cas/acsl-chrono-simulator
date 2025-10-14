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
% File:       read_input.m                                                   
% Author:     Giri Mugundan Kumar
% Date:       June 20, 2025
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: Function reads in the input datafile at input/datafile.yaml
%
% GitHub: https://github.com/girimugundankumar/qrbp-unwinding-constraints.git   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function traj = read_input(yaml_filepath)
%READ_INPUT Loads trajectory data from a YAML file into a MATLAB struct.
%
%   traj = READ_INPUT(yaml_filepath)
%
%   Inputs:
%       yaml_filepath - String, path to the YAML file
%
%   Outputs:
%       traj - Struct with fields:
%           .name                    : trajectory name (char or string)
%           .alphaFactor             : alpha factor (scalar)
%           .wpts_nt                 : Nx3 matrix of waypoints [x y z]
%           .segment_relative_times  : Nx1 vector of times

% Read YAML file
yamlData = yaml.ReadYaml(yaml_filepath);

% Extract metadata
traj.name = yamlData.name;
traj.alphaFactor = yamlData.alpha_factor;
traj.numsamples = yamlData.numsamples;
traj.step = yamlData.tstep;
traj.codebase = yamlData.codebase;
traj.hover_time = yamlData.hover_time;

% Extract waypoints and initialize empty matrices
wpts = yamlData.waypoints;
num_wpts = numel(wpts);
traj.wpts_nt = zeros(num_wpts, 3);
traj.segment_relative_times = zeros(num_wpts, 1);

% Loop through and assign the values to waypoints and relative times
for i = 1:num_wpts
    traj.wpts_nt(i, :) = [wpts{i}.x, wpts{i}.y, wpts{i}.z];
    traj.segment_relative_times(i) = wpts{i}.time;
end

% tranpose the way points
traj.wpts = transpose(traj.wpts_nt);

% Compute the absolute times
segment_times = traj.segment_relative_times;
absolute_times = traj.alphaFactor * cumsum(segment_times);

traj.tpts = absolute_times;


end
