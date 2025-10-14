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
% File:       structToJSONcpp.m                                                   
% Author:     Giri Mugundan Kumar
% Date:       June 20, 2025
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: This function saves the json files in the correct format for
%              the flightstack and the chrono simulator.
%
% GitHub: https://github.com/girimugundankumar/qrbp-unwinding-constraints.git   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function structToJSONcpp(json_struct, traj_data)
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

    % Define JSON filename with traj_output_name
    json_filename = fullfile(folder_path, [traj_output_name, '.json']);
    disp(json_filename)

    % Encode struct to JSON with pretty print
    jsonFile = jsonencode(json_struct, 'PrettyPrint', true);

    % Write JSON to file (ensure filename is char)
    fid = fopen(json_filename, 'w');
    if fid == -1
        error('Cannot create JSON file.');
    end
    fwrite(fid, jsonFile, 'char');
    fclose(fid);

    % Copy input/datafile.yaml to the new folder, using pwd for absolute path
    source_file = fullfile(pwd, 'input', 'datafile.yaml');
    dest_file = fullfile(folder_path, [traj_output_name, '.yaml']);

    if exist(source_file, 'file')
        copyfile(source_file, dest_file);
    else
        warning('Source YAML file does not exist: %s', source_file);
    end

    disp(['JSON file saved as: ' json_filename]);
    disp(['YAML file copied to: ' dest_file]);
end
