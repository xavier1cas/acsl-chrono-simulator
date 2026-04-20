%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Copyright (c) 2025 Giri M. Kumar, Andrea L'Afflitto. All rights
% reserved.
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions
% are met:
%
% 1. Redistributions of source code must retain the above copyright
%    notice, this list of conditions and the following disclaimer.
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
% PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
% HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
% SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
% TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
% PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
% LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
% NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
% SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% File:        writeCalibrationJson.m
% Author:      Giri Mugundan Kumar
% Date:        April 20, 2026
% For info:    Andrea L'Afflitto
%              a.lafflitto@vt.edu
%
% Description:
%   Write calibration data produced by processSheet.m to a JSON file on
%   disk. For a given motor identifier F, this function:
%
%     1) Creates an output/ directory (if it does not exist).
%     2) Creates a motor-specific subfolder output/F/.
%     3) Writes the calibration data fields in DATA to
%        output/F/F_calibration.json, using JSON encoding compatible
%        with MATLAB's jsonencode().
%
%   The JSON file can be consumed by the simulator or flight stack to
%   reconstruct the polynomial models for thrust and rotor speed.
%
% Github: https://github.com/girimugundankumar/acsl-physics-sim.git
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function writeCalibrationJson(F, data)
%WRITECALIBRATIONJSON  Export calibration struct to JSON in output/F/.
%
%   WRITECALIBRATIONJSON(F, DATA) creates (if necessary) the folder
%   output/F/ (relative to the current working directory) and writes
%   the contents of DATA to a JSON file named F_calibration.json in
%   that folder.
%
%   INPUTS:
%     F    : String or char, motor identifier. The JSON file is written
%            to output/F/F_calibration.json.
%
%     DATA : Struct with calibration fields, typically returned by
%            processSheet(F, PARAM), containing:
%              - normalized_thrust_to_thrust_polynomial : 1×n double
%              - thrust_to_normalized_thrust_polynomial : 1×m double
%              - normalized_thrust_to_rps_polynomial    : 1×p double
%              - estimated_Ct                           : scalar double
%
%   Notes:
%     - This function assumes MATLAB R2016b or later, where jsonencode()
%       is available.
%     - All polynomial coefficient vectors are stored as row vectors
%       in the JSON file for consistency.

    %----------------------------------------------------------------------
    % Validate inputs
    %----------------------------------------------------------------------

    if nargin < 2
        error('writeCalibrationJson:NotEnoughInputs', ...
              'Usage: writeCalibrationJson(F, data)');
    end

    % Convert motor identifier to string for robust path handling
    F = string(F);

    % Ensure required fields are present in DATA
    requiredFields = [ ...
        "normalized_thrust_to_thrust_polynomial", ...
        "thrust_to_normalized_thrust_polynomial", ...
        "normalized_thrust_to_rads_polynomial", ...
        "estimated_Ct" ...
    ];

    for k = 1:numel(requiredFields)
        if ~isfield(data, requiredFields(k))
            error('writeCalibrationJson:MissingField', ...
                  'DATA is missing required field: %s', requiredFields(k));
        end
    end

    %----------------------------------------------------------------------
    % Create output directory hierarchy
    %----------------------------------------------------------------------

    % Base output directory
    outBaseDir = "output";

    % Create output/ if it does not already exist
    if ~isfolder(outBaseDir)
        mkdir(outBaseDir);
    end

    % Create motor-specific subfolder output/F/
    motorOutDir = fullfile(outBaseDir, F);
    if ~isfolder(motorOutDir)
        mkdir(motorOutDir);
    end

    %----------------------------------------------------------------------
    % Assemble JSON-serializable struct
    %----------------------------------------------------------------------

    % Store vectors as row vectors for readability and consistency
    jsonStruct = struct();
    jsonStruct.normalized_thrust_to_thrust_polynomial = ...
        data.normalized_thrust_to_thrust_polynomial(:).';
    jsonStruct.thrust_to_normalized_thrust_polynomial = ...
        data.thrust_to_normalized_thrust_polynomial(:).';
    jsonStruct.normalized_thrust_to_rps_polynomial = ...
        data.normalized_thrust_to_rads_polynomial(:).';
    jsonStruct.estimated_Ct = data.estimated_Ct;

    %----------------------------------------------------------------------
    % Encode struct as JSON string
    %----------------------------------------------------------------------

    % Use pretty-printed JSON if supported by the MATLAB version
    try
        jsonStr = jsonencode(jsonStruct, PrettyPrint=true);
    catch
        % Fallback for older MATLAB versions that do not support PrettyPrint
        jsonStr = jsonencode(jsonStruct);
    end

    %----------------------------------------------------------------------
    % Write JSON file
    %----------------------------------------------------------------------

    % Output filename: output/F/F_calibration.json
    jsonFile = fullfile(motorOutDir, F + "_calibration.json");

    fid = fopen(jsonFile, 'w');
    if fid == -1
        error('writeCalibrationJson:FileOpenFailed', ...
              'Could not open file for writing: %s', jsonFile);
    end

    fwrite(fid, jsonStr, 'char');
    fclose(fid);

    fprintf('[writeCalibrationJson] Wrote calibration JSON: %s\n', jsonFile);
end