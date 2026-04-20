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
% File:        processSheet.m
% Author:      Giri Mugundan Kumar
% Date:        November 29, 2024
% For info:    Andrea L'Afflitto
%              a.lafflitto@vt.edu
% 
% Description: Function reads in data sheet and processes it
%
% Github: https://github.com/girimugundankumar/acsl-physics-sim.git
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function data = processSheet(F, param)
%PROCESSHEET  Process thrust stand calibration data for a single motor.
%
%   DATA = PROCESSHEET(F, PARAM) reads thrust-stand measurements from the
%   Excel workbooks associated with motor identifier F, fits polynomial
%   models for thrust and rotor speed as functions of normalized command,
%   and generates diagnostic plots. The resulting polynomial coefficients
%   and derived quantities are returned in the struct DATA.
%
%   INPUTS:
%     F     : String or char, base filename (without .xlsx) for the main
%             thrust data workbook in data/. The RPM workbook is assumed
%             to be [F '_rpm.xlsx'] in the same folder.
%
%     PARAM : Struct containing tuning parameters and constants with fields:
%               - order_thr : polynomial order for thrust vs. command fit
%               - order_rps : polynomial order for RPS vs. command fit
%               - rho       : air density [kg/m^3]
%               - D         : propeller diameter [m]
%               - font_size : font size for plots
%
%   OUTPUT:
%     DATA  : Struct with fields
%               - normalized_thrust_to_thrust_polynomial
%               - thrust_to_normalized_thrust_polynomial
%               - normalized_thrust_to_rps_polynomial
%
%   Notes:
%     - Normalized control input ∈ [0,1] corresponds to the Pixhawk
%       normalized thrust input; 0.05–1.0 roughly spans 0–100% throttle.
%     - Each motor is calibrated using four runs (Excel sheets) as
%       described in the experimental procedure.

    %----------------------------------------------------------------------
    % Validate input Excel files and sheet count
    %----------------------------------------------------------------------

    % Base directory where the Excel files live
    dataDir = "data";  % adjust if needed

    % Main thrust data file and RPM file (with explicit path)
    thrustFile = fullfile(dataDir, F + ".xlsx");
    rpmFile    = fullfile(dataDir, F + "_rpm.xlsx");

    % Check that the thrust file exists
    if ~isfile(thrustFile)
        error('processSheet:MissingThrustFile', ...
            'Thrust data file not found: %s', thrustFile);
    end

    % Check that the RPM file exists
    if ~isfile(rpmFile)
        error('processSheet:MissingRPMFile', ...
            'RPM data file not found: %s', rpmFile);
    end

    % Check that the thrust file has exactly 4 sheets
    S = sheetnames(thrustFile);
    if numel(S) ~= 4
        error('processSheet:InvalidSheetCount', ...
            ['Thrust data file %s must contain exactly 4 sheets ', ...
             '(found %d). See calibration procedure.'], ...
            thrustFile, numel(S));
    end

    %----------------------------------------------------------------------
    % Import thrust data from Excel (4 runs per motor)
    %----------------------------------------------------------------------

    % Suppress "column headers were modified" warnings for readtable
    warning('off', 'MATLAB:table:ModifiedAndSavedVarnames');

    % Build filename and obtain sheet names
    S          = sheetnames(thrustFile);

    % Read individual runs
    P1 = readtable(thrustFile, "Sheet", S(1));
    P2 = readtable(thrustFile, "Sheet", S(2));
    P3 = readtable(thrustFile, "Sheet", S(3));
    P4 = readtable(thrustFile, "Sheet", S(4));

    %----------------------------------------------------------------------
    % Extract control input and thrust measurements
    %----------------------------------------------------------------------

    % Normalized commanded thrust (control input) [0–1]
    control_input = P1.ControlInput;

    % Thrust in grams (each run)
    Thrust_run1 = P1.CorrectedThrust_g_;
    Thrust_run2 = P2.CorrectedThrust_g_;
    Thrust_run3 = P3.CorrectedThrust_g_;
    Thrust_run4 = P4.CorrectedThrust_g_;

    % Thrust in Newtons (each run)
    Thrust_run1_N = P1.CorrectedThrust_N_;
    Thrust_run2_N = P2.CorrectedThrust_N_;
    Thrust_run3_N = P3.CorrectedThrust_N_;
    Thrust_run4_N = P4.CorrectedThrust_N_;

    % Average thrust over the four runs (g and N)
    Thrust_avg   = (Thrust_run1     ...
                    + Thrust_run2   ...
                    + Thrust_run3   ...
                    + Thrust_run4  ) / 4;
    Thrust_avg_N = (Thrust_run1_N   ...
                    + Thrust_run2_N ...
                    + Thrust_run3_N ...
                    + Thrust_run4_N) / 4;

    %----------------------------------------------------------------------
    % Forward transfer function: normalized command → thrust [N]
    %----------------------------------------------------------------------

    % Independent variable and measured response
    normalized_commanded_thrust = control_input;
    measured_thrust             = Thrust_avg_N;

    % Polynomial fit: thrust = f(command) (used in simulator)
    coeff_interp_thrust = polyfit(normalized_commanded_thrust, ...
                                  measured_thrust, ...
                                  param.order_thr);

    % Evaluate forward mapping on a dense grid (for plotting, inversion)
    control_input_vector = linspace(0.05, 1.0, 1000);
    y = polyval(coeff_interp_thrust, control_input_vector);

    % Inverse polynomial: thrust → normalized command
    % revP() must return coefficients such that:
    %   command ≈ polyval(rP, thrust)
    rP = revP(coeff_interp_thrust, control_input_vector);

    %----------------------------------------------------------------------
    % RPM / RPS data and rotor speed polynomials
    %----------------------------------------------------------------------

    % RPM vendor data file (first sheet only)
    S_RPM      = sheetnames(rpmFile);
    RPM_table  = readtable(rpmFile, "Sheet", S_RPM(1));

    % Normalized command associated with vendor RPM data
    control_input_estimated = RPM_table.ControlInput;

    % Convert RPM → RPS → rad/s
    rpm_to_rps = 1/60;                                    % RPM to RPS
    RPMs       = RPM_table.RPM;
    RPSs       = RPMs * rpm_to_rps;
    rads_p_s   = RPSs * 2*pi;

    % Polynomial fit: RPS = g(command)
    coeff_interp_rps = polyfit(control_input_estimated, ...
                               RPSs, ...
                               param.order_rps);

    % Polynomial fit: rad/s = h(command)
    coeff_interp_rads = polyfit(control_input_estimated, ...
                                rads_p_s, ...
                                param.order_rps);

    % Evaluate rotor speeds at same grid used for thrust
    estimated_rps      = polyval(coeff_interp_rps, control_input_vector);
    estimated_thrust_N = polyval(coeff_interp_thrust, control_input_vector);

    %----------------------------------------------------------------------
    % Estimate thrust coefficient Ct (dimensionless)
    %----------------------------------------------------------------------

    % Ct ≈ T / (rho * D^4 * n^2), with n [rev/s]; here we use RPS^2
    Ct = zeros(size(estimated_rps));
    for ii = 1:numel(control_input_vector)
        Ct(ii) = estimated_thrust_N(ii) / ...
            ((param.rho * param.D^4) * estimated_rps(ii)^2);
    end
    avg_Ct = mean(Ct);

    %----------------------------------------------------------------------
    % Plot experimental thrust data (g and N)
    %----------------------------------------------------------------------

    % Thrust in grams
    set(figure, 'Color', 'white');
    subplot(1, 2, 1);
    hold on;
    plot(control_input, Thrust_run1,   'Color', "red",     'LineWidth', 2, 'LineStyle', '-.');
    plot(control_input, Thrust_run2,   'Color', "blue",    'LineWidth', 2, 'LineStyle', '-.');
    plot(control_input, Thrust_run3,   'Color', "green",   'LineWidth', 2, 'LineStyle', '-.');
    plot(control_input, Thrust_run4,   'Color', "magenta", 'LineWidth', 2, 'LineStyle', '-.');
    plot(control_input, Thrust_avg,    'Color', "black",   'LineWidth', 2);
    hold off;
    title("Stable Thrust", 'Interpreter', 'latex', 'FontSize', param.font_size);
    xlabel("Thrust Input", 'Interpreter', 'latex', 'FontSize', param.font_size);
    ylabel("Thrust [g]",   'Interpreter', 'latex', 'FontSize', param.font_size);
    grid minor;
    l = legend("Run1", "Run2", "Run3", "Run4", "Avg", 'Location', "southeast");
    set(l, 'Interpreter', 'latex', 'FontSize', param.font_size);
    axis padded;
    xticks(0:0.5:1);
    set(gca, 'FontSize', param.font_size);

    % Thrust in Newtons
    subplot(1, 2, 2);
    hold on;
    plot(control_input, Thrust_run1_N, 'Color', "red",     'LineWidth', 2, 'LineStyle', '-.');
    plot(control_input, Thrust_run2_N, 'Color', "blue",    'LineWidth', 2, 'LineStyle', '-.');
    plot(control_input, Thrust_run3_N, 'Color', "green",   'LineWidth', 2, 'LineStyle', '-.');
    plot(control_input, Thrust_run4_N, 'Color', "magenta", 'LineWidth', 2, 'LineStyle', '-.');
    plot(control_input, Thrust_avg_N,  'Color', "black",   'LineWidth', 3);
    hold off;
    title("Stable Thrust", 'Interpreter', 'latex', 'FontSize', param.font_size);
    xlabel("Thrust Input", 'Interpreter', 'latex', 'FontSize', param.font_size);
    ylabel("Thrust [N]",   'Interpreter', 'latex', 'FontSize', param.font_size);
    grid minor;
    l = legend("Run1", "Run2", "Run3", "Run4", "Avg", 'Location', "southeast");
    set(l, 'Interpreter', 'latex', 'FontSize', param.font_size);
    axis padded;
    xticks(0:0.5:1);
    set(gca, 'FontSize', param.font_size);

    %----------------------------------------------------------------------
    % Plot forward transfer function (command → thrust)
    %----------------------------------------------------------------------

    set(figure, 'Color', 'white');
    hold on;
    plot(normalized_commanded_thrust, measured_thrust, '-o', 'LineWidth', 2);
    plot(control_input_vector, polyval(coeff_interp_thrust, control_input_vector), '-', 'LineWidth', 2);
    l = legend( ...
        'Average thrust', ...
        ['Interpolated average thrust - polynomial order: ', num2str(param.order_thr)], ...
        'Location', 'best');
    set(l, 'Interpreter', 'latex', 'FontSize', param.font_size);
    xlabel('Normalized Commanded Thrust [-]', 'Interpreter', 'latex', 'FontSize', param.font_size);
    ylabel('Thrust [N]',                       'Interpreter', 'latex', 'FontSize', param.font_size);
    title('Thrust test',                       'Interpreter', 'latex', 'FontSize', param.font_size);
    axis tight;
    grid on;
    set(gca, 'FontSize', param.font_size);

    %----------------------------------------------------------------------
    % Plot forward and inverse transfer functions
    %----------------------------------------------------------------------

    set(figure, 'Color', 'white');

    % Forward: command → thrust
    subplot(2, 1, 1);
    plot(control_input_vector, y, '-', 'LineWidth', 2);
    grid on;
    legend('From Polynomial', 'Interpreter', 'latex', 'FontSize', param.font_size, 'Location', 'best');
    xlim([min(control_input_vector) max(control_input_vector)]);
    xlabel('Normalized Commanded Thrust [-]', 'Interpreter', 'latex', 'FontSize', param.font_size);
    ylabel('Thrust [N]',                       'Interpreter', 'latex', 'FontSize', param.font_size);
    title('Forward transfer function -- Thrust test', 'Interpreter', 'latex', 'FontSize', param.font_size);
    set(gca, 'FontSize', param.font_size);

    % Inverse: thrust → command
    subplot(2, 1, 2);
    plot(y, control_input_vector, '--', 'LineWidth', 2);
    hold on;
    plot(sort(y), polyval(rP, sort(y)), '-', 'LineWidth', 2);
    grid on;
    xlim([min(y) max(y)]);
    legend('Exact', 'From Polynomial', 'Interpreter', 'latex', 'FontSize', param.font_size, 'Location', 'best');
    title('Reverse transfer function -- Thrust test', 'Interpreter', 'latex', 'FontSize', param.font_size);
    set(gca, 'FontSize', param.font_size);
    ylabel('Normalized Commanded Thrust [-]', 'Interpreter', 'latex', 'FontSize', param.font_size);
    xlabel('Thrust [N]',                       'Interpreter', 'latex', 'FontSize', param.font_size);

    %----------------------------------------------------------------------
    % Plot RPM / rad/s data and fitted rotor-speed model
    %----------------------------------------------------------------------

    set(figure, 'Color', 'white');
    plot(control_input_estimated, rads_p_s, '-o', 'LineWidth', 2);
    hold on;
    plot(control_input_vector, polyval(coeff_interp_rads, control_input_vector), '-', 'LineWidth', 2);
    l = legend( ...
        'Vendor rad/s', ...
        ['Interpolated average [rad/s] - polynomial order: ', num2str(param.order_rps)], ...
        'Location', 'best');
    set(l, 'Interpreter', 'latex', 'FontSize', param.font_size);
    xlabel('Normalized Commanded Thrust [-]',       'Interpreter', 'latex', 'FontSize', param.font_size);
    ylabel('Radians per second [rad/s]',            'Interpreter', 'latex', 'FontSize', param.font_size);
    title('Estimated radians per second',           'Interpreter', 'latex', 'FontSize', param.font_size);
    axis tight;
    grid on;
    set(gca, 'FontSize', param.font_size);

    %----------------------------------------------------------------------
    % Assemble output structure
    %----------------------------------------------------------------------

    data.normalized_thrust_to_thrust_polynomial = coeff_interp_thrust;
    data.thrust_to_normalized_thrust_polynomial = rP;
    data.normalized_thrust_to_rads_polynomial    = coeff_interp_rads;
    data.estimated_Ct = avg_Ct;
end