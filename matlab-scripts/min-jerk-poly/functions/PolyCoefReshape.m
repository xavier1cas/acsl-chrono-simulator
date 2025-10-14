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
% File:       PolyCoefReshape.m                                                   
% Author:     Giri Mugundan Kumar
% Date:       June 23, 2025
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: Function rearragnes the polynomial coefficient.
%
% GitHub: https://github.com/girimugundankumar/qrbp-unwinding-constraints.git   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [poly_coeff_matrix_out_x, ...
          poly_coeff_matrix_out_y, ...
          poly_coeff_matrix_out_z] = PolyCoefReshape(poly_coeff_matrix_in)
%POLYCOEFASSIGNING Deinterleaves polynomial coefficients for 3D trajectory
%   Takes an input matrix where polynomial coefficients for X, Y, Z coordinates
%   are stored in interleaved rows (X1, Y1, Z1, X2, Y2, Z2, ...) and separates
%   them into three coordinate-specific matrices.
%
%   Inputs:
%       poly_coeff_matrix_in - Matrix of size (3*n, m) where:
%                              - Rows 1,4,7,... contain X coefficients
%                              - Rows 2,5,8,... contain Y coefficients  
%                              - Rows 3,6,9,... contain Z coefficients
%
%   Outputs:
%       poly_coeff_matrix_out_x - Matrix (n, m) containing X coefficients
%       poly_coeff_matrix_out_y - Matrix (n, m) containing Y coefficients
%       poly_coeff_matrix_out_z - Matrix (n, m) containing Z coefficients

    % Input validation
    [rows, ~] = size(poly_coeff_matrix_in);
    
    if mod(rows, 3) ~= 0
        error('Input matrix must have a number of rows divisible by 3');
    end
    
    if rows == 0
        error('Input matrix cannot be empty');
    end
    
    % Vectorized extraction using direct indexing (most efficient)
    poly_coeff_matrix_out_x = poly_coeff_matrix_in(1:3:end, :);
    poly_coeff_matrix_out_y = poly_coeff_matrix_in(2:3:end, :);
    poly_coeff_matrix_out_z = poly_coeff_matrix_in(3:3:end, :);
end
