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
% File:       PolyDerMatrix.m                                                   
% Author:     Giri Mugundan Kumar
% Date:       June 23, 2025
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: Function obtains the derivative of a polynomial.
%
% GitHub: https://github.com/girimugundankumar/qrbp-unwinding-constraints.git   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function poly_coeff_matrix_out = PolyDerMatrix(poly_coeff_matrix_in)
%POLYDERMATRIX Derivative of the piecewise polynomial coefficient matrix
%   Each row of poly_coeff_matrix_in contains polynomial coefficients.
%   The function returns a matrix where each row contains the coefficients
%   of the derivative polynomial.

    % Input validation
    if isempty(poly_coeff_matrix_in)
        error('Input matrix cannot be empty.');
    end

    [nRows, nCols] = size(poly_coeff_matrix_in);

    if nCols < 2
        error('Each polynomial must have at least 2 coefficients.');
    end

    % Preallocate output matrix for performance
    poly_coeff_matrix_out = zeros(nRows, nCols-1);

    % Loop through each row (polynomial)
    for i = 1:nRows
        % Use MATLAB's built-in polyder for clarity and robustness
        poly_coeff_matrix_out(i,:) = polyder(poly_coeff_matrix_in(i,:));
    end
end
