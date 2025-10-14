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
% File:       norm2Dder.m                                                   
% Author:     Giri Mugundan Kumar
% Date:       June 23, 2025
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: Computes the derivative of the 2D norm from polynomial 
%              coefficients of the components of the vector.
%
% GitHub: https://github.com/girimugundankumar/qrbp-unwinding-constraints.git   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%


function derivative_value = norm2Dder(poly_coef_x,...
                                      poly_coef_y,...
                                      poly_coef_x_prime,...
                                      poly_coef_y_prime,...
                                      t)
%NORM2DDERIVATIVE Computes the derivative of the 2D norm from polynomial
%coefficients of the components of the vector
%   Given polynomial coefficients for x and y components, their derivatives,
%   and a time value t, this function evaluates all polynomials at time t
%   and computes the time derivative of the 2D Euclidean norm (magnitude)
%   of the vector [x(t), y(t)].
%
%   Inputs:
%       poly_coef_x       - polynomial coefficients for x-component (1D array)
%       poly_coef_y       - polynomial coefficients for y-component (1D array)
%       poly_coef_x_prime - polynomial coefficients for x-component derivative
%       poly_coef_y_prime - polynomial coefficients for y-component derivative
%       t                 - time value at which to evaluate the polynomials
%
%   Outputs:
%       derivative_value  - time derivative of the 2D Euclidean norm at time t
%
%   The formula used is:
%       d/dt ||v(t)|| = (x(t)*x'(t) + y(t)*y'(t)) / ||v(t)||
%   where v(t) = [x(t), y(t)], x'(t) and y'(t) are derivatives.

% Evaluate x(t), x'(t), y(t), y'(t) at time t
% Compute numerator: x(t)*x'(t) + y(t)*y'(t)
% Compute denominator: Euclidean norm sqrt(x(t)^2 + y(t)^2)
% Return the derivative of the norm at time t
derivative_value = (polyval(poly_coef_x,t)*polyval(poly_coef_x_prime,t) +...
                    polyval(poly_coef_y,t)*polyval(poly_coef_y_prime,t)) /...
                    norm2D(poly_coef_x,poly_coef_y,t);

end
