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
% File:       ComputeYawTrajectory.m                                                   
% Author:     Giri Mugundan Kumar
% Date:       June 23, 2025
% For info:   Andrea L'Afflitto                                               
%             a.lafflitto@vt.edu                                              
%                                                                             
% Description: Computes the yaw, yaw_dot, yaw_ddot for the trajectory.
%
% GitHub: https://github.com/girimugundankumar/qrbp-unwinding-constraints.git   
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function [yaw, yaw_dot, yaw_ddot] = ComputeYawTrajectory(Vx_coef, Vy_coef, ...
                                                         Ax_coef, Ay_coef, ...
                                                         Jx_coef, Jy_coef, ...
                                                         t)
%COMPUTEYAWTRAJECTORY Computes yaw angle and derivatives from polynomial coeffs
%   Given polynomial coefficients for velocity, acceleration, and jerk,
%   this function evaluates all polynomials at time t and computes the 
%   yaw angle (heading direction) and its first and second derivatives.
%
%   Inputs:
%       Vx_coef - polynomial coefficients for x-velocity component
%       Vy_coef - polynomial coefficients for y-velocity component
%       Ax_coef - polynomial coefficients for x-acceleration component
%       Ay_coef - polynomial coefficients for y-acceleration component
%       Jx_coef - polynomial coefficients for x-jerk component
%       Jy_coef - polynomial coefficients for y-jerk component
%       t       - time value at which to evaluate the polynomials
%
%   Outputs:
%       yaw      - yaw angle (heading direction) at time t [radians]
%       yaw_dot  - yaw rate (angular velocity) at time t [rad/s]
%       yaw_ddot - yaw acceleration (angular acceleration) at time t [rad/s²]

% Yaw trajectory computation ------------------------------------------------
% Evaluate velocity components at time t
Vx = polyval(Vx_coef, t);
Vy = polyval(Vy_coef, t);
% Compute yaw angle as the direction of the velocity vector
yaw = atan2(Vy, Vx);

% Yaw Dot computation ------------------------------------------------------
% Evaluate acceleration components at time t
Ax = polyval(Ax_coef, t);
Ay = polyval(Ay_coef, t);
% Compute acceleration direction angle
acceleration_angle = atan2(Ay, Ax);
% Compute velocity and acceleration magnitudes
velocity_norm = norm2D(Vx_coef,Vy_coef,t);
acceleration_norm = norm2D(Ax_coef,Ay_coef,t);
% Compute yaw rate using the perpendicular component of acceleration
yaw_dot = (acceleration_norm/velocity_norm)*sin(acceleration_angle - yaw);

% Yaw Ddot computation -----------------------------------------------------
% Evaluate jerk components at time t
Jx = polyval(Jx_coef, t);
Jy = polyval(Jy_coef, t);
% Compute jerk direction angle and magnitude
jerk_angle = atan2(Jy, Jx);
jerk_norm = norm2D(Jx_coef,Jy_coef,t);

% Compute derivative of velocity magnitude (acceleration in direction of motion)
velocity_norm_prime = norm2Dder(Vx_coef,...
                                Vy_coef,...
                                Ax_coef,...
                                Ay_coef,...
                                t);

% Compute yaw acceleration using jerk and velocity norm derivative
yaw_ddot = (jerk_norm * sin(jerk_angle - yaw) - ...
            2*velocity_norm_prime*yaw_dot)/velocity_norm;

end
