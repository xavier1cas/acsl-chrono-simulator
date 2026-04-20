function [varargout] = revP(P,d,O,Mu)
%% [rP,S,Mu] = revP(P,d,O,Mu)
% Given a set of polynomial coefficients P approximating y = f(x), this
% function approximates the polynomial coefficients of order O
% (defaults to order of P) which approximate the inverse function x = f(y).
%
% This function works similarly to LabView's:
% 'DAQmx Compute Reverse Polynomial Coefficients (VI)'
% http://zone.ni.com/reference/en-XX/help/370469AG-01/lvdaqmx/mxcomputereversecoeff/
%
% This type of functionality is commonly used in performing calibration of
% manipulated variables (e.g. DAQ card voltage) onto real system parameters
% (e.g. fan speed, current, flow rate). P then describes the forward
% transfer (function) coefficients such that evaluating polynomial described
% at P with the manipulated variable will result in the corresponding
% system parameter. P, therefore, describes the calibration curve for the
% system parameter.
% The inverse functionality comes in when one wants to know what value to
% set the manipulated variable to get a certain result. rP's are then
% the reverse transfer (function) coefficients.
%
% An example of such functionality can be found here:
% http://digital.ni.com/public.nsf/allkb/0464919A11C092E386256D800079C3BB
%
% Notably, P defines a valid transfer function only over a certain domain
% specified by 'd' (see INPUT for more details). Furthermore, if the
% transfer function defined by P is not monotonically increasing or has very
% low derivative, it may not be well invertible (See EXAMLE 3).
%
% INPUT
%  P  - Forward polynomial fit coefficients
%  d  - Domain over which to invert P.
%  O  - Order of the reverse polynomial. Defaults to order of P.
%  Mu - Two-element vector with centering and scaling parameters used in
%       evaluating the numerical transfer function.
%       See help for 'polyval' for more details.
%
% OUTPUT
%  The output of this function is analogous to that of 'polyfit'.
%  rP - Reverse polynomial fit coefficients.
%  S  - Structure S that can be used as an input to polyval to obtain error estimates
%  Mu - Two-element vector with centering and scaling values.
%
% EXAMPLE 1 - Invert a simple quadratic transfer function
%  P = [0.2 1 1];
%  x = linspace(-1,1,100);
%  y = polyval(P,x);
%  rP = revP(P,x);
%  subplot(2,1,1); plot(x,y); grid on;
%  xlim([min(x) max(x)]);
%  title('Forward transfer function')
%  subplot(2,1,2); plot(y,x,sort(y),polyval(rP,sort(y))); grid on;
%  xlim([min(y) max(y)]);
%  legend('Exact','From Polynomial'); title('Reverse transfer function')
%
% EXAMPLE 2 - Prediction bounds
%  P = [0.3 0.2 1 1];
%  x = linspace(-1,1,100);
%  y = polyval(P,x);
%  [rP,S] = revP(P,[-1,1]);
%  subplot(5,1,1:4);
%  plot(y,x); hold on; grid on;
%  y = sort(y); [xi,dxi] = polyval(rP,y,S); % Inverse x and prediction bounds
%  plot(y,xi,y,xi+dxi,'r--',y,xi-dxi,'r--'); xlim([min(y) max(y)]);
%  legend('Exact','From Polynomial','polyfit errorbounds','Location','best');
%  title('Reverse transfer function'); ylabel('x');
%  subplot(5,1,5);
%  stem(y,polyval(rP,sort(y))-x);
%  grid on; xlabel('y'); xlim([min(y) max(y)]);
%
% EXAMPLE 3 - Badly conditioned transfer function
%  P = [10 0.2 1 1]; x = linspace(-1,1,100);
%  y = polyval(P,x); [rP,S] = revP(P,[-1,1]);
%  plot(y,x); hold on; grid on;
%  y = sort(y); [xi,dxi] = polyval(rP,y,S); % Inverse x and prediction bounds
%  plot(y,xi,y,xi+dxi,'r--',y,xi-dxi,'r--');
%  xlim([min(y) max(y)]); xlabel('y'); ylabel('x');
%  legend('Exact','From Polynomial','polyfit errorbounds','Location','best');
%  title('Reverse transfer function');
% 
% EXAMPLE 4 - Draw your own transfer function
%  close all; clear; clc;
%  figure; axes; xlim([0 1]); ylim([0 10]);
%  grid on; box on; hold on; xlabel('x'); ylabel('y');
%  title('Input 5 monotonically increasing/decreasing points');
%  for j = 1:5, [x(j),y(j)] = ginput(1); plot(x,y,'ok'); end; % Grab 5 points
%  [x,I] = sort(x); y = y(I); % Sort them in increasing order
%  P = polyfit(x,y,2); % Fit the data to get sample coefficients
%  x = linspace(0,1,100); % Sample the domain {0,1} with 100 points
%  y = polyval(P,x); % Evaluate the numerical transfer function
%  plot(x,y); title('Sample transfer function x -> y');
%  rP = revP(P,[0,1],3); % Calculate the reverse coefficients
%  figure;
%  subplot(5,5,1:20); plot(y,x,sort(y),polyval(rP,sort(y))); xlim([min(y) max(y)]);
%  grid on; box on; ylabel('x');
%  legend('Exact inverse','Inverse from coefficients');
%  title('Approximate reverse transfer function y -> x');
%  subplot(5,5,21:25); stem(y,polyval(rP,sort(y))-x); xlim([min(y) max(y)]);
%  grid on; box on; xlabel('y');
%
% Written by Marcin Konowalczyk
% Timmel group @ Oxford University
 
%% Parse the input
narginchk(2,3); nargoutchk(0,3);
msgID = 'revP:InvalidInput';
switch length(d)
    case 1
        message = ['The domain input must be of the form [min max], ',...
                   '[min max N], or be an axis vector defining the ',...
                   'domain over which P is well defined.'];
        error(msgID,message);
    case {2,3}
        % 'd' is the domain in the form of [min max]
        message = 'The domain input must be of the form [min max] or [min max N]';
        assert(d(1) ~= d(2),msgID,message);
        assert(d(1) < d(2),msgID,message);
        if length(d) == 2, d(end+1) = 10e3; end; % Default to 10k points if none given
        assert(isequal(d(3),fix(d(3))),msgID,message);
        x = linspace(d(1),d(2),d(3));
    otherwise
        % 'd' is the axis vector for which P is defined
        x = d;
        assert(isnumeric(x),msgID,'Input vector must be numeric');
        assert(isequal(x,sort(x)),msgID,'Input axis vector must be monotonically increasing');
end
 
if nargin < 3 || isempty(O);
    O = length(P)-1; % Default O to be the order of P
end
 
if nargin < 4 || isempty(Mu);
    Mu = [0 1]; % Set the centering and scaling parameters to have no effect
end
 
%% Calculate the numerical transfer function y = f(x)
y = polyval(P,x,[],Mu); % Evaluate the polynomial at x
 
%% Fit a the reverse transfer function x = f(y)
switch nargout
    case {0, 1}
        rP = polyfit(y,x,O);
        varargout = {rP};
    case 2
        [rP,S] = polyfit(y,x,O);
        varargout = {rP,S};
    case 3
        [rP,S,Mu] = polyfit(y,x,O);
        varargout = {rP,S,Mu};
end
end