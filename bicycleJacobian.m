function [A, B] = bicycleJacobian(x, u)
% Jacobian of model equations for the free-flying robot example.
%
% States:
%   x(1)  x 
%   x(2)  y 
%   x(3)  theta

%
% Inputs:
%   u(1) velocity
%   u(2) steering angle

% Copyright 2018-2021 The MathWorks, Inc.

% Parameters
L = 2;

% Variables
 A = [0, 0, -u(1)*sin(x(3));    % df/dx
     0, 0, u(1)*cos(x(3));     % df/dy
     0, 0, 0];           % df/dtheta

B =    [cos(x(3)), 0;             % df/dv
     sin(x(3)), 0;             % df/dphi
     (1/L)*tan(u(2)), (u(1)/L)*sec(u(2))^2];
