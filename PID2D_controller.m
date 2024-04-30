%%%%
% ME 601 Example PID Controller
%%%%
function [v, phi] = PID2D_controller(x,Y,params)
%PID_CONTROLLER This is a LQR controller example.
% [v, phi] = LQR_CONTROLLER(x,Y) returns the control inputs.
%
% FUNCTION RETURNS
% v is the forward velocity and phi is the steering angle.
%
% FUNTION ARGUMENTS
% The control inputs are calculated as a function of x and Y. x is the
% state, and Y is a struct containing the flat output and its derivatives.
% The argument params is a struct containing information about the
% quadcopter system. See crazyflie.m for these parameter details.
% 
% x is [3x1] and contains variables 
% [x, y, theta]
% Y is a struct containing the following fields:
% Y.y     = [x; y; theta]
% Y.dy    = [dx; dy; dtehta]
% Y.ddy   = [ddx; dddy; 0]
% Y.dddy  = [dddx; dddy; 0]
% Y.ddddy = [ddddx; ddddy; 0]
%
% NOTE: This function needs to return in less than 0.01 seconds

%% PID Gains

Kpos_p = diag([.25 .5 1.5]); % proportional position gain
Kpos_d = diag([2 1 2]); % derivative position gain
Katt_p = 6*diag([20 20 .2]); % proportional attitude gain
Katt_d = 0.5*diag([20 20 1.2]); % derivative attitude gain
