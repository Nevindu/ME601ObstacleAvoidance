function [v, phi] = lqr_controller2d(x,Y,params)
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

%% Gain Matrices
Q =  10*diag([.5 .5 1.5]);
R =  diag([1, 1]);


%% System Matrices
% Linearize around reference point
theta_d = Y.y(3);
v_d = Y.dy(1)/cos(theta_d);
phi_d =  atan( params.L*( (Y.dy(1)*Y.ddy(2) - Y.ddy(1)*Y.dy(1))/(Y.dy(1)^2 + Y.dy(2)^2)^(3/2) ) );
if isnan(phi_d)
    phi_d = 0;
end
% A = [0 0 -Y.dy(2);
%      0 0 Y.dy(1);
%      0 0 0];
% 
% B = [cos(theta_d) 0;
%      sin(theta_d) 0;
%      (1/params.L)*tan(phi_d) (v_d/params.L*cos(phi_d)*cos(phi_d))];

A = [0 0 0;
     0 0 1;
     0 0 0];

B = [1 0;
     0 0;
     0 0.5];

%% Precomputed LQR constant feedback gain matrix


K = [2.2361    0.0000    0.0000;
    -0.0000    2.2361    4.8933;]
%K = lqr(A,B,Q,R,0);

%% Error dynamics
e_x = x -Y.y;
%u_d = [v_d phi_d];
u_d = [1 0];

%% control law
u = -K*e_x + u_d;

%% Control Inputs
v = u(1);
phi = u(2);