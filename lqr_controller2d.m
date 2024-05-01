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
% Q = diag([1 1 10]);
% R = 0.1*diag([1, 1]);

Q = diag([1000 1000 10]);
R = diag([1, 1]);


%% System Matrices
% Linearize around reference point
K = zeros(2,3);

if 1 %Y.dy(1) == 0
    %% Precomputed LQR constant feedback gain matrix
    K = [2.2361    0.0000    0.0000;
        -0.0000    2.2361    4.8933;];
    A = [0 0 0;
         0 0 1;
         0 0 0];
    
    B = [1 0;
         0 0;
         0 0.5];
    K = lqr(A,B,Q,R,0);
    u_d = [1 0];
else
    theta_d = Y.y(3);
    x_d = Y.y(1);
    y_d = Y.y(2);
    x_dd = Y.dy(1);
    y_dd = Y.dy(2);
    L = 2;
    v_d = x_dd/cos(theta_d);
    phi_d =  atan( L*( (x_d*y_dd - x_dd*y_d)/(x_d^2 + y_d^2)^(3/2) ) );
    if isnan(phi_d)
        phi_d = 0;
    end
    A = [0 0 -y_d;
         0 0 x_d;
         0 0 0];
    
    
    B = [cos(theta_d) 0;
         sin(theta_d) 0;
         (1/L)*tan(phi_d) (v_d/L*cos(phi_d)*cos(phi_d))];
    K = lqr(A,B,Q,R,0);
    u_d = [v_d phi_d];
end
u_d = [ 1 0];
%% Error dynamics
%e_x = x -Y.y;
e_pos = [x(1) - Y.y(1); x(2) - Y.y(2)];

ref = Y.y(3);
act = x(3);
% if (ref > 0 && act > 0) || (ref <= 0 && act <= 0)
%     e_theta = ref - act;
% elseif ref <= 0 && act > 0
%     if abs(ref - act) < abs(2 * pi + ref - act)
%         e_theta = -abs(act - ref);
%     else
%         e_theta = abs(2 * pi + ref - act);
%     end
% else
%     if abs(ref - act) < abs(2 * pi - ref + act)
%         e_theta = abs(act - ref);
%     else
%         e_theta = -abs(2 * pi - ref + act);
%     end
% end
e_theta = x(3) - Y.y(3);
e_theta = mod(e_theta + pi/2, 2*pi/2) - pi/2;
e_x = [e_pos;e_theta];


%% control law
u = -K*e_x + u_d;

%% Control Inputs
v = u(1);
phi = u(2);

[u_d [v phi]]

% if phi > pi/4
%     phi = pi/4
% else phi < -pi/4
%      phi = -pi/4
% end
% phi