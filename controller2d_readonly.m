function [v, phi] = controller2d_readonly(t, qd, params, student_controller)
% CONTROLLER quadrotor controller
% The current states are:
%% Inputs:
%
% qd: state and desired state information for quadrotor #qn (qn
%         will be = 1 since we are only flying a single robot)
%
%  qd.pos, qd{qn}.vel   position and velocity
%  qd.euler = [roll;pitch;yaw]
%  qd{qn}.omega     angular velocity in body frame
% 
%  qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des qd{qn}.jerk_des qd{qn}.snap_des
%  desired position,
%  velocity, accel, jerk and snap
%  qd{qn}.yaw_des, qd{qn}.yawdot_des
%
% t: current time
%   
%    
% params: various parameters
%  params.L   Length of vehicle
%
%% Outputs:
%
% v: forward velocity
% phi: steering angle
% trpy: thrust, roll, pitch, yaw (attitude you want to command!)
% drpy: time derivative of trpy
%
% Using these current and desired states, you have to compute the desired
% controls u, and from there F and M
%

% populate template_controller function with vijay data
x = [qd.pos; qd.theta];
Y.y = [qd.pos_des; qd.yaw_des];
Y.dy = [qd.vel_des; qd.yawdot_des];
Y.ddy = [qd.acc_des; 0];
Y.dddy  = [qd.jerk_des; 0];
Y.ddddy = [qd.snap_des; 0];
[v, phi] = student_controller(x,Y,params);



% Output trpy and drpy as in hardware

end

%
% ------------------------------------------------------------
%    should you decide to write a geometric controller,
%    the following functions should come in handy
%

function m = eulzxy2rotmat(ang)
    phi   = ang(1);
    theta = ang(2);
    psi   = ang(3);
    
    m = [[cos(psi)*cos(theta) - sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), ...
          cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)];
         [cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta),  cos(phi)*cos(psi), ...
          sin(psi)*sin(theta) - cos(psi)*cos(theta)*sin(phi)];
         [-cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta)]];
end

function eul = rotmat2eulzxy(R)
    if R(3,2) < 1
        if R(3,2) > -1
            thetaX = asin(R(3,2));
            thetaZ = atan2(-R(1,2), R(2,2));
            thetaY = atan2(-R(3,1), R(3,3));
        else % R(3,2) == -1
            thetaX = -pi/2;
            thetaZ = -atan2(R(1,3),R(1,1));
            thetaY = 0;
        end
    else % R(3,2) == +1
        thetaX = pi/2;
        thetaZ = atan2(R(1,3),R(1,1));
        thetaY = 0;
    end
    eul = [thetaX, thetaY, thetaZ];
end

function w = veemap(R)
    w = [-R(2,3), R(1,3), -R(1,2)];
end
