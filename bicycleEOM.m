function sdot = bicycleEOM(t, s, controlhandle, trajhandle, params, student_controller)
% QUADEOM Wrapper function for solving quadrotor equation of motion
% 	quadEOM takes in time, state vector, controller, trajectory generator
% 	and parameters and output the derivative of the state vector, the
% 	actual calcution is done in quadEOM_readonly.
%
% INPUTS:
% t             - 1 x 1, time
% s             - 3 x 1, state vector = [x, y, theta]
% controlhandle - function handle of your controller
% trajhandle    - function handle of your trajectory generator
% params        - struct, output from crazyflie() and whatever parameters you want to pass in
%
% OUTPUTS:
% sdot          - 3 x 1, derivative of state vector s
%
% NOTE: You should not modify this function
% See Also: quadEOM_readonly, crazyflie

% convert state to quad stuct for control
qd.pos = s(1:2);
qd.theta = s(3);

% Get desired_state
desired_state = trajhandle(t);

% The desired_state is set in the trajectory generator
qd.pos_des      = desired_state.y(1:2);
qd.vel_des      = desired_state.dy(1:2);
qd.acc_des      = desired_state.ddy(1:2);
qd.jerk_des     = desired_state.dddy(1:2);
qd.snap_des     = desired_state.ddddy(1:2);
qd.yaw_des      = desired_state.y(3);
qd.yawdot_des   = desired_state.dy(3);

% get control outputs
[v, phi] = controlhandle(t, qd, params, student_controller);

% compute derivative
kinematicModel = bicycleKinematics; 
kinematicModel.WheelBase = params.L;
sdot = derivative(kinematicModel, s, [v phi]);
%sdot = quadEOM_readonly(t, s, F, M, params);

end
