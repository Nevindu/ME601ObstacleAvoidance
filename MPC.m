% Parameters
L = 2;                % Wheelbase
deltaT = 0.01;           % Sampling time

% Load trajectory data
data = readmatrix('traj2d.csv');
t = data(:,1);             % Time
refX = data(:,2);          % Reference x
refY = data(:,3);          % Reference y
refTheta = data(:,4);  % Reference theta

% Define the prediction horizon
predictionHorizon = 10;

% Create the nonlinear MPC controller
nlobj = nlmpc(3, 3, 2);

nlobj.ControlHorizon = 20;
nlobj.PredictionHorizon = 20;
nlobj.Ts = 0.01;

% Specify the prediction model
nlobj.Model.StateFcn = @(x, u) [
    u(1) * cos(x(3));
    u(1) * sin(x(3));
    (u(1) / L) * tan(u(2));
];

nlobj.Jacobian.StateFcn = "bicycleJacobian";

% No output function needed since all states are measured
%nlobj.Model.IsContinuousTime = false;

% Define the control inputs (velocity and steering angle)
nlobj.MV(1).Min = -5;   % Minimum velocity
nlobj.MV(1).Max = 5;    % Maximum velocity
nlobj.MV(2).Min = -pi/4; % Minimum steering angle
nlobj.MV(2).Max = pi/4;  % Maximum steering angle

% Define the Weights on states and inputs
%nlobj.Weights.OutputVariables = [50 50 0];
nlobj.Weights.OutputVariables = [1 1 0.1];
nlobj.Weights.ManipulatedVariables = [0.1 0.1];
nlobj.Weights.ManipulatedVariablesRate = [1 1];



% Setup the solver options
%nlobj.Optimization.SolverOptions.Algorithm = 'sqp';

% Initial condition
x0 = [refX(1); refY(1); refTheta(1)];  % Start at the first point of the trajectory
u0 = [0; 0];     % Initial input [v phi]

% Set up the simulation
xHistory = x0';

mv = [1,0];
T = 10;
time = 0:deltaT:T;
for t  = time
 % Find row of the desired state at time t
    tr = round(t, 2); % round given time t to closest multiple of 0.01
    row = int64(tr / 0.01) + 1;

    % Ensure the row does not exceed matrix dimensions
    if row > size(data, 1)
        row = size(data, 1);
    end
    % Update the reference for the current time step
    %nlobj.Model.Reference = [refX(k) refY(k) refTheta(k)];
    %nlobj.Model.Reference = data(k,2:4); 
    % Solve the MPC optimization problem
    ref = data(row,2:4);
    [mv, info] = nlmpcmove(nlobj, x0, mv, ref);
    mv;
    % Update the state
    x0 = nlobj.Model.StateFcn(x0, mv);
    %u0 = mv;
    
    % Store the history
    xHistory = [xHistory; x0'];
end

% Plot the results
figure;
plot(xHistory(:,1), xHistory(:,2), 'b-', refX, refY, 'r--');
xlabel('X');
ylabel('Y');
title('Vehicle Trajectory');
legend('Actual', 'Reference');
