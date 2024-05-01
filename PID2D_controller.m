function PID2D_controller()
    close all;
    clear all;
    % Load reference matrix M
    % M = [t, x, y, theta, dx, dy, dtheta]; % Assume this is loaded or passed
    M = readmatrix('traj2d.csv'); % Replace with actual loading method

    % Bicycle parameters
    L = 2;  % Wheelbase (m)

    % PID parameters for velocity
    Kp_v = 1;   % Proportional gain
    Ki_v = 0.1; % Integral gain
    Kd_v = 0.05; % Derivative gain

    % PID parameters for heading
    Kp_theta = 1;
    Ki_theta = 0.1;
    Kd_theta = 0.05;

    % State initialization
    x = M(1, 2);
    y = M(1, 3);
    theta = M(1, 4);
    v = sqrt(M(1, 5)^2 + M(1, 6)^2); % Initial velocity from dx, dy

    % Error initialization for integral calculations
    integral_v = 0;
    integral_theta = 0;
    prev_error_v = 0;
    prev_error_theta = 0;

    % Time parameters
    dt = 0.01;  % Time step (s)
    T = M(end, 1);  % Total simulation time (s) from reference data
    time = 0:dt:T;  % Time array

    % Setup figure for live plotting of trajectory
    figure('Name', 'Real-time Simulation Results', 'NumberTitle', 'off')
    hold on;
    grid on;
    xlabel('X Position');
    ylabel('Y Position');
    hRefTraj = plot(M(:,2), M(:,3), 'b--', 'LineWidth', 2);
    hTraj = plot(x, y, 'r', 'LineWidth', 2);
    hMarker = plot(x, y, 'ko', 'MarkerFaceColor', 'k'); % Marker for the current position
    legend('Reference Trajectory', 'Controller Trajectory');
    title('Trajectories');

    % Arrays for storing data for post-simulation plotting
    simTime = [];
    velocities = [];
    headings = [];
    vRef = [];
    thetaRef = [];

    % Simulation loop
    for t = time
        % Find row of the desired state at time t
        tr = round(t, 2); % round given time t to closest multiple of 0.01
        row = int64(tr / 0.01) + 1;

        % Ensure the row does not exceed matrix dimensions
        if row > size(M, 1)
            row = size(M, 1);
        end

        v_ref = sqrt(M(row, 5)^2 + M(row, 6)^2); % Reference velocity
        theta_ref = M(row, 4); % Reference heading

        % Velocity PID Controller
        error_v = v_ref - v;
        integral_v = integral_v + error_v * dt;
        derivative_v = (error_v - prev_error_v) / dt;
        u_v = Kp_v * error_v + Ki_v * integral_v + Kd_v * derivative_v;

        % Heading PID Controller
        error_theta = wrapToPi(theta_ref - theta);
        integral_theta = integral_theta + error_theta * dt;
        derivative_theta = (error_theta - prev_error_theta) / dt;
        u_theta = Kp_theta * error_theta + Ki_theta * integral_theta + Kd_theta * derivative_theta;

        % Update the state
        v = v + u_v * dt; % Simple integrator for velocity control
        delta = atan2(L * u_theta, v); % Compute steering angle from heading control output
        x = x + v * cos(theta) * dt;
        y = y + v * sin(theta) * dt;
        theta = theta + (v / L) * tan(delta) * dt;

        % Update live trajectory plot
        set(hTraj, 'XData', [get(hTraj, 'XData'), x], 'YData', [get(hTraj, 'YData'), y]);
        set(hMarker, 'XData', x, 'YData', y);
        title(sprintf('Trajectories at t = %.2fs', t));
        drawnow;

        % Store data for post-simulation analysis
        simTime = [simTime, t];
        velocities = [velocities, v];
        headings = [headings, theta];
        vRef = [vRef, v_ref];
        thetaRef = [thetaRef, theta_ref];

        % Update previous errors
        prev_error_v = error_v;
        prev_error_theta = error_theta;
    end

    % Plot velocity and heading after simulation
    figure;
    subplot(2, 1, 1);
    plot(simTime, velocities, 'r', simTime, vRef, 'b--');
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    legend('Actual Velocity', 'Desired Velocity');
    title('Velocity Tracking');

    subplot(2, 1, 2);
    plot(simTime, headings, 'r', simTime, thetaRef, 'b--');
    xlabel('Time (s)');
    ylabel('Heading (radians)');
    legend('Actual Heading', 'Desired Heading');
    title('Heading Tracking');
end

function eWrapped = wrapToPi(e)
    % Wrap angle error to [-pi, pi]
    eWrapped = mod(e + pi, 2*pi) - pi;
end
