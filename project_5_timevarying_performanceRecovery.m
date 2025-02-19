% MATLAB script for consensus protocol with system dynamics where Agent 1 follows a reference model
clc; clear; close all;

% Parameters
num_agents = 5;                     % Number of agents (Agent 1 is the leader + 4 followers)
time_step = 0.01;                   % Time step for simulation
total_time = 20;                    % Total simulation time
num_steps = total_time / time_step; % Number of simulation steps

% Time vector
time = linspace(0, total_time, num_steps);

% System matrices
A = [0 1.5; -2 0];       % Dynamics matrix for each agent (2x2)
B = [0; 1];              % Input matrix for each agent (2x1)
C = [1 0];               % Output matrix (observing the first state only)

% LQR cost matrices
Q = 0.1 * eye(2);        % State-cost matrix
R = 0.01;                % Input-cost matrix

% Adjacency matrix representing connectivity between agents (including leader as Agent 1)
A_adj = [0 1 1 0 0;
         1 0 1 1 0;
         1 1 0 1 1;
         0 1 1 0 1;
         0 0 1 1 0];

% Degree matrix and Laplacian matrix
D = diag(sum(A_adj, 2));
L = D - A_adj;

% Compute the eigenvalues of the Laplacian matrix
lambda = eig(L);

% Calculate individual LQR gain matrices for each follower based on Laplacian eigenvalues
K_array = zeros(num_agents - 1, 2); % LQR gains for each follower (row per agent)
for i = 2:num_agents                % Skip lambda_1 = 0 (consensus mode)
    lambda_i = sqrt(lambda(i));
    K_array(i - 1, :) = lqr(A, lambda_i * B, Q, R); % Compute LQR gain for each follower
end

% Store the history of states for plotting
x_history = zeros(num_agents, 2, num_steps);

% Initial states
x_ref = [1; 1];                  % Initial state of the reference model for Agent 1

% Initial states of followers
x_followers = [0 0;
               1 2;
               3 1;
               2 2];             % Each row corresponds to a follower's initial state

% Initialize history
x_history(1, :, 1) = x_ref';     % Initial state of Agent 1
x_history(2:end, :, 1) = x_followers;

% Define the uncertainties for each follower agent
delta_funcs = {
    @(t) (2 + sin(0.6 * t)) * [0.1; 0.2];    % delta_1(t)
    @(t) (-1 + sin(0.85 * t)) * [0.2; 0.4];  % delta_2(t)
    @(t) (-4 + sin(0.25 * t)) * [0.3; 0.6];  % delta_3(t)
    @(t) (1 + sin(0.25 * t)) * [0.4; 0.8];   % delta_4(t)
};

% Initialize uncertainty estimates and adaptive gains
delta_hat = zeros(num_agents - 1, 1);      % Estimated uncertainties for each follower (scalar)
gamma = ones(num_agents - 1, 1) * 1.0;     % Adaptation gains
delta_bound = ones(num_agents - 1, 1) * 5; % Bounds for the projection operator

% Consensus protocol update loop
for k = 2:num_steps
    t = time(k); % Current time
    
    % Update Agent 1 (reference model) with its own dynamics
    x_ref = x_ref + time_step * (A * x_ref);
    
    % Initialize control input for followers
    u = zeros(num_agents - 1, 1);
    
    % Calculate control inputs and uncertainties for each follower agent
    for i = 1:num_agents - 1
        % Extract the LQR gain for the current follower
        K_i = K_array(i, :);
        
        % Compute the tracking error between follower i and the leader
        e_i = x_followers(i, :)' - x_ref;
        
        % Compute the consensus error between follower i and its neighbors
        consensus_error = zeros(size(x_ref));
        for j = 1:num_agents
            if A_adj(i + 1, j) == 1  % If agent j is a neighbor of agent i
                if j == 1            % Agent 1 is the leader
                    x_j = x_ref;
                else
                    x_j = x_followers(j - 1, :)';
                end
                consensus_error = consensus_error + (x_followers(i, :)' - x_j);
            end
        end
        
        % Compute the total error
        e_total = e_i + consensus_error;
        
        % Update the uncertainty estimate using projection operator
        delta_hat(i) = delta_hat(i) + time_step * (-gamma(i) * e_i(2));
        % Apply projection to keep the estimate within bounds
        if delta_hat(i) > delta_bound(i)
            delta_hat(i) = delta_bound(i);
        elseif delta_hat(i) < -delta_bound(i)
            delta_hat(i) = -delta_bound(i);
        end
        
        % Compute the control input for follower i, compensating for the estimated uncertainty
        u(i) = -K_i * e_total - delta_hat(i);
    end
    
    % Compute actual uncertainties for each follower agent
    deltas = zeros(num_agents - 1, 2);
    for i = 1:num_agents - 1
        deltas(i, :) = delta_funcs{i}(t)';
        % Zero out the first component to focus on the second state
        deltas(i, 1) = 0;
    end
    
    % Update each follower's state based on system dynamics and uncertainties
    x_dot_followers = (A * x_followers')' + (B * u')' + deltas; % Include uncertainties
    x_followers = x_followers + time_step * x_dot_followers;     % Update follower states
    
    % Store the states for plotting
    x_history(1, :, k) = x_ref';       % Update Agent 1's state (leader)
    x_history(2:end, :, k) = x_followers;
end

% Plot results for each component of each agent's state
figure;
subplot(2, 1, 1);
% Plot Agent 1 (reference model)
plot(time, squeeze(x_history(1, 1, :)), 'k--', 'LineWidth', 3, 'DisplayName', 'Agent 1 (Reference Model)');
hold on;
% Plot the followers
colors = lines(num_agents - 1);  % Use distinct colors for followers
for i = 2:num_agents
    plot(time, squeeze(x_history(i, 1, :)), 'LineWidth', 2, 'Color', colors(i - 1, :), ...
         'DisplayName', sprintf('Agent %d', i));
end
xlabel('Time');
ylabel('State Component 1');
title('State Component 1 over Time');
legend('show', 'Location', 'northeast', 'FontSize', 12);
grid on;

subplot(2, 1, 2);
% Plot Agent 1 (reference model)
plot(time, squeeze(x_history(1, 2, :)), 'k--', 'LineWidth', 3, 'DisplayName', 'Agent 1 (Reference Model)');
hold on;
% Plot the followers
for i = 2:num_agents
    plot(time, squeeze(x_history(i, 2, :)), 'LineWidth', 2, 'Color', colors(i - 1, :), ...
         'DisplayName', sprintf('Agent %d', i));
end
xlabel('Time');
ylabel('State Component 2');
title('State Component 2 over Time');
legend('show', 'Location', 'northeast', 'FontSize', 12);
grid on;

% Adjust the overall figure properties for better readability
set(gcf, 'Position', [100, 100, 800, 600]); % Resize figure window

% Add 3D plot for the system trajectories
figure;
hold on;
% Plot Agent 1 (reference model) trajectory
plot3(squeeze(x_history(1, 1, :)), squeeze(x_history(1, 2, :)), time, 'k--', 'LineWidth', 3, 'DisplayName', 'Agent 1 (Reference Model)');
% Plot the followers' trajectories
for i = 2:num_agents
    plot3(squeeze(x_history(i, 1, :)), squeeze(x_history(i, 2, :)), time, 'LineWidth', 2, 'Color', colors(i - 1, :), ...
         'DisplayName', sprintf('Agent %d', i));
end
xlabel('State Component 1');
ylabel('State Component 2');
zlabel('Time');
title('3D Trajectories of Agents over Time');
legend('show', 'Location', 'northeast', 'FontSize', 12);
grid on;
view(3); % Adjust the view angle for better visualization