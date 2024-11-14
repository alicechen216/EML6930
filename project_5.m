% MATLAB script for consensus protocol with system dynamics where Agent 1 follows a reference model
clc; clear;

% Parameters
num_agents = 5;                     % Number of agents (Agent 1 is the leader + 4 followers)
time_step = 0.01;                    % Time step for simulation
total_time = 20;                     % Total simulation time
num_steps = total_time / time_step;  % Number of simulation steps

% System matrices
A = [0 1; -2 -1];                    % Dynamics matrix for each agent
B = [0; 1];                          % Input matrix for each agent
C = [1 0];                           % Output matrix (observing the first state only)

% LQR cost matrices (you can tune Q and R for desired performance)
Q = eye(2);                          % State-cost matrix
R = 0.01;                             % Input-cost matrix

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
K_array = zeros(num_agents - 1, 2);  % LQR gains for each follower (row per agent)
for i = 2:num_agents                 % Skip lambda_1 = 0 (as it corresponds to the consensus mode)
    lambda_i = sqrt(lambda(i));
    K_array(i-1, :) = lqr(A, lambda_i * B, Q, R); % Compute LQR gain for each follower
end

% Store the history of states for plotting
x_history = zeros(num_agents, 2, num_steps);

% Initial states
x_ref = [sin(0); cos(0)];            % Initial state of the reference model for Agent 1
x_followers = repmat(x_ref', num_agents - 1, 1) + [2, 1; 2, 0; 1, 2; 0, 2]; % Perturbed initial states for followers

% Initialize history
x_history(1, :, 1) = x_ref';         % Initial state of Agent 1
x_history(2:end, :, 1) = x_followers; % Initial states of followers

% Consensus protocol update loop
for k = 2:num_steps
    % Update Agent 1 (reference model) with a predefined sinusoidal trajectory
    t = (k - 1) * time_step;
    x_ref = [sin(t); cos(t)];  % Agent 1 follows this trajectory directly
    
    % Calculate the output of Agent 1
    y_ref = C * x_ref;
    
    % Calculate outputs for each follower agent
    y_followers = x_followers * C';  % Compute output y = C * x for followers only

    % Initialize control input for followers
    u = zeros(num_agents - 1, 1);
    
    % Calculate control inputs for each follower agent
    for i = 1:num_agents - 1
        % Extract the LQR gain for the current follower
        K_i = K_array(i, :);
        
        % Reference tracking term (make follower i track Agent 1's state)
        ref_tracking = K_i(1) * (y_ref - y_followers(i));
        
        % Consensus term (neighbor influence) - uses neighbor states
        y_diff = (y_followers' - y_followers(i))';  % Differences with neighbors
        neighbor_influence = K_i(2) * sum(L(i+1, :) .* y_diff, 'all');  % Use row i+1 to match the followers
        
        % Compute the control input for follower i
        u(i) = ref_tracking - neighbor_influence;
    end
    
    % Update each follower's state based on system dynamics
    x_dot_followers = (A * x_followers')' + (B * u')';  % Matrix operations to match dimensions
    x_followers = x_followers + time_step * x_dot_followers;  % Update follower states
    
    % Store the states for plotting
    x_history(1, :, k) = x_ref';       % Update Agent 1's state (leader)
    x_history(2:end, :, k) = x_followers;  % Update follower states
end

% Plot results for the first component of each agent's state
time = linspace(0, total_time, num_steps);
figure;

% Plot Agent 1 (reference model) with a distinct style and thicker line
plot(time, squeeze(x_history(1, 1, :)), 'k--', 'LineWidth', 3, 'DisplayName', 'Agent 1 (Reference Model)');
hold on;

% Plot the followers with thicker lines and markers for clarity
colors = lines(num_agents - 1);  % Use distinct colors for followers
for i = 2:num_agents
    plot(time, squeeze(x_history(i, 1, :)), 'LineWidth', 2, 'Color', colors(i-1, :), ...
         'DisplayName', sprintf('Agent %d', i));
end

% Add labels, title, and legend
xlabel('Time');
ylabel('Agent States (First Component)');
title('Consensus Protocol with Sinusoidal Reference Model and Different Gains','FontSize',24);
lgd = legend;
lgd.FontSize = 20;            % Increase font size of the legend
lgd.Location = 'northeast';    % Place legend in the top right corner
lgd.Box = 'on';                % Add a box around the legend for clarity

grid on;
% Customize axis tick labels for clarity
ax = gca; % Get the current axis
ax.FontSize = 18; % Increase font size of tick labels
ax.XColor = [0.1, 0.1, 0.1]; % Darken X-axis tick labels
ax.YColor = [0.1, 0.1, 0.1]; % Darken Y-axis tick labels
ax.LineWidth = 1.5; % Increase axis line width for better clarity