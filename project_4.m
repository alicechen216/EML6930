% MATLAB script for consensus protocol with system dynamics and a reference model
clc;clear;
% Parameters
num_agents = 5;                     % Number of agents (1 leader + 4 followers)
time_step = 0.01;                    % Time step for simulation
total_time = 20;                     % Total simulation time
num_steps = total_time / time_step;  % Number of simulation steps
K = 1;                               % Consensus gain

% System matrices
A = [0 1; -2 -1];                    % Dynamics matrix for each agent
B = [0; 1];                          % Input matrix for each agent
C = [1 0];                           % Output matrix (observing the first state only)

% Initial states of agents and reference model
x_ref = [5; 0];                      % Initial state of the reference model (leader)
x = rand(num_agents - 1, 2) * 10;    % Random initial states for followers (2D for each agent)


% Define the adjacency matrix (connectivity between followers)
A_adj = [0 1 1 0;                    % Adjacency matrix for followers only
         1 0 1 1;
         1 1 0 1;
         0 1 1 0];


% Degree matrix and Laplacian matrix for followers
D = diag(sum(A_adj, 2));
L = D - A_adj;

% Store the history of states for plotting
x_history = zeros(num_agents, 2, num_steps);
x_history(1, :, 1) = x_ref';         % Initial state of the reference model
x_history(2:end, :, 1) = x;          % Initial states of the followers

% Consensus protocol update loop
for k = 2:num_steps
    % Update the leader's state according to its own dynamics (reference model)
    x_ref_dot = A * x_ref;
    x_ref = x_ref + time_step * x_ref_dot;
    
    % Calculate the output of the leader
    y_ref = C * x_ref;
    
    % Calculate outputs for each follower agent
    y = x * C';  % Compute output y = C * x for followers only
    
    % Consensus protocol control input for each follower agent
    u = -K * (y_ref - y) - K * L * y; % Modified control: track leader + neighbor influence
    
    % Update each follower's state based on system dynamics
    x_dot = (A * x')' + (B * u')';  % Matrix operations to match dimensions
    x = x + time_step * x_dot;       % Update follower states
    
    % Store the states for plotting
    x_history(1, :, k) = x_ref';     % Include the leader's state as the first row
    x_history(2:end, :, k) = x;      % Include follower states
end

% Plot results for the first component of each agent's state
time = linspace(0, total_time, num_steps);
figure;

% Plot the leader (reference model) with a distinct style and thicker line
plot(time, squeeze(x_history(1, 1, :)), 'k--', 'LineWidth', 3, 'DisplayName', 'Leader (Reference Model)');
hold on;

% Plot the followers with thicker lines and markers for clarity
colors = lines(num_agents - 1); % Use distinct colors for followers
for i = 2:num_agents
    plot(time, squeeze(x_history(i, 1, :)), 'LineWidth', 2, 'Color', colors(i-1, :), ...
         'DisplayName', sprintf('Agent %d', i - 1));
end

% Add labels, title, and legend
xlabel('Time');
ylabel('Agent States (First Component)');
title('Consensus Protocol with Reference Model','FontSize',24);
% Adjust the legend for better readability
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