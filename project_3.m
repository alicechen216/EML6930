% MATLAB script for consensus protocol with system dynamics
clc;clear;
% Parameters
num_agents = 5;                     % Number of agents
time_step = 0.01;                    % Time step for simulation
total_time = 20;                     % Total simulation time
num_steps = total_time / time_step;  % Number of simulation steps
K = 0.1;                               % Consensus gain

% System matrices
A = [0 1; -2 -1];                    % Dynamics matrix for each agent
B = [0; 1];                          % Input matrix for each agent
C = [1 0];                           % Output matrix (observing the first state only)

% Initial states of agents
x = rand(num_agents, 2) * 10;        % Random initial states (2D for each agent)

% Define the adjacency matrix (connectivity between agents)
A_adj = [0 1 1 0 0;                  % Adjacency matrix
         1 0 1 1 0;
         1 1 0 1 1;
         0 1 1 0 1;
         0 0 1 1 0];

% Degree matrix
D = diag(sum(A_adj, 2));

% Laplacian matrix
L = D - A_adj;

% Store the history of states for plotting
x_history = zeros(num_agents, 2, num_steps);
x_history(:, :, 1) = x;

% Consensus protocol update loop
for k = 2:num_steps
    % Calculate outputs for each agent
    y = x * C';  % Compute output y = C * x (observe only first state component)
    
    % Consensus protocol control input for each agent
    u = -K * L * y;
    
    % Update each agent's state based on system dynamics
    x_dot = (A * x')' + (B * u')';  % Matrix operation to match dimensions
    x = x + time_step * x_dot;
    
    % Store the states for plotting
    x_history(:, :, k) = x;
end

% Plot results
time = linspace(0, total_time, num_steps);
figure;
colors = lines(num_agents); % Use distinct colors for followers
for i = 1:num_agents
    plot(time, squeeze(x_history(i, 1, :)), 'LineWidth', 2, 'Color', colors(i, :), ...
         'DisplayName', sprintf('Agent %d', i )); % Plot first component of state
    hold on;
end
xlabel('Time');
ylabel('Agent States (First Component)');
title('Consensus Protocol with Linear Dynamic System','FontSize',24);
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