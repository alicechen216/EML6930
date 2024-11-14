% MATLAB script for consensus protocol with system dynamics-scalar
clc;clear;
% Parameters
num_agents = 5;                     % Number of agents
time_step = 0.01;                    % Time step for simulation
total_time = 10;                     % Total simulation time
num_steps = total_time / time_step;  % Number of simulation steps
K = 1;                               % Consensus gain

% System matrices
A = -1;                              % Dynamics matrix
B = 1;                               % Input matrix
C = 1;                               % Output matrix

% Initial states of agents
x = rand(num_agents, 1) * 10;        % Random initial states between 0 and 10

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
x_history = zeros(num_agents, num_steps);
x_history(:, 1) = x;

% Consensus protocol update loop
for k = 2:num_steps
    % Calculate outputs for each agent
    y = C * x;
    
    % Consensus protocol control input for each agent
    u = -K * L * y;
    
    % Update each agent's state based on system dynamics
    x_dot = A * x + B * u;
    x = x + time_step * x_dot;
    
    % Store the states for plotting
    x_history(:, k) = x;
end

% Plot results
time = linspace(0, total_time, num_steps);
figure;
plot(time, x_history','LineWidth', 2);
xlabel('Time');
ylabel('Agent States');
title('Consensus Protocol with Scalar System Dynamics','FontSize',24);
legend(arrayfun(@(i) sprintf('Agent %d', i), 1:num_agents, 'UniformOutput', false));
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