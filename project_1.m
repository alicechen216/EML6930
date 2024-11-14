% MATLAB script for consensus protocol with 5 agents
clc;clear;
% Parameters
num_agents = 5;                    % Number of agents
time_step = 0.01;                   % Time step for simulation
total_time = 10;                    % Total simulation time
num_steps = total_time / time_step; % Number of simulation steps

% Initial states of agents
x = rand(num_agents, 1) * 10;       % Random initial states between 0 and 10

% Define the adjacency matrix (connectivity between agents)
A = [0 1 1 0 0;                     % Adjacency matrix
     1 0 1 1 0;
     1 1 0 1 1;
     0 1 1 0 1;
     0 0 1 1 0];

% Degree matrix
D = diag(sum(A, 2));

% Laplacian matrix
L = D - A;

% Store the history of states for plotting
x_history = zeros(num_agents, num_steps);
x_history(:, 1) = x;

% Consensus protocol update loop
for k = 2:num_steps
    % Update rule: x_dot = -L * x
    x_dot = -L * x;
    
    % Euler integration to update the states
    x = x + time_step * x_dot;
    
    % Store the states for plotting
    x_history(:, k) = x;
end

% Plot results
time = linspace(0, total_time, num_steps);
figure;
plot(time, x_history','LineWidth', 2);

% Axis labels and title with larger font size
xlabel('Time','FontSize', 24);
ylabel('Agent States','FontSize', 24);
title('Consensus Protocol: Agent States Converging to Consensus','FontSize', 24);

% Customize the legend
legend(arrayfun(@(i) sprintf('Agent %d', i), 1:num_agents, 'UniformOutput', false));
lgd = legend;
lgd.FontSize = 20;            % Increase font size of the legend
lgd.Location = 'northeast';    % Place legend in the top right corner
lgd.Box = 'on';                % Add a box around the legend for clarity

% Add grid
grid on;

% Customize axis tick labels for clarity
ax = gca; % Get the current axis
ax.FontSize = 18; % Increase font size of tick labels
ax.XColor = [0.1, 0.1, 0.1]; % Darken X-axis tick labels
ax.YColor = [0.1, 0.1, 0.1]; % Darken Y-axis tick labels
ax.LineWidth = 1.5; % Increase axis line width for better clarity