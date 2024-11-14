% MATLAB script for consensus protocol with system dynamics where Agent 1 follows a reference model
clc; clear; close all;

% Parameters
num_agents = 5;                     % Number of agents (Agent 1 is the leader + 4 followers)
time_step = 0.01;                   % Time step for simulation
total_time = 20;                    % Total simulation time
num_steps = total_time / time_step; % Number of simulation steps

% System matrices
A = [0 1 0; 0 0 1; -4 -4 -1];       % Dynamics matrix for each agent
B = [0; 0; 1];                      % Input matrix for each agent
C = [1 0 0];                        % Output matrix (observing the first state only)

% LQR gain matrices
K_leader = [0, 0, 0];               % Leader K is zero
K_follower = [0.22, 0.09, 0.19];    % Fixed LQR gains for followers

% Adjacency matrix representing connectivity between agents (including leader as Agent 1)
A_adj = [0 1 1 0 0;                  
         1 0 1 1 0;
         1 1 0 1 1;
         0 1 1 0 1;
         0 0 1 1 0];

% Degree matrix and Laplacian matrix
D = diag(sum(A_adj, 2));
L = D - A_adj;

% Store the history of states for plotting
x_history = zeros(num_agents, 3, num_steps);

% Initial states
x_ref = [1; 1; 3];                  % Initial state of the reference model for Agent 1

% Initial states of followers
x_followers = [0  0   0;
               1  2  -1;
               3  1   0;
               2  2   2];           % Each row corresponds to a follower's initial state

% Initialize history
x_history(1, :, 1) = x_ref';        % Initial state of Agent 1
x_history(2:end, :, 1) = x_followers;

% Consensus protocol update loop
for k = 2:num_steps
    % Update Agent 1 (reference model) with its own dynamics
    x_ref = x_ref + time_step * (A * x_ref);
    
    % Initialize control input for followers
    u = zeros(num_agents - 1, 1);
    
    % Calculate control inputs for each follower agent
    for i = 1:num_agents - 1
        % Use the fixed LQR gain for followers
        K_i = K_follower;
        
        % Compute the tracking error between follower i and the leader
        e_i = x_followers(i, :)' - x_ref;
        
        % Compute the consensus error between follower i and its neighbors
        consensus_error = zeros(size(x_ref));
        for j = 1:num_agents
            if A_adj(i+1, j) == 1  % If agent j is a neighbor of agent i
                if j == 1          % Agent 1 is the leader
                    x_j = x_ref;
                else
                    x_j = x_followers(j-1, :)';
                end
                consensus_error = consensus_error + (x_followers(i, :)' - x_j);
            end
        end
        
        % Compute the total error
        e_total = e_i + consensus_error;
        
        % Compute the control input for follower i
        u(i) = -K_i * e_total;
    end
    
    % Update each follower's state based on system dynamics
    x_dot_followers = (A * x_followers')' + (B * u')'; % Matrix operations to match dimensions
    x_followers = x_followers + time_step * x_dot_followers; % Update follower states
    
    % Store the states for plotting
    x_history(1, :, k) = x_ref';       % Update Agent 1's state (leader)
    x_history(2:end, :, k) = x_followers;
end

% Plot results for each component of each agent's state
time = linspace(0, total_time, num_steps);
figure;

% First component subplot
subplot(3,1,1);
% Plot Agent 1 (reference model)
plot(time, squeeze(x_history(1, 1, :)), 'k--', 'LineWidth', 3, 'DisplayName', 'Agent 1 (Reference Model)');
hold on;
% Plot the followers
colors = lines(num_agents - 1);  % Use distinct colors for followers
for i = 2:num_agents
    plot(time, squeeze(x_history(i, 1, :)), 'LineWidth', 2, 'Color', colors(i-1, :), ...
         'DisplayName', sprintf('Agent %d', i));
end
xlabel('Time');
ylabel('State Component 1');
title('State Component 1 over Time');
legend('show', 'Location', 'northeast', 'FontSize', 12);
grid on;

% Second component subplot
subplot(3,1,2);
% Plot Agent 1 (reference model)
plot(time, squeeze(x_history(1, 2, :)), 'k--', 'LineWidth', 3, 'DisplayName', 'Agent 1 (Reference Model)');
hold on;
% Plot the followers
for i = 2:num_agents
    plot(time, squeeze(x_history(i, 2, :)), 'LineWidth', 2, 'Color', colors(i-1, :), ...
         'DisplayName', sprintf('Agent %d', i));
end
xlabel('Time');
ylabel('State Component 2');
title('State Component 2 over Time');
legend('show', 'Location', 'northeast', 'FontSize', 12);
grid on;

% Third component subplot
subplot(3,1,3);
% Plot Agent 1 (reference model)
plot(time, squeeze(x_history(1, 3, :)), 'k--', 'LineWidth', 3, 'DisplayName', 'Agent 1 (Reference Model)');
hold on;
% Plot the followers
for i = 2:num_agents
    plot(time, squeeze(x_history(i, 3, :)), 'LineWidth', 2, 'Color', colors(i-1, :), ...
         'DisplayName', sprintf('Agent %d', i));
end
xlabel('Time');
ylabel('State Component 3');
title('State Component 3 over Time');
legend('show', 'Location', 'northeast', 'FontSize', 12);
grid on;

% Adjust the overall figure properties for better readability
set(gcf, 'Position', [100, 100, 800, 600]); % Resize figure window