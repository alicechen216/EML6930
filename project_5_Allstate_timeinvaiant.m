% MATLAB script implementing robust control law with uncertainties
clc; clear;close all

% Parameters
num_agents = 5;                     % Number of agents (Agent 1 is the leader + 4 followers)
time_step = 0.01;                   % Time step for simulation
total_time = 20;                    % Total simulation time
num_steps = total_time / time_step; % Number of simulation steps

% System matrices
A = [0 1 0; 0 0 1; -4 -4 -1];       % Dynamics matrix for each agent
B = [0; 0; 1];                      % Input matrix for each agent
C = [1 0 0];                        % Output matrix (observing the first state only)

% LQR cost matrices
Q = 0.1*eye(3);                         % State-cost matrix
R = 0.01;                           % Input-cost matrix

% LQR gains for each follower (from original code)
% Compute the eigenvalues of the Laplacian matrix
A_adj = [0 1 1 0 0;                  
         1 0 1 1 0;
         1 1 0 1 1;
         0 1 1 0 1;
         0 0 1 1 0];
D = diag(sum(A_adj, 2));
L = D - A_adj;
lambda = eig(L);

K_array = zeros(num_agents - 1, 3); % LQR gains for each follower (row per agent)
for i = 2:num_agents                % Skip lambda_1 = 0 (consensus mode)
    lambda_i = sqrt(lambda(i));
    K_array(i-1, :) = lqr(A, lambda_i * B, Q, R); % Compute LQR gain for each follower
end

% Robust control parameters
gamma = 100;                          % Assumed gamma = 1 (since not specified)
c = 6;
mu = 0.2;
P = [0.0850, 0.0316, 0.0224;
     0.0316, 0.0467, 0.0090;
     0.0224, 0.0090, 0.0187];
M = gamma * A' * P + mu * eye(3);   % Compute M matrix

% l_i values for followers (l_i = 1 if connected to leader)
l_i = [A_adj(2,1); A_adj(3,1); A_adj(4,1); A_adj(5,1)]; % l_i for followers 1 to 4

% Time-invariant uncertainties (delta_i) for each follower agent
delta_followers = [5,  -7,  -3;
                   4,  -5,  -4;
                   6,   3,  -5;
                  -4,   1,   2];     % Each row corresponds to a follower's delta_i

% Initial states
x_ref = [1; 1; 3];                  % Initial state of the reference model for Agent 1

% Initial states of followers
x_followers = [0  0   0;
               1  2  -1;
               3  1   0;
               2  2   2];           % Each row corresponds to a follower's initial state

% Initialize estimated states for followers
hat_x_followers = x_followers;      % Initial estimated states same as initial states

% Store the history of states for plotting
x_history = zeros(num_agents, 3, num_steps);
x_history(1, :, 1) = x_ref';        % Initial state of Agent 1
x_history(2:end, :, 1) = hat_x_followers;

% Consensus protocol update loop
for k = 2:num_steps
    % Update Agent 1 (reference model) with its own dynamics
    x_ref = x_ref + time_step * (A * x_ref);
    x_history(1, :, k) = x_ref';
    
    % For each follower
    for i = 1:num_agents - 1
        % Agent index
        agent_idx = i + 1;
        
        % Current estimated state of agent i
        hat_x_i = hat_x_followers(i, :)'; % (3 x 1)
        
        % Initialize error vector e_i
        e_i = zeros(3,1);
        
        % Neighbors of agent i (from adjacency matrix)
        for j = 1:num_agents
            if A_adj(agent_idx, j) == 1
                if j == 1 % Neighbor is leader
                    e_i = e_i + l_i(i) * (hat_x_i - x_ref);
                else % Neighbor is another follower
                    hat_x_j = hat_x_followers(j-1, :)';
                    e_i = e_i + (hat_x_i - hat_x_j);
                end
            end
        end
        
        % Extract K_i
        K_i = K_array(i, :); % (1 x 3)
        
        % Compute K_i * e_i (scalar)
        K_e_i = K_i * e_i; % Scalar
        
        % Compute control input u_i = - c * B * (K_i * e_i)
        u_i = -c * B * K_e_i; % (3 x 1)
        
        % Get delta_i for follower i
        delta_i = delta_followers(i, :)'; % (3 x 1)
        
        % Compute dot_hat_x_i = A * hat_x_i + u_i - M * delta_i
        dot_hat_x_i = A * hat_x_i + u_i - M * delta_i;
        
        % Update hat_x_i
        hat_x_i = hat_x_i + time_step * dot_hat_x_i;
        
        % Store updated state
        hat_x_followers(i, :) = hat_x_i';
        
        % Store history
        x_history(i+1, :, k) = hat_x_i';
    end
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