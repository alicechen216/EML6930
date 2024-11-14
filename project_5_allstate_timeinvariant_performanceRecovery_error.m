% MATLAB script for robust consensus protocol with specified K gains and time-invariant uncertainties
clc; clear;close all;

% Parameters
num_agents = 5;                      % Number of agents (Agent 1 is the leader + 4 followers)
time_step = 0.01;                    % Time step for simulation
total_time = 20;                     % Total simulation time
num_steps = total_time / time_step;  % Number of simulation steps

% System matrices
A = [0 1 0; 0 0 1; -4 -4 -1];        % Dynamics matrix for each agent
B = eye(3);                          % Input matrix (identity for vector control input)
C = [1 0 0];                         % Output matrix (observing the first state only)

% Robust control parameters
mu = 0.2;                            % Given parameter
c = 6;                               % Consensus scaling factor
P = [0.0850, 0.0316, 0.0224;         % Given matrix P
     0.0316, 0.0467, 0.0090;
     0.0224, 0.0090, 0.0187];
gamma = 100;                           % Given gamma

% Adjacency matrix representing connectivity between agents (including leader as Agent 1)
A_adj = [0 1 1 0 0;                  
         1 0 1 1 0;
         1 1 0 1 1;
         0 1 1 0 1;
         0 0 1 1 0];

% Degree matrix and Laplacian matrix
D = diag(sum(A_adj, 2));
L = D - A_adj;

% Specified K gains
K_leader = zeros(3, 3);              % Leader's K gain is zero
K_follower = diag([0.22, 0.09, 0.19]); % Followers' K gains

% Time-invariant uncertainties for each follower agent (delta_i)
delta_followers = [5   4   6  -4;     % Each column corresponds to a follower's delta_i (3x4 matrix)
                  -7  -5   3   1;
                  -3   4  -5   2];

% Initialize storage for plotting
x_history = zeros(num_agents, 3, num_steps);        % Actual states of all agents
x_hat_history = zeros(num_agents - 1, 3, num_steps);% Estimated states of followers
delta_hat_history = zeros(num_agents - 1, 3, num_steps); % Estimated uncertainties
error_history = zeros(num_agents - 1, num_steps);   % Error between followers and leader
delta_error_history = zeros(num_agents - 1, num_steps); % Estimation error norms

% Initial states
x_ref = [1; 1; 3];                   % Initial state of the leader (Agent 1)

% Initial states of followers
x_followers = [0  0   0;
               1  2  -1;
               3  1   0;
               2  2   2];            % Each row corresponds to a follower's initial state

% Initial estimated states \hat{x}_i(0)
x_hat_followers = x_followers;       % Can be initialized differently if needed

% Initial estimated uncertainties \hat{\delta}_i(0) for each follower agent
delta_hat_followers = zeros(3, num_agents - 1); % Initialize to zero or any initial guess

% Initialize history
x_history(1, :, 1) = x_ref';                     % Initial state of Agent 1
x_history(2:end, :, 1) = x_followers;
x_hat_history(:, :, 1) = x_hat_followers;
delta_hat_history(:, :, 1) = delta_hat_followers';

% Compute initial errors
for i = 1:num_agents - 1
    error = norm(x_followers(i, :)' - x_ref);
    error_history(i, 1) = error;
    
    delta_error = x_followers(i, :)' - x_hat_followers(i, :)' - delta_hat_followers(:, i);
    delta_error_history(i, 1) = norm(delta_error);
end

% Consensus protocol update loop
for k = 2:num_steps
    % Update Agent 1 (reference model) with its own dynamics
    x_ref = x_ref + time_step * (A * x_ref);
    
    % Initialize control input for followers
    u = zeros(3, num_agents - 1);  % Control inputs are 3-dimensional vectors
    
    % Update followers
    for i = 1:num_agents - 1
        % Use the specified K gain for followers
        K_i = K_follower;  % 3x3 diagonal matrix
        
        % Get current states and estimates
        x_i = x_followers(i, :)';
        x_hat_i = x_hat_followers(i, :)';
        delta_hat_i = delta_hat_followers(:, i);
        delta_i = delta_followers(:, i);
        
        % Compute estimation error
        e_i = x_i - x_hat_i - delta_hat_i;
        
        % Update estimated uncertainty \hat{\delta}_i(t)
        delta_hat_dot_i = -gamma * A' * P * e_i;
        delta_hat_i = delta_hat_i + time_step * delta_hat_dot_i;
        delta_hat_followers(:, i) = delta_hat_i; % Update \hat{\delta}_i
        
        % Update estimated state \hat{x}_i(t)
        % Compute consensus term based on estimated states
        consensus_error = zeros(size(x_i));
        for j = 1:num_agents
            if A_adj(i+1, j) == 1  % If agent j is a neighbor of agent i
                if j == 1          % Agent 1 is the leader
                    x_hat_j = x_ref;
                else
                    x_hat_j = x_hat_followers(j-1, :)';
                end
                consensus_error = consensus_error + (x_hat_i - x_hat_j);
            end
        end
        
        % Compute control input u_i
        e_total = c * consensus_error + (x_hat_i - x_ref); % Total error
        u(:, i) = -K_i * e_total;
        
        % Compute corrective signal
        corrective_signal = (gamma * A' * P + mu * eye(3)) * e_i;
        
        % Update estimated state \hat{x}_i(t)
        x_hat_dot_i = A * x_hat_i + B * u(:, i) + corrective_signal;
        x_hat_i = x_hat_i + time_step * x_hat_dot_i;
        x_hat_followers(i, :) = x_hat_i';
        
        % Update actual state x_i(t)
        x_dot_i = A * x_i + B * u(:, i) + delta_i;
        x_i = x_i + time_step * x_dot_i;
        x_followers(i, :) = x_i';
        
        % Compute errors for plotting
        error = norm(x_i - x_ref);
        error_history(i, k) = error;
        
        delta_error = e_i;
        delta_error_history(i, k) = norm(delta_error);
    end
    
    % Store the states and estimates for plotting
    x_history(1, :, k) = x_ref';             % Update Agent 1's state (leader)
    x_history(2:end, :, k) = x_followers;
    x_hat_history(:, :, k) = x_hat_followers;
    delta_hat_history(:, :, k) = delta_hat_followers';
end

% Time vector for plotting
time = linspace(0, total_time, num_steps);

% Plot results for each component of each agent's state
figure;

% First component subplot
subplot(3,1,1);
% Plot Agent 1 (leader)
plot(time, squeeze(x_history(1, 1, :)), 'k--', 'LineWidth', 3, 'DisplayName', 'Leader');
hold on;
% Plot the followers
colors = lines(num_agents - 1);  % Use distinct colors for followers
for i = 2:num_agents
    plot(time, squeeze(x_history(i, 1, :)), 'LineWidth', 2, 'Color', colors(i-1, :), ...
         'DisplayName', sprintf('Follower %d', i-1));
end
xlabel('Time');
ylabel('State Component 1');
title('State Component 1 over Time');
legend('show', 'Location', 'northeast', 'FontSize', 12);
grid on;

% Second component subplot
subplot(3,1,2);
% Plot Agent 1 (leader)
plot(time, squeeze(x_history(1, 2, :)), 'k--', 'LineWidth', 3, 'DisplayName', 'Leader');
hold on;
% Plot the followers
for i = 2:num_agents
    plot(time, squeeze(x_history(i, 2, :)), 'LineWidth', 2, 'Color', colors(i-1, :), ...
         'DisplayName', sprintf('Follower %d', i-1));
end
xlabel('Time');
ylabel('State Component 2');
title('State Component 2 over Time');
legend('show', 'Location', 'northeast', 'FontSize', 12);
grid on;

% Third component subplot
subplot(3,1,3);
% Plot Agent 1 (leader)
plot(time, squeeze(x_history(1, 3, :)), 'k--', 'LineWidth', 3, 'DisplayName', 'Leader');
hold on;
% Plot the followers
for i = 2:num_agents
    plot(time, squeeze(x_history(i, 3, :)), 'LineWidth', 2, 'Color', colors(i-1, :), ...
         'DisplayName', sprintf('Follower %d', i-1));
end
xlabel('Time');
ylabel('State Component 3');
title('State Component 3 over Time');
legend('show', 'Location', 'northeast', 'FontSize', 12);
grid on;

% Adjust the overall figure properties for better readability
set(gcf, 'Position', [100, 100, 800, 600]); % Resize figure window

% Plot the error between leader and followers over time
figure;
% Plot the error norms for each follower
for i = 1:num_agents - 1
    plot(time, error_history(i, :), 'LineWidth', 2, 'Color', colors(i, :), ...
         'DisplayName', sprintf('Follower %d', i));
    hold on;
end
xlabel('Time');
ylabel('Error Norm');
title('Error between Leader and Followers over Time');
legend('show', 'Location', 'northeast', 'FontSize', 12);
grid on;
set(gca, 'FontSize', 14);

% Plot the delta error (estimation error) over time
figure;
% Plot the delta error norms for each follower
for i = 1:num_agents - 1
    plot(time, delta_error_history(i, :), 'LineWidth', 2, 'Color', colors(i, :), ...
         'DisplayName', sprintf('Follower %d', i));
    hold on;
end
xlabel('Time');
ylabel('Estimation Error Norm');
title('Estimation Error between Actual and Estimated States over Time');
legend('show', 'Location', 'northeast', 'FontSize', 12);
grid on;
set(gca, 'FontSize', 14);