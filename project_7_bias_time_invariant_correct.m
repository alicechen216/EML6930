% MATLAB script for consensus protocol with constant noise across all states and adaptive bias estimation
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
R = 0.01;                            % Input-cost matrix

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

% Adaptive gain for bias estimation
gamma = 10;  % Adaptation rate for unknown bias compensation

% Initialize bias estimates for each neighbor of each follower
bias_estimates = zeros(num_agents - 1, num_agents - 1, 2);  % Each bias estimate is a 2D vector

% Store the history of states and errors for plotting
x_history = zeros(num_agents, 2, num_steps);
error_history = zeros(num_agents - 1, num_steps); % Error between followers and the leader
noise_history = zeros(num_agents - 1, num_steps); % Noise added to each follower

% Initial states
x_ref = [sin(0); cos(0)];            % Initial state of the reference model for Agent 1
x_followers = repmat(x_ref', num_agents - 1, 1) + [2, 1; 2, 0; 1, 2; 0, 2]; % Perturbed initial states for followers

% Initialize history
x_history(1, :, 1) = x_ref';         % Initial state of Agent 1
x_history(2:end, :, 1) = x_followers; % Initial states of followers

% Constant noise for each follower (different constant for each, applied to all states)
constant_noise = [5, -2; 1, -8; -3, 4; 6, -5];  % Constant noise for each agent's state

% Consensus protocol update loop with constant noise across all states and adaptive bias compensation
for k = 2:num_steps
    % Update Agent 1 (reference model) with a predefined sinusoidal trajectory
    t = (k - 1) * time_step;
    x_ref = [sin(t); cos(t)];  % Agent 1 follows this trajectory directly
    
    % Calculate the output of Agent 1
    y_ref = C * x_ref;
    
    % Apply constant noise to the states of each follower
    x_followers_noisy = x_followers + constant_noise;  % Add constant noise to each follower's state
    
    % Store noise data for plotting (take only the first component for simplicity)
    noise_history(:, k) = constant_noise(:, 1);

    % Initialize control input for followers
    u = zeros(num_agents - 1, 1);
    
    % Calculate control inputs for each follower agent with noise and adaptive bias estimation
    for i = 1:num_agents - 1
        % Extract the LQR gain for the current follower
        K_i = K_array(i, :);
        
        % Reference tracking term (make follower i track Agent 1's state)
        ref_tracking = K_i(1) * (y_ref - C * x_followers_noisy(i, :)');
        
        % Consensus term with bias compensation
        consensus_term = 0;
        for j = 1:num_agents - 1
            if L(i+1, j+1) == 1  % Only consider neighbors
                % Update bias estimates adaptively
                bias_estimates(i, j, :) = bias_estimates(i, j, :) + ...
                    gamma * ((C * x_followers_noisy(j, :)' - C * x_followers_noisy(i, :)') - bias_estimates(i, j, :)) * time_step;
                
                % Consensus term with adaptive bias compensation
                consensus_term = consensus_term + K_i(2) * ((C * x_followers_noisy(j, :)' - C * x_followers_noisy(i, :)') - bias_estimates(i, j, :));
            end
        end
        
        % Control input combining reference tracking and consensus
        u(i) = ref_tracking + consensus_term;
    end
    
    % Update each follower's state based on system dynamics
    x_dot_followers = (A * x_followers')' + (B * u')';  % Matrix operations to match dimensions
    x_followers = x_followers + time_step * x_dot_followers;  % Update follower states
    
    % Store the states and errors for plotting
    x_history(1, :, k) = x_ref';       % Update Agent 1's state (leader)
    x_history(2:end, :, k) = x_followers;  % Update follower states
    error_history(:, k) = vecnorm(x_followers - x_ref', 2, 2);  % Error between each follower and the leader
end
% Plot results
time = linspace(0, total_time, num_steps);

% Plot the first component of each agent's state over time
figure;
subplot(1, 2, 1);
hold on;
plot(time, squeeze(x_history(1, 1, :)), 'k--', 'LineWidth', 3, 'DisplayName', 'Agent 1 (Reference Model)');
colors = lines(num_agents - 1);  % Use distinct colors for followers
for i = 2:num_agents
    plot(time, squeeze(x_history(i, 1, :)), 'LineWidth', 2, 'Color', colors(i-1, :), ...
         'DisplayName', sprintf('Agent %d', i));
end
title('2D Plot: First Component Over Time','FontSize',24);
xlabel('Time');
ylabel('Agent States (First Component)');
legend;
grid on;
ax = gca; % Get the current axis
ax.FontSize = 18; % Increase font size of tick labels
ax.XColor = [0.1, 0.1, 0.1]; % Darken X-axis tick labels
ax.YColor = [0.1, 0.1, 0.1]; % Darken Y-axis tick labels
ax.LineWidth = 1.5; % Increase axis line width for better clarity
% 3D Plot: System Trajectories (First Component, Second Component, Time)
subplot(1, 2, 2);
hold on;
plot3(squeeze(x_history(1, 1, :)), squeeze(x_history(1, 2, :)), time, 'k--', 'LineWidth', 3, 'DisplayName', 'Agent 1 (Reference Model)');
for i = 2:num_agents
    plot3(squeeze(x_history(i, 1, :)), squeeze(x_history(i, 2, :)), time, 'LineWidth', 2, 'Color', colors(i-1, :), ...
          'DisplayName', sprintf('Agent %d', i));
end
title('3D Plot: System Trajectories','FontSize',24);
xlabel('First Component');
ylabel('Second Component');
zlabel('Time');
legend;
grid on;
view(45, 30);  % Adjust view angle for better visualization
ax = gca; % Get the current axis
ax.FontSize = 18; % Increase font size of tick labels
ax.XColor = [0.1, 0.1, 0.1]; % Darken X-axis tick labels
ax.YColor = [0.1, 0.1, 0.1]; % Darken Y-axis tick labels
ax.LineWidth = 1.5; % Increase axis line width for better clarity
% Plot error between each follower and the leader
figure;
subplot(1, 2, 1);
hold on;
for i = 1:num_agents - 1
    plot(time, error_history(i, :), 'LineWidth', 2, 'Color', colors(i, :), ...
         'DisplayName', sprintf('Error: Agent %d to Leader', i + 1));
end
title('Error between Followers and Leader over Time','FontSize',24);
xlabel('Time');
ylabel('Error Magnitude');
legend;
grid on;
ax = gca; % Get the current axis
ax.FontSize = 18; % Increase font size of tick labels
ax.XColor = [0.1, 0.1, 0.1]; % Darken X-axis tick labels
ax.YColor = [0.1, 0.1, 0.1]; % Darken Y-axis tick labels
ax.LineWidth = 1.5; % Increase axis line width for better clarity
% Plot noise added to each follower over time
subplot(1, 2, 2);
hold on;
for i = 1:num_agents - 1
    plot(time, noise_history(i, :), 'LineWidth', 1.5, 'Color', colors(i, :), ...
         'DisplayName', sprintf('Noise for Agent %d', i + 1));
end
ylim([-10 10]);
title('Time Invariant Uncertainty Effect on Followers over Time','FontSize',24);
xlabel('Time');
ylabel('Time Invariant Uncertainty Magnitude');
legend;
grid on;
% Customize axis tick labels for clarity
ax = gca; % Get the current axis
ax.FontSize = 18; % Increase font size of tick labels
ax.XColor = [0.1, 0.1, 0.1]; % Darken X-axis tick labels
ax.YColor = [0.1, 0.1, 0.1]; % Darken Y-axis tick labels
ax.LineWidth = 1.5; % Increase axis line width for better clarity