% MATLAB script for consensus protocol with Brownian noise and nonlinear uncertainty using Neural adaptive control
%Performance recovery
clc; clear;close all;

% Parameters
num_agents = 5;                     % Number of agents (Agent 1 is the leader + 4 followers)
time_step = 0.001;                    % Time step for simulation
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

% Adaptive gain and leakage rate for bias estimation
gamma = 0.1;       % Adaptation rate for unknown bias compensation
lambda_leakage = 0.01;  % Leakage rate to counteract Brownian noise drift
b_max = 0.0001;         % Maximum bound for bias estimate

% Initialize bias estimates for each neighbor of each follower
bias_estimates = zeros(num_agents - 1, num_agents - 1, 2);  % Each bias estimate is a 2D vector

% Store the history of states for plotting
x_history = zeros(num_agents, 2, num_steps);

% Initial states
x_ref = [sin(0); cos(0)];            % Initial state of the reference model for Agent 1
x_followers = repmat(x_ref', num_agents - 1, 1) + [2, 1; 2, 0; 1, 2; 0, 2]; % Perturbed initial states for followers

% Initialize history
x_history(1, :, 1) = x_ref';         % Initial state of Agent 1
x_history(2:end, :, 1) = x_followers; % Initial states of followers
% Initialize Brownian noise
brownian_noise = zeros(num_agents - 1, num_steps);
noise_std = 0.01;
    
% Parameters for neural-adaptive control with RBF network
alpha = 0.5;      % Unknown parameter for sine term in nonlinear uncertainty
beta = 0.3;       % Unknown parameter for cosine term in nonlinear uncertainty
learning_rate = 0.01;  % Learning rate for RBF weight adaptation

% RBF network parameters
num_rbfs = 20;              % Number of RBFs
rbf_centers = linspace(-5, 5, num_rbfs);  % Centers of RBFs in state space
sigma = 0.1;                % Spread of RBFs

% Initialize RBF weights for each follower
W = zeros(num_agents - 1, num_rbfs);  % One RBF weight per center for each agent
% Store the history of true nonlinear uncertainty for plotting
f_true_history = zeros(num_agents - 1, num_steps);
error_history= zeros(num_agents - 1, num_steps);   % Error between each follower and leader
% Define RBF basis function
rbf = @(x, c) exp(-norm(x - c)^2 / (2 * sigma^2));

% Define the performance recovery threshold
recovery_threshold = 0.0001; % Adjust this threshold as needed
recovery_gain = 20;        % Gain for the performance recovery signal

% Define colors for each agent
colors = lines(num_agents - 1);  % Assign distinct colors for each follower

% Animation setup
video_filename = 'agent_trajectories_3D_animation_fixed.mp4'; % Name of the video file
video_writer = VideoWriter(video_filename, 'MPEG-4'); % Create video writer object
video_writer.FrameRate = 6000; % Set frames per second
open(video_writer);

% Initialize figure and axis outside the loop
figure_handle = figure('Position', [100, 100, 560, 420]); % Set fixed figure size
axis_handle = axes('Parent', figure_handle);
set(axis_handle, 'XLim', [-1, 2], 'YLim', [-10, 10], 'ZLim', [0, total_time]); % Set fixed axis limits

% Prepare plot handles for updating instead of replotting
hold on;
h_ref = plot3(axis_handle, NaN, NaN, NaN, 'k--', 'LineWidth', 2, 'DisplayName', 'Agent 1 (Reference Model)');
h_agents = gobjects(num_agents - 1, 1);
for i = 2:num_agents
    h_agents(i-1) = plot3(axis_handle, NaN, NaN, NaN, 'LineWidth', 1.5, 'Color', colors(i-1, :), 'DisplayName', sprintf('Agent %d', i));
end
hold off;

% Add labels, legend, and axis properties
xlabel(axis_handle, 'First Component');
ylabel(axis_handle, 'Second Component');
zlabel(axis_handle, 'Time');
title(axis_handle, '3D Plot: System Trajectories');
legend(axis_handle, 'show');
grid(axis_handle, 'on');
view(axis_handle, 45, 30); % Set a good view angle for 3D

% Adaptive control with RBF approximation of uncertainty
for k = 2:num_steps
    % Update Agent 1 (reference model) with a predefined sinusoidal trajectory
    t = (k - 1) * time_step;
    x_ref = [sin(t); cos(t)];  % Agent 1 follows this trajectory directly
    
    % Calculate the output of Agent 1
    y_ref = C * x_ref;
    
    % Generate Brownian noise by accumulating Gaussian noise
    brownian_noise(:, k) = brownian_noise(:, k-1) + noise_std * sqrt(time_step) * randn(num_agents - 1, 1);

    % Calculate noisy outputs for each follower agent
    y_followers = x_followers * C';  % Compute output y = C * x for followers only
    y_followers_noisy = y_followers + brownian_noise(:, k);  % Add Brownian noise to the followers' measurements

    % Initialize control input for followers
    u = zeros(num_agents - 1, 1);
    
    % Calculate control inputs for each follower agent with adaptive bias and RBF network
    for i = 1:num_agents - 1
        % Extract the LQR gain for the current follower
        K_i = K_array(i, :);
        
        % Reference tracking term (make follower i track Agent 1's state)
        ref_tracking = K_i(1) * (y_ref - y_followers_noisy(i));
        
        % RBF network output for nonlinear uncertainty approximation
        Phi_i = arrayfun(@(c) rbf(x_followers(i, :), c), rbf_centers);
        f_hat_i = W(i, :) * Phi_i';  % Approximation of f_i using RBF network
        
        % Ground truth nonlinear uncertainty function
        f_true_i = alpha * sin(x_followers(i, 1)) + beta * cos(x_followers(i, 2));
        % Store the true nonlinear uncertainty for plotting
        f_true_history(i, k) = f_true_i; 
        % Consensus term with bias compensation
        consensus_term = 0;
        for j = 1:num_agents - 1
            if L(i+1, j+1) == 1  % Only consider neighbors
                % Update bias estimates adaptively with leakage and barrier function
                norm_diff = norm(bias_estimates(i, j, :) - b_leader);
                if norm_diff < delta
                    barrier_grad = 0;
                else
                    barrier_grad = (bias_estimates(i, j, :) - b_leader) / (delta^2 - norm_diff^2);
                end
                
                bias_estimates(i, j, :) = bias_estimates(i, j, :) + ...
                    (gamma * ((y_followers_noisy(j) - y_followers_noisy(i)) - bias_estimates(i, j, :)) ...
                    - lambda_leakage * bias_estimates(i, j, :) + barrier_grad) * time_step;
                
                % Consensus term with adaptive bias compensation
                consensus_term = consensus_term + K_i(2) * ((y_followers_noisy(j) - y_followers_noisy(i)) - bias_estimates(i, j, :));
            end
        end
        
        % Control input combining reference tracking, consensus, and RBF network adaptation
        u(i) = ref_tracking + consensus_term - f_hat_i;
        error=abs(x_ref(1) - x_followers(i, 1));
        error_history(i, k) = error;

        % Update RBF weights for follower i
        W(i, :) = W(i, :) + learning_rate * (f_true_i - f_hat_i) * Phi_i * time_step; % Adapt weights based on error
        
        % Apply performance recovery signal if error exceeds threshold
        if error > recovery_threshold
            recovery_signal = recovery_gain * (x_ref(1) - x_followers(i, 1));
            u(i) = u(i) + recovery_signal;  % Add recovery signal to control input
        end
    end
    
    % Update each follower's state based on system dynamics including true nonlinear uncertainty
    x_dot_followers = (A * x_followers')' + (B * u')' +f_true_i;  % Dynamics include true uncertainty
    x_followers = x_followers + time_step * x_dot_followers;  % Update follower states
    % Store error between follower and leader
   
    % Store the states for plotting
    x_history(1, :, k) = x_ref';       % Update Agent 1's state (leader)
    x_history(2:end, :, k) = x_followers;  % Update follower states
    
    % Update plot data instead of clearing figure
    set(h_ref, 'XData', squeeze(x_history(1, 1, 1:k)), 'YData', squeeze(x_history(1, 2, 1:k)), 'ZData', (1:k) * time_step);
    for i = 2:num_agents
        set(h_agents(i-1), 'XData', squeeze(x_history(i, 1, 1:k)), 'YData', squeeze(x_history(i, 2, 1:k)), 'ZData', (1:k) * time_step);
    end
    
    % Capture the current frame from the specific axis
    frame = getframe(axis_handle);
    writeVideo(video_writer, frame);
end

% Close the video writer
close(video_writer);

disp(['3D Animation saved as ', video_filename]);