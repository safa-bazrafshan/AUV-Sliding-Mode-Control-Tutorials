% Episode 17 - Capstone Project: Full SMC Controller Integration for AUV
clear; clc; close all;

%% Simulation Parameters
dt = 0.01; 
T = 60; 
t = 0:dt:T; 
N = length(t);

% AUV states for Basic SMC [x, y, u, v]
x_smc = zeros(N,1); 
y_smc = zeros(N,1);
u_smc = zeros(N,1); 
v_smc = zeros(N,1);

% AUV states for DOB-SMC [x, y, u, v]
x_dob = zeros(N,1); 
y_dob = zeros(N,1);
u_dob = zeros(N,1); 
v_dob = zeros(N,1);

% Control inputs
tau_smc = zeros(N,2); % Basic SMC
tau_dob = zeros(N,2); % DOB-SMC

% Disturbances
dist_true = [0.5*sin(0.3*t); 0.3*randn(1,N)]'; % [surge, sway]

% Disturbance observer states
dist_hat = zeros(N,2);

%% Desired Path (Infinity shape)
x_ref = sin(0.1*t); 
y_ref = sin(0.1*t).*cos(0.1*t);
x_ref_dot = 0.1*cos(0.1*t); 
y_ref_dot = 0.1*(cos(0.2*t)-sin(0.2*t))/2; % derivative approx

%% Controller Parameters
lambda = diag([1,1]);     % Sliding surface gains
k_smc = [3,3];            % Basic SMC gains
k_dob = [3,3];            % DOB-SMC gains
gamma = 5;                % Observer gain

%% Simulation Loop
for k = 1:N-1
    % --- Basic SMC ---
    % Tracking errors
    e_smc = [x_smc(k)-x_ref(k); y_smc(k)-y_ref(k)];
    e_dot_smc = [u_smc(k)-x_ref_dot(k); v_smc(k)-y_ref_dot(k)];
    
    % Sliding surface
    s_smc = e_dot_smc + lambda*e_smc;
    
    % Control law
    tau_smc(k,:) = (-k_smc).*sign(s_smc');
    
    % Update dynamics
    u_smc(k+1) = u_smc(k) + dt*(tau_smc(k,1) + dist_true(k,1)); % surge vel
    v_smc(k+1) = v_smc(k) + dt*(tau_smc(k,2) + dist_true(k,2)); % sway vel
    x_smc(k+1) = x_smc(k) + dt*u_smc(k+1);
    y_smc(k+1) = y_smc(k) + dt*v_smc(k+1);
    
    % --- DOB-SMC ---
    % Tracking errors
    e_dob = [x_dob(k)-x_ref(k); y_dob(k)-y_ref(k)];
    e_dot_dob = [u_dob(k)-x_ref_dot(k); v_dob(k)-y_ref_dot(k)];
    
    % Sliding surface
    s_dob = e_dot_dob + lambda*e_dob;
    
    % Observer update (first-order low-pass filter style)
    dist_hat(k+1,:) = dist_hat(k,:) + dt*(-gamma*(dist_hat(k,:) - dist_true(k,:)));
    
    % Control law
    tau_dob(k,:) = (-k_dob).*sign(s_dob') - dist_hat(k,:);
    
    % Update dynamics
    u_dob(k+1) = u_dob(k) + dt*(tau_dob(k,1) + dist_true(k,1)); % surge vel
    v_dob(k+1) = v_dob(k) + dt*(tau_dob(k,2) + dist_true(k,2)); % sway vel
    x_dob(k+1) = x_dob(k) + dt*u_dob(k+1);
    y_dob(k+1) = y_dob(k) + dt*v_dob(k+1);
end

%% Plot Results
% Path tracking comparison
figure('Name','Path Tracking Comparison');
plot(x_ref, y_ref, 'k--', 'LineWidth', 2); hold on;
plot(x_smc, y_smc, 'b', 'LineWidth', 1.5);
plot(x_dob, y_dob, 'r', 'LineWidth', 1.5);
legend('Reference Path', 'Basic SMC', 'DOB-SMC');
xlabel('X (m)'); ylabel('Y (m)'); 
title('AUV Path Tracking: Basic SMC vs DOB-SMC');
grid on;

% Disturbance estimation
figure('Name','Disturbance Estimation');
subplot(2,1,1);
plot(t, dist_true(:,1), 'k', 'LineWidth', 1.5); hold on;
plot(t, dist_hat(:,1), 'r--', 'LineWidth', 1.5);
legend('True Surge Disturbance', 'Estimated Surge Disturbance');
xlabel('Time (s)'); ylabel('Disturbance (N)'); 
title('Surge Disturbance Estimation');
grid on;

subplot(2,1,2);
plot(t, dist_true(:,2), 'k', 'LineWidth', 1.5); hold on;
plot(t, dist_hat(:,2), 'r--', 'LineWidth', 1.5);
legend('True Sway Disturbance', 'Estimated Sway Disturbance');
xlabel('Time (s)'); ylabel('Disturbance (N)'); 
title('Sway Disturbance Estimation');
grid on;

% Tracking error comparison
error_smc = sqrt((x_smc-x_ref').^2 + (y_smc-y_ref').^2);
error_dob = sqrt((x_dob-x_ref').^2 + (y_dob-y_ref').^2);

figure('Name','Tracking Error Comparison');
plot(t, error_smc, 'b', 'LineWidth', 1.5); hold on;
plot(t, error_dob, 'r', 'LineWidth', 1.5);
legend('Basic SMC Error', 'DOB-SMC Error');
xlabel('Time (s)'); ylabel('Tracking Error (m)'); 
title('Tracking Error: Basic SMC vs DOB-SMC');
grid on;