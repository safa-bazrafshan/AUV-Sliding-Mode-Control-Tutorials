% DOB-SMC vs Pure SMC for a simple 2D AUV (surge & sway)
% Author: (Safa_bazrafshan) -- example for episode 15
clear; clc; close all;
rng(1);               % for reproducible noise

%% ---------------- Parameters ----------------
dt = 0.01; T = 40;         % time step and total time [s]
t = (0:dt:T)';             % column time vector
N = length(t);             % number of timesteps (fix for mismatch)

M = diag([20, 30]);        % mass matrix (2x2)
D = diag([5, 8]);          % linear damping (2x2)

alpha = 2;                 % sliding surface coefficient (s = de + alpha*e)
K = diag([20, 20]);        % SMC gain (2x2)
lambda_obs = 30;           % DOB observer gain (tune this)

%% ------------- Desired Trajectory --------------
xd = 5*cos(0.1*t);         % desired x (circle radius 5)
yd = 5*sin(0.1*t);         % desired y
xd_dot = -5*0.1*sin(0.1*t);% derivative of xd: -0.5*sin(0.1 t)
yd_dot =  5*0.1*cos(0.1*t);% derivative of yd:  0.5*cos(0.1 t)

%% --------------- Disturbance (true) -------------
% same disturbance for both controllers (fair comparison)
dist = zeros(N,2);
dist(:,1) = 0.5*sin(0.5*t) + 0.2*randn(N,1);  % surge disturbance (x)
dist(:,2) = 0.3*cos(0.3*t) + 0.2*randn(N,1);  % sway disturbance (y)

%% -------------- State Initialization ------------
% state vector: [pos_x; pos_y; vel_x; vel_y]
x_pure = zeros(4,N);   % trajectory under Pure SMC
x_dob  = zeros(4,N);   % trajectory under DOB-SMC

% observer state (disturbance estimate) for DOB
xh = zeros(2,N);       % xh(:,k) ≈ estimated disturbance at time k
dhat = zeros(2,N);     % store estimates for plotting

%% ---------------- Simulation Loop ----------------
for k = 1:N-1
    % --- current states (pure SMC) ---
    pos_p = x_pure(1:2,k);
    vel_p = x_pure(3:4,k);
    % --- current states (DOB-SMC) ---
    pos_d = x_dob(1:2,k);
    vel_d = x_dob(3:4,k);

    % desired at time k
    pos_des = [xd(k); yd(k)];
    vel_des = [xd_dot(k); yd_dot(k)];

    % tracking errors (actual - desired)
    e_p  = pos_p - pos_des;
    de_p = vel_p - vel_des;
    s_p  = de_p + alpha*e_p;          % sliding variable (pure)

    e_d  = pos_d - pos_des;
    de_d = vel_d - vel_des;
    s_d  = de_d + alpha*e_d;          % sliding variable (dob)

    % --- Pure SMC control (use sign) ---
    tau_smc_p = -K * sign(s_p);       % 2x1

    % --- DOB: get current estimate and build DOB-SMC input ---
    dhat(:,k) = xh(:,k);              % current disturbance estimate
    tau_smc_d = -K * sign(s_d);       % SMC component for DOB-SMC
    tau_dob   = tau_smc_d - dhat(:,k);% subtract estimate to compensate

    % --- System dynamics integration (Euler forward) ---
    % Pure SMC dynamics
    dv_p = M \ (tau_smc_p - D*vel_p + dist(k,:)');
    vel_p_next = vel_p + dt * dv_p;
    pos_p_next = pos_p + dt * vel_p;

    % DOB-SMC dynamics (uses tau_dob)
    dv_d = M \ (tau_dob - D*vel_d + dist(k,:)');
    vel_d_next = vel_d + dt * dv_d;
    pos_d_next = pos_d + dt * vel_d;

    % store next states
    x_pure(:,k+1) = [pos_p_next; vel_p_next];
    x_dob(:,k+1)  = [pos_d_next; vel_d_next];

    % --- Update Disturbance Observer (simple dynamics) ---
    % Note: observer uses position error (e_d) and M for scaling.
    % xh_dot = -lambda_obs*( xh + M^{-1} * e )
    xh(:,k+1) = xh(:,k) + dt * ( -lambda_obs * ( xh(:,k) + (M \ e_d) ) );
end
% store last estimate for plotting
dhat(:,N) = xh(:,N);

%% --------------- Performance Metrics -------------
err_p = sqrt( (x_pure(1,:) - xd').^2 + (x_pure(2,:) - yd').^2 ); % Euclid pos error over time
err_d = sqrt( (x_dob(1,:)  - xd').^2 + (x_dob(2,:)  - yd').^2 );

rms_p = sqrt(mean(err_p.^2));
rms_d = sqrt(mean(err_d.^2));

fprintf('RMS position error (Pure SMC): %.4f m\n', rms_p);
fprintf('RMS position error (DOB-SMC) : %.4f m\n', rms_d);

%% -------------------- Plots -----------------------
figure('Position',[100 100 1200 700]);
subplot(2,2,1);
plot(x_pure(1,:), x_pure(2,:), 'b', x_dob(1,:), x_dob(2,:), 'g', xd, yd, 'r--','LineWidth',1.2);
legend('Pure SMC','DOB-SMC','Desired'); axis equal;
xlabel('x [m]'); ylabel('y [m]'); title('Trajectory: Pure SMC vs DOB-SMC');

subplot(2,2,2);
plot(t, dist(:,1),'k', t, dhat(1,:),'r--','LineWidth',1);
legend('True dist (x)','Estimated d_x'); xlabel('Time [s]'); ylabel('Force');

subplot(2,2,3);
plot(t, dist(:,2),'k', t, dhat(2,:),'r--','LineWidth',1);
legend('True dist (y)','Estimated d_y'); xlabel('Time [s]'); ylabel('Force');

subplot(2,2,4);
plot(t, err_p, 'b', t, err_d, 'g','LineWidth',1);
legend('Error Pure SMC','Error DOB-SMC'); xlabel('Time [s]'); ylabel('Pos error [m]');
title(sprintf('RMS errors: Pure = %.3f m, DOB = %.3f m', rms_p, rms_d));