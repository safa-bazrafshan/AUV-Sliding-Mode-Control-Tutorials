% Episode 16: Full Mission Simulation with Disturbances
clear; clc; close all;

%% Parameters
M = diag([50, 50]);     % Mass matrix (kg)
D = diag([20, 20]);     % Damping matrix
dt = 0.01;              % Time step
T  = 60;                % Total simulation time
t  = 0:dt:T; 
N  = length(t);

%% Mission Path (S-shaped trajectory)
xd = 5*sin(0.1*t);
yd = 5*sin(0.05*t);
dxd = 0.5*cos(0.1*t);
dyd = 0.25*cos(0.05*t);

%% Disturbances (sinusoidal + noise)
dist = [0.5*sin(0.2*t)' + 0.2*randn(N,1), ...
        0.3*cos(0.1*t)' + 0.2*randn(N,1)];

%% Initialization
x_pure = zeros(2,1);   v_pure = zeros(2,1);
x_dob  = zeros(2,1);   v_dob  = zeros(2,1);
x_hist_pure = zeros(2,N); 
x_hist_dob  = zeros(2,N);
dist_est = zeros(2,1); 
dist_est_hist = zeros(2,N);

%% Controller Gains
K = diag([30,30]);   % SMC gain
Lambda = 5*eye(2);   % Observer gain

%% Simulation Loop
for k=1:N
    xd_k = [xd(k); yd(k)];
    dxd_k = [dxd(k); dyd(k)];
    err_pure = x_pure - xd_k;
    err_dob  = x_dob - xd_k;

    % Sliding variables
    s_pure = dxd_k - v_pure + 2*err_pure;
    s_dob  = dxd_k - v_dob + 2*err_dob;

    % Pure SMC control
    u_pure = -K*sign(s_pure);

    % DOB-SMC control (subtract estimated disturbance)
    u_dob = -K*sign(s_dob) - dist_est;

    % Dynamics update
    v_pure = v_pure + dt*(M\(u_pure - D*v_pure + dist(k,:)'));
    x_pure = x_pure + dt*v_pure;

    v_dob = v_dob + dt*(M\(u_dob - D*v_dob + dist(k,:)'));
    x_dob = x_dob + dt*v_dob;

    % Disturbance observer update
    dist_est = dist_est + dt*(-Lambda*dist_est + Lambda*(u_dob - M*(v_dob/dt)));
    
    % Save
    x_hist_pure(:,k) = x_pure;
    x_hist_dob(:,k)  = x_dob;
    dist_est_hist(:,k) = dist_est;
end

%% RMS Error
rms_pure = sqrt(mean(sum((x_hist_pure - [xd;yd]).^2,1)));
rms_dob  = sqrt(mean(sum((x_hist_dob - [xd;yd]).^2,1)));

%% Plots
figure;
subplot(2,2,1)
plot(xd,yd,'k--','LineWidth',1.5); hold on;
plot(x_hist_pure(1,:),x_hist_pure(2,:),'r');
plot(x_hist_dob(1,:),x_hist_dob(2,:),'b');
legend('Desired Path','Pure SMC','DOB-SMC');
xlabel('x [m]'); ylabel('y [m]');
title('Trajectory Tracking');

subplot(2,2,2)
plot(t,dist(:,1),'k',t,dist_est_hist(1,:),'r--');
xlabel('Time [s]'); ylabel('Disturbance X');
legend('True','Estimated'); title('X Disturbance');

subplot(2,2,3)
plot(t,dist(:,2),'k',t,dist_est_hist(2,:),'r--');
xlabel('Time [s]'); ylabel('Disturbance Y');
legend('True','Estimated'); title('Y Disturbance');

subplot(2,2,4)
plot(t, sqrt(sum((x_hist_pure-[xd;yd]).^2,1)),'r');
hold on;
plot(t, sqrt(sum((x_hist_dob-[xd;yd]).^2,1)),'b');
xlabel('Time [s]'); ylabel('Position Error [m]');
legend('Pure SMC','DOB-SMC');
title(sprintf('RMS Error: Pure=%.3f, DOB=%.3f',rms_pure,rms_dob));