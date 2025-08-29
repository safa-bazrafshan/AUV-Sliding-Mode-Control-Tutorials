clc; clear; close all;

%% Parameters
dt = 0.01; 
T  = 10; 
N  = T/dt;

x = 0; 
y = 5;                 % initial offset
y_des = 0;             % desired path

% Control parameters
lambda = 1.5;
k      = 2;            % SMC gain
phi    = 0.1;          % boundary layer
gamma  = 5;            % adaptation gain

% Unknown disturbance (real system)
d_true = @(t) 0.5 + 0.2*sin(0.5*t);  

% Adaptive estimate of disturbance
d_hat = 0;

% Data storage
X  = zeros(1,N);
Y  = zeros(1,N);
Yd = zeros(1,N);
d_hat_hist = zeros(1,N);
d_true_hist = zeros(1,N);

%% Simulation loop
for i = 1:N
    t = (i-1)*dt;
    
    % Error
    e  = y - y_des;
    de = 0;   % assume desired trajectory derivative = 0
    
    % Sliding surface
    s = lambda*e + de;
    
    % Adaptive SMC control law
    sat_s = max(min(s/phi,1),-1);   % saturation function
    u = -k*sat_s - d_hat;           % control input with disturbance estimate
    
    % True disturbance
    d = d_true(t);
    
    % AUV dynamics (simple model)
    dy = u + d;   % y_dot
    
    % Update state
    y = y + dy*dt;
    x = x + dt;   % move forward in x
    
    % Adaptation law (gradient update)
    d_hat = d_hat - gamma * s * sat_s * dt;
    
    % Store data
    X(i) = x;
    Y(i) = y;
    Yd(i) = y_des;
    d_hat_hist(i) = d_hat;
    d_true_hist(i) = d;
end

%% Plot results
figure;
plot(X,Y,'b','LineWidth',2); hold on;
plot(X,Yd,'r--','LineWidth',2);
xlabel('X [m]'); ylabel('Y [m]');
legend('AUV Path','Desired Path');
title('Adaptive Sliding Mode Control for AUV Path Following');
grid on;

figure;
plot((0:N-1)*dt,d_true_hist,'r--','LineWidth',2); hold on;
plot((0:N-1)*dt,d_hat_hist,'b','LineWidth',2);
xlabel('Time [s]'); 
ylabel('Disturbance / Estimate','Interpreter','latex');
legend('True disturbance $d$','Estimated $\hat{d}$','Interpreter','latex');
title('Adaptive Disturbance Estimation (SMC)');
grid on;