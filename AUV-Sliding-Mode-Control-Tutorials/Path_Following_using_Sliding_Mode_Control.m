%% Episode 13 - Path Following using Sliding Mode Control (SMC)
% Smooth implementation with boundary layer to reduce chattering

clear; clc; close all;

%% --- Parameters ---
u = 1;              % Forward speed [m/s]
lambda = 1.5;       % Sliding surface parameter
k = 1;              % SMC gain (reduced for smooth response)
phi = 0.5;          % Boundary layer thickness for smooth control
dt = 0.05;          % Time step [s] (smaller for smoother integration)
T = 50;             % Total simulation time [s]
N = T/dt;           % Number of iterations

%% --- Initial states ---
x = 0;              % Initial x position
y = 5;              % Initial y position (off-path)
psi = 0;            % Initial heading angle [rad]

%% --- Storage for plotting ---
X = zeros(1,N);
Y = zeros(1,N);

%% --- Simulation loop ---
for i = 1:N
    % Cross-track error (desired path: y=0)
    ey = y;                 
    % Derivative of error
    eydot = u*sin(psi);
    
    % Sliding surface
    s = ey + lambda*eydot;
    
    % Equivalent control (for straight path)
    req = -lambda*eydot;
    
    % SMC control law with boundary layer (smooth sign)
    r = req - k * max(min(s/phi,1),-1); % saturation function
    
    % Update dynamics
    psi = psi + r*dt;
    x = x + u*cos(psi)*dt;
    y = y + u*sin(psi)*dt;
    
    % Save data for plotting
    X(i) = x; 
    Y(i) = y;
end

%% --- Plot results ---
figure;
plot(X,Y,'b','LineWidth',2); hold on;    % AUV path in blue
yline(0,'r--','LineWidth',2);            % Desired path at y=0 in red dashed
xlabel('X [m]'); 
ylabel('Y [m]');
legend('AUV Path','Desired Path');
title('Path Following using Sliding Mode Control (SMC)');
ylim([-0.5 5.5]);                        % y-axis limits
grid on;