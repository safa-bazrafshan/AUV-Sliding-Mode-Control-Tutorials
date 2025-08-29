% AUV Comparison of SMC Variants
clear; clc;

% Simulation parameters
dt = 0.01; T = 10; t = 0:dt:T;
xd = sin(0.5*t); % Desired trajectory
xdot = 0.5*cos(0.5*t); % Desired velocity

% Model parameters
lambda = 2; k_basic = 3; phi = 0.1; % Boundary layer thickness
k_st = 2; % Super-twisting gain

% State initialization
x = zeros(size(t)); x_bl = zeros(size(t)); x_st = zeros(size(t));
v = zeros(size(t)); v_bl = zeros(size(t)); v_st = zeros(size(t));

% Super-twisting integral term
z = 0;

% Simulation loop
for i = 2:length(t)
    % Error terms
    e = x(i-1) - xd(i-1); edot = v(i-1) - xdot(i-1); s = lambda*e + edot;
    e_bl = x_bl(i-1) - xd(i-1); edot_bl = v_bl(i-1) - xdot(i-1); s_bl = lambda*e_bl + edot_bl;
    e_st = x_st(i-1) - xd(i-1); edot_st = v_st(i-1) - xdot(i-1); s_st = lambda*e_st + edot_st;

    % Basic SMC
    u = -k_basic * sign(s);

    % Boundary Layer SMC
    u_bl = -k_basic * sat(s_bl/phi);

    % Super-Twisting Algorithm
    u_st = -k_st * sqrt(abs(s_st)) * sign(s_st) + z;
    z = z - k_st * sign(s_st) * dt;

    % Dynamics (simple double integrator)
    v(i) = v(i-1) + u * dt; x(i) = x(i-1) + v(i) * dt;
    v_bl(i) = v_bl(i-1) + u_bl * dt; x_bl(i) = x_bl(i-1) + v_bl(i) * dt;
    v_st(i) = v_st(i-1) + u_st * dt; x_st(i) = x_st(i-1) + v_st(i) * dt;
end

% Plot results
figure;
plot(t, xd, 'k--', 'LineWidth', 1.5); hold on;
plot(t, x, 'r', 'LineWidth', 1.5);
plot(t, x_bl, 'b', 'LineWidth', 1.5);
plot(t, x_st, 'g', 'LineWidth', 1.5);
legend('Desired', 'Basic SMC', 'Boundary Layer', 'Super-Twisting');
xlabel('Time (s)'); ylabel('Position (m)');
title('Comparison of SMC Variants');
grid on;

% Saturation function
function y = sat(x)
    y = max(min(x, 1), -1);
end