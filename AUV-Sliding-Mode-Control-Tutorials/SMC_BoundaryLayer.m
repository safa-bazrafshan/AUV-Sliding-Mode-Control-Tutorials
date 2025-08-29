% Sliding Mode Control with Boundary Layer

% Parameters
lambda = 2;   % lambda in s = e_dot + lambda*e
k = 5;        % control gain
phi = 0.1;    % boundary layer thickness

% Time
dt = 0.01; 
T = 5;
t = 0:dt:T;

% Desired trajectory
xd = sin(t);
xd_dot = cos(t);

% System initialization
x = zeros(size(t));     % actual position
x_dot = zeros(size(t)); % actual velocity

for i = 1:length(t)-1
    e = x(i) - xd(i);
    edot = x_dot(i) - xd_dot(i);
    s = edot + lambda * e;

    % Boundary layer saturation
    sat_s = max(min(s / phi, 1), -1);  % saturation function

    % Control law
    u = -k * sat_s;

    % Simple plant: x_ddot = u
    x_ddot = u;
    x_dot(i+1) = x_dot(i) + x_ddot * dt;
    x(i+1) = x(i) + x_dot(i+1) * dt;
end

% Plot results
figure;
plot(t, xd, 'r--', 'LineWidth', 2); hold on;
plot(t, x, 'b', 'LineWidth', 2);
xlabel('Time (s)'); ylabel('Position');
legend('Desired', 'Actual');
title('SMC with Boundary Layer (Saturation)');
grid on;