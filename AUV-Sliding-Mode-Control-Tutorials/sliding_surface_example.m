% sliding_surface_example.m
% Simulate error dynamics for a basic sliding surface

lambda = 2;                   % Convergence rate
e0 = 5;                       % Initial tracking error
t = 0:0.01:5;                 % Time vector

e = e0 * exp(-lambda * t);    % Error decay: e(t) = e0 * exp(-lambda*t)

% Plotting
plot(t, e, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Tracking Error e(t)');
title('Error Dynamics on Sliding Surface');
grid on;