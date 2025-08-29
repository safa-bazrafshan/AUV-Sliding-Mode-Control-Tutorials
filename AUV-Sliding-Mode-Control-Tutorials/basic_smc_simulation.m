% basic_smc_simulation.m

clc; clear;

% Time
dt = 0.001;
T = 5;
t = 0:dt:T;

% Desired trajectory
xd = sin(t);
xd_dot = cos(t);
xd_ddot = -sin(t);



% Initial conditions
x = zeros(size(t));
x_dot = zeros(size(t));

% Controller gains
lambda = 2;
eta = 5;

% Storage for control input
u = zeros(size(t));



for i = 1:length(t)-1
    % Tracking error
    e = x(i) - xd(i);
    e_dot = x_dot(i) - xd_dot(i);
    
    % Sliding surface
    s = e_dot + lambda * e;
    
    % Control input
    u(i) = -lambda * e_dot - eta * sign(s);
    
    % System dynamics (Euler integration)
    x_ddot = u(i);
    x_dot(i+1) = x_dot(i) + x_ddot * dt;
    x(i+1) = x(i) + x_dot(i) * dt;
end



% Plotting
figure;
subplot(3,1,1);
plot(t, xd, 'r--', t, x, 'b');
legend('Desired', 'Actual');
ylabel('Position');

subplot(3,1,2);
plot(t, u);
ylabel('Control Input');

subplot(3,1,3);
plot(t, x - xd);
ylabel('Tracking Error');
xlabel('Time');
