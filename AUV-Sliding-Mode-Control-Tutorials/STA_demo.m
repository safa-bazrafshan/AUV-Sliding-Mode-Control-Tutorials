% Super-Twisting Algorithm demo
clear; clc; close all;

% Parameters
dt = 0.001;
t = 0:dt:5;
N = length(t);

% Reference and initial state
xd = sin(0.5*t); % desired trajectory
x = 0;           % initial position
xdot = 0;
x_hist = zeros(1,N);
u_hist = zeros(1,N);
s_hist = zeros(1,N);

% Gains
c = 2;
k1 = 5;
k2 = 10;

% Integral term
z = 0;

for i = 1:N
    e = x - xd(i);
    edot = xdot - 0.5*cos(0.5*t(i));
    s = c*e + edot;

    % STA control law
    u = -k1 * sqrt(abs(s)) * sign(s) - k2 * z;
    z = z + sign(s)*dt; % integral of sign(s)

    % Simple plant: x_ddot = u
    xddot = u;
    xdot = xdot + xddot*dt;
    x = x + xdot*dt;

    % Store data
    x_hist(i) = x;
    u_hist(i) = u;
    s_hist(i) = s;
end

% Plots
figure;
subplot(3,1,1);
plot(t, xd, 'r--', t, x_hist, 'b');
ylabel('Position');
legend('Desired','Actual');

subplot(3,1,2);
plot(t, s_hist);
ylabel('s');

subplot(3,1,3);
plot(t, u_hist);
ylabel('Control u');
xlabel('Time (s)');