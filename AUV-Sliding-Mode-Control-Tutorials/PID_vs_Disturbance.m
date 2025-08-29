% PID Control with disturbance

t = 0:0.01:10;
r = ones(size(t)); % reference
y = zeros(size(t)); % output
e = zeros(size(t));
u = zeros(size(t));
Kp = 2; Ki = 1; Kd = 0.5;

for i = 2:length(t)
    dt = t(i) - t(i-1);
    e(i) = r(i) - y(i-1);
    de = (e(i) - e(i-1)) / dt;
    ie = sum(e(1:i)) * dt;
    u(i) = Kp*e(i) + Ki*ie + Kd*de;
    
    % Add a disturbance at t = 5
    d = 0;
    if t(i) > 5
        d = 2; % sudden disturbance
    end
    
    y(i) = y(i-1) + dt*(-y(i-1) + u(i) + d); % simple plant: y_dot = -y + u + d
end

plot(t, r, '--', t, y, 'LineWidth', 2);
xlabel('Time (s)');
ylabel('Output');
legend('Reference', 'Output');
title('PID Control under Disturbance');
grid on;