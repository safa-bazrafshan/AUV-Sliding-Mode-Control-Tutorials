% smc_basic_example.m
% Simple SMC for second-order system

lambda = 2;
eta = 3;
x_d = 0;           % desired position
dx_d = 0;          % desired velocity
ddx_d = 0;         % desired acceleration

dt = 0.01;
T = 5;
t = 0:dt:T;

x = 5; dx = 0;     % initial condition
X = []; U = [];

for i = 1:length(t)
    e = x - x_d;
    de = dx - dx_d;
    s = de + lambda * e;
    
    u = ddx_d - lambda * de - eta * sign(s);
    
    ddx = u;
    dx = dx + ddx * dt;
    x = x + dx * dt;
    
    X(end+1) = x;
    U(end+1) = u;
end

% Plot
figure;
plot(t, X);
xlabel('Time (s)');
ylabel('x (position)');
title('System Response with Basic SMC');
grid on;