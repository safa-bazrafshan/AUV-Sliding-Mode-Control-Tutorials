% AUV_StateSpace_Model.m

A = [-0.5  0     0;
      0   -0.4   0.1;
      0   -0.05 -0.3];

B = eye(3);
C = eye(3);
D = zeros(3);

sys = ss(A, B, C, D);  % Create state-space system

t = 0:0.1:20;
u = ones(length(t), 3);  % Constant input

[y, t, x] = lsim(sys, u, t);  % Simulate system

plot(t, y)
legend('u', 'v', 'r')
title('AUV Simplified State-Space Response')
xlabel('Time (s)')
ylabel('Velocity')