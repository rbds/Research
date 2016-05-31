clear x1 x2 x_d
close all

robot.r = 0.75; %robot parameters
robot.t = 0;

robot.p = [0; 0];               %initialize robot position to x_start
robot.v = [0; 0; 0];

dt = 0.01;          %timestep
t = 0:dt:30;

x_d = [2*cos(0.5*t); 2*sin(0.5*t)]';    %desired trajectory (x and y velocity).

x1 = [0 0 0]; %robot position
x2 = [0 0 0]; %robot velocity

for i=1:length(t)-1
%     F = -[robot.p]' + x_d(i);
    F = x_d(i,:);   %desired velocity (virtual force from PFM).
    robot = state_int(robot, F, dt); %integrate robot state
    x1(end+1, :) = [robot.p(1), robot.p(2), robot.t]; 
    x2(end+1, :) = robot.v';
end

plot(t, x1)
legend('x', 'y', '\theta')
title('actual position')
figure
plot(t, x2)
hold on
plot(t, x_d(:,1), 'k--', t, x_d(:,2), 'k--')
title('actual and desired velocities')
legend('xdot', 'ydot', 'theta_dot')
