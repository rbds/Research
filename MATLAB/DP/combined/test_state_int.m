clear all
close all

robot.r = 0.75; %robot parameters
robot.t = 0;

% robot.p = [0; 0];               %initialize robot position to x_start
robot.p = [5;28];
robot.v = [0; 0; 0];

dt = 0.01;          %timestep
t = 0:dt:30;

x_d = [2*cos(0.5*t); 2*sin(0.5*t)]';    %desired trajectory (x and y velocity).
% x_d = 12*[ones(3500,2)];


x1 = [0 0 0]; %robot position
x2 = [0 0 0]; %robot velocity
x1_d = 0;
for i=1:length(t)-1
%     F = -[robot.p]' + x_d(i);
if (i<2) xdd = [0 0 0]; else  xdd = [x_d(i-1,:), 0]; end   %previous desired velocity.
    F = x_d(i,:);   %desired velocity (virtual force from PFM).
%     [robot] = state_int(robot, F, dt, xdd); %integrate robot state
    [robot] = si(robot, F, dt);
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
