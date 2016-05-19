robot.r = 0.75;
robot.t = 0;

robot.p = [0; 0];               %set robot position to x_start
robot.v = [0; 0; 0];

x_d = [10, 10];
F = -[robot.p]' + x_d;
dt = 0.01;

x1 = [0 0 0];
x2 = [0 0 0];

while norm(F)> 1
    robot = state_int(robot, F, dt);
    x1(end+1, :) = [robot.p(1), robot.p(2), robot.t];
    x2(end+1, :) = robot.v';
end

