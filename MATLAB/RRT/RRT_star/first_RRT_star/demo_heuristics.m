close all
figure(1)
hold on
clear

course = [-15 -20 30 13];
numberOfObstacles = 5;
map = [];
map(1).p = [course(1) course(2)]';
s = [];
% Set Start and Goal locations
p_start = [-10; 8]; %For going to the sun
% p_start = [-10;-15];%For skipping the sun
p_goal =  [26; -14];
p_sun = [5; -7];
% Draw Course    
obst = draw_obstacles(numberOfObstacles, course);
goal.r = .5;            %radius of goal            
robot.r = 0.75;

robot.p = p_start;               %set robot position to x_start

circle(p_start(1,1),p_start(2,1),goal.r,'g');               %draw the location of x_start and x_goal
circle(p_goal(1,1),p_goal(2,1),goal.r,'g');
%draw the sun.
th = 0:pi/50:2*pi;
xunit = 2 * cos(th) + p_sun(1);
yunit = 2 * sin(th) + p_sun(2);
orange = [230 218 57]./256;
plot(xunit, yunit,'LineWidth',3, 'Color', orange);
drawnow
axis([-20 40 -30 20])
h = draw_robot(robot);
% pause(.5);
line = [robot.p(1), robot.p(2), p_goal(1), p_goal(2)];
l = plot([line(1);line(3)],[line(2);line(4)], 'r');
c = plot([robot.p(1);robot.p(1)],[robot.p(2);robot.p(2)]);
y = plot([robot.p(1); p_sun(1)],[robot.p(2); p_sun(2)], 'color', orange);

M(1) = getframe;
% % This section is for the demo where it uses the sun
% % move robot towards the goal.
[l,c,h, y, robot, M] = move_robot(robot, p_goal, p_goal, obst, l, c, h, y, M);

new_goal = [2; 5.8];
[l,c,h, y, robot, M] = move_robot(robot, p_goal, new_goal, obst, l, c, h, y, M);
[map, s, M] = sensor(robot, obst, map, s, M);
new_goal = p_goal;
[l,c,h, y, robot, M] = move_robot(robot, p_goal, new_goal, obst, l, c, h, y, M);
[map, s, M] = sensor(robot, obst, map, s, M);
new_goal = p_sun;
[l,c,h, y, robot, M] = move_robot(robot, p_goal, new_goal, obst, l, c, h, y, M);
[map, s, M] = sensor(robot, obst, map, s, M);
new_goal = p_goal;
[l,c,h, y, robot, M] = move_robot(robot, p_goal, new_goal, obst, l, c, h, y, M);
[map, s, M] = sensor(robot, obst, map, s, M);
new_goal = [10;-12];
[l,c,h, y, robot, M] = move_robot(robot, p_goal, new_goal, obst, l, c, h, y, M);
[map, s, M] = sensor(robot, obst, map, s, M);
new_goal = p_goal;
[l,c,h, y, robot, M] = move_robot(robot, p_goal, new_goal, obst, l, c, h, y, M);
[map, s, M] = sensor(robot, obst, map, s, M);
new_goal = [18; -16];
[l,c,h, y, robot, M] = move_robot(robot, p_goal, new_goal, obst, l, c, h, y, M);
[map, s, M] = sensor(robot, obst, map, s, M);
new_goal = [26; -16];
[l,c,h, y, robot, M] = move_robot(robot, p_goal, new_goal, obst, l, c, h, y, M);
[map, s, M] = sensor(robot, obst, map, s, M);
new_goal = p_goal;
[l,c,h, y, robot, M] = move_robot(robot, p_goal, new_goal, obst, l, c, h, y, M);
[map, s, M] = sensor(robot, obst, map, s, M);


% %This section is for the demo where it does not use the sun.
% new_goal = [-5, -17]';
% [l,c,h, y, robot, M] = move_robot(robot, p_goal, new_goal, obst, l, c, h, y, M);
% [map, s, M] = sensor(robot, obst, map, s, M);
% 
% new_goal = [ 26, -17]';
% [l,c,h, y, robot, M] = move_robot(robot, p_goal, new_goal, obst, l, c, h, y, M);
% [map, s, M] = sensor(robot, obst, map, s, M);
% 
% new_goal = p_goal;
% [l,c,h, y, robot, M] = move_robot(robot, p_goal, new_goal, obst, l, c, h, y, M);
% [map, s, M] = sensor(robot, obst, map, s, M);