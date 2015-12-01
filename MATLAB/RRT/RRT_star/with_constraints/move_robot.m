function [ l, c, h, y, robot, M ] = move_robot( robot, p_goal, target, obst,l, c, h, y, M )
%Function to move the robot until it sees an obstacle while driving with
%heuristic.


%Draw appropriate lines
%robot.p to goal

%direction robot just travelled.
orange = [230 218 57]./256;
p_sun = [5; -7];
k = 1;
stepsize = 10;
%Start moving towards the goal in increments of 'stepsize'
% dist2goal = norm(p_start - p_goal);
line = [robot.p(1), robot.p(2), target(1), target(2)];

set(l, 'visible', 'off')
l = plot([robot.p(1); p_goal(1)],[robot.p(2);p_goal(2)], 'r');
set(l, 'visible', 'on')
set(y, 'visible', 'off')
y = plot([robot.p(1); p_sun(1)], [robot.p(2); p_sun(2)], 'color', orange);
set(y, 'visible', 'on')

[~, colx, coly, ~] = sensorSweep(line, obst);
ints = cat(1, colx, coly);
for i = 1:length(colx)
   d = norm(robot.p - ints(:,i));
   if i==1 || d<closest
      closest = d; 
   end
end
if isnan(closest) %Check for NaN
   closest = 100; 
end

x_new = k*(target(1) -robot.p(1))/stepsize + robot.p(1);
y_new = k*(target(2) - robot.p(2))/stepsize + robot.p(2);
p_new = [x_new;y_new];
p_start = robot.p;
M(end +1) = getframe;
while (closest > (norm(robot.p - p_new) + 2*robot.r)) && (0 ~= norm(robot.p-target)) %while the next point is farther than the move distance + buffer,
%move robot
M(end +1) = getframe;
oldpos = robot.p;
robot.p = [x_new; y_new];
set(h, 'Visible', 'off')
h = draw_robot(robot);
set(h, 'Visible', 'on');
% set(c, 'visible', 'off')
c = plot([oldpos(1);robot.p(1)],[oldpos(2);robot.p(2)]);
% set(c, 'visible', 'on')
set(l, 'visible', 'off')
l = plot([robot.p(1); p_goal(1)],[robot.p(2);p_goal(2)], 'r');
set(l, 'visible', 'on')
set(y, 'visible', 'off')
y = plot([robot.p(1); p_sun(1)],[robot.p(2); p_sun(2)], 'color', orange);
set(y, 'visible', 'on')
% pause(.5)
k = k+1;

%find next point
x_new = k*(target(1) - p_start(1))/stepsize + p_start(1);
y_new = k*(target(2) - p_start(2))/stepsize + p_start(2);
p_new = [x_new;y_new];

%find the distance to the closest obstacle
line = [robot.p(1), robot.p(2), target(1), target(2)];
[~, colx, coly, ~] = sensorSweep(line, obst);
ints = cat(1, colx, coly);
    for i = 1:length(colx)
       d = norm(robot.p - ints(:,i));
       if i==1 || d<closest
          closest = d; 
       end
    end
    if isnan(closest)   %check for NaN
       closest = 100; 
    end
end

end

