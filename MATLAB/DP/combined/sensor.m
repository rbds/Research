function [ map, s ] = sensor( robot, obst, map, s, rad, course)
% function [ map, s, M ] = sensor( robot, obst, map, s, rad, course, M)
%Sensor Sweep
num_readings = 25;

theta = linspace(0, 2*pi, num_readings);
x1(1:num_readings) = robot.p(1);
y1(1:num_readings) = robot.p(2);
x2 = rad*cos(theta) + x1;       
y2 = rad*sin(theta) + y1;
lines(:, 1) = x1;
lines(:,2) = y1;
lines(:, 3) = x2;
lines(:,4) = y2;

%comment out to remove sensor line in plot
% for i = 1:length(x1)
%  set(s, 'visible','off')
%  s = plot([max(x1(i), course(1)); min(max(0,x2(i)), course(3))],[min(course(4),max(y1(i), course(2))); min(15,max(0,min(y2(i), course(4))))], 'k');
% 
%  set(s, 'visible', 'on')
%  pause(.001);
%  M(end+1) = getframe;
% end
[xlocations, ylocations, rs] = sensorSweep(lines,obst, robot, rad);

for i = 1:size(xlocations,1)
    x = xlocations(i,:);        %xlocations is the N1xN2 matrix where i,j is the x coordinate of the intersections between X1(i) and X2(j)
    y = ylocations(i,:);        %x,y are the coordinates of the intersection between sensor line and obstacles

    map(end+1).p = [x', y'];

end

for i = 1:length(map)
    
plot(map.p(:,1), map(i).p(:,2), 'k*', 'LineWidth', 3);    %plot the coordinates of the full map.
% M(end+1) = getframe;  
end
% M(end+1) = getframe;

% 
if length(map) < 1
   map(1).p = [0 0]; 
end
end
    




