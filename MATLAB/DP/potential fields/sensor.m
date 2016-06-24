function [ map, s ] = sensor( robot, obst, map, s, rad, course)

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

for i = 1:length(x1)
 set(s, 'visible','off')
 s = plot([max(x1(i), course(1)); max(course(1),min(x2(i), course(3)))],[max(y1(i), course(2)); min(y2(i), course(4))], 'k');

 set(s, 'visible', 'on')
 pause(.001);
%  M(end+1) = getframe;
end
[~, xlocations, ylocations] = sensorSweep(lines,obst);

for i = 1:size(xlocations,1)
    x = xlocations(i,:);        %xlocations is the N1xN2 matrix where i,j is the x coordinate of the intersections between X1(i) and X2(j)
    y = ylocations(i,:);        %x,y are the coordinates of the intersection between sensor line and obstacles
    z = cat(1,x,y);             %Z is the concat coordinates of the intersections of sensor line with each obstacles
    for k = 1:length(z)
        if xlocations(i,k) == 0
            d(k) = 100000000; %random big number
        else
        d(k) = norm(z(:,k) - robot.p);  %distance from robot.p to each obstacle intersection.
        end
    end
     closest = find(d== min(d));        %find closest obstacle for each sensor line.
    if size(closest,2) == 1
           map(end +1).p(1,1) = x(closest); %add closest intersection to the map.
           map(end).p(2,1) = y(closest);
    end
end

for i = 1:length(map)
    
plot(map(i).p(1), map(i).p(2), 'r*', 'LineWidth', 3);    %plot the coordinates of the full map.
% M(end+1) = getframe;  
end
% M(end+1) = getframe;

% 

end
    




