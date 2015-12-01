
close all
figure(1)
hold on
clear


%create obstacles by code         
course = [-15 -20 30 13];
numberOfObstacles = 5;
s = [];
% Draw Course    
obst = draw_obstacles(numberOfObstacles, course);

% Set Start and Goal locations
p_start = [-10; 8];
% p_start = [-10;-15];%For skipping the sun
p_goal =  [26; -14];
map = [];
newmap = [];
landmarks(1).p = [];
l = zeros(10,10);

gapdist = 5;
% Parameters 
param.res = 0.25;            %Resolution of intermediate points. Must be <0.25 for InCollision_Edge to run.
param.thresh = 12;           %Bias towards the goal starting at this distance
param.maxiters = 100;      %Cap on iterations to run RRT
param.RRTstarrad = 15;      %Maximum length of lines redrawn by RRT*
param.goalbias = .95;        %Probability of checking the goal as p_new
param.maxpathlength = 20;    %Maximum length of any path segment.
%param.smoothiters = 150;    %Number of iterations for smoothing algorithm
goal.r = .5;            %radius of goal
robot.r = 0.75;
robot.t = 0;

robot.p = p_start;               %set robot position to x_start

circle(p_start(1,1),p_start(2,1),goal.r,'g');               %draw the location of x_start and x_goal
circle(p_goal(1,1),p_goal(2,1),goal.r,'g');

%Sensor Sweep
num_readings = 10;
gapsize = 7;
theta = linspace(0, 6.2, num_readings);
x1(1:num_readings) = robot.p(1);
y1(1:num_readings) = robot.p(2);
rad = 75;
x2 = rad*cos(theta) + x1;       %rad is an arbitrarily large number. This may need to be changed to reflect course size
y2 = rad*sin(theta) + y1;
lines(:, 1) = x1;
lines(:,2) = y1;
lines(:, 3) = x2;
lines(:,4) = y2;
mindist = 100;

locked = [];
%  plot([x1;x2],[y1;y2])
[~, xlocations, ylocations, distance2obst] = sensorSweep(lines,obst);

%Find sensor line intersections, add them to the map.
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

       newmap(1, end +1)= x(closest); %add closest intersection to the map.
       newmap(2, end) = y(closest);
end

for i = 1:length(newmap)
plot(newmap(1,i), newmap(2,i), 'r*', 'LineWidth', 3);    %plot the coordinates of the full map.
end

%For each new point, find the closest point in map
%If distance is > gapdist, new landmark
%Else if distance is < gapdist, add to existing landmark
mind = gapdist + 1; %
for i=1:length(newmap)
        if length(map) == 0 %first time through, Initialize map to set of new points.
          map = newmap;
            break
        end
    
    for j = 1:length(map)
        d = norm(newmap(:,i) - map(:,j));
        if j==1 || d< mind
          if j == i
             continue 
          end
           mind = d;
           j_d = j;
        end
    end

    %build adjacency matrix, add new data point to landmarks
    if mind < gapdist
        %add to existing landmark
        nt = find(l(:,j_d)>0);          %nt is landmark associated with data point.
        l(nt, (i) ) = 1;
        landmarks(nt).p(:, end+1) = i;
    elseif mind>= gapdist
        %add new landmark at map point i
        l(end + 1, (length(map) +i) ) = 1;
        landmarks(end +1).p(:,1) = newmap(:, i);
    end
end

map = cat(2,map,newmap);
