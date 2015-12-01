% Current RRT Script
%Original: University of Illinois
%Last Modified:  by Randy Schur

% %TO DO: 

% %ERROR CODES:
close all
figure
hold on
clear


%create obstacles by code         
course = [0 0 45 33];
numberOfObstacles = 5;

% Draw Course    
[obst, obst_ptsx, obst_ptsy] = draw_obstacles(numberOfObstacles, course);

% Set Start and Goal locations
p_start = [5;28];
p_goal = [41; 6];
p_sun = [24;15];
        th = 0:pi/50:2*pi;
        xunit = .5 * cos(th) + p_sun(1);
        yunit = .5 * sin(th) + p_sun(2);
        orange = [230 218 57]./256;
        plot(xunit, yunit,'LineWidth',3, 'Color', orange);
        drawnow
% Parameters 

goal.r = .5;            %radius of goal

circle(p_start(1,1),p_start(2,1),goal.r,'g');               %draw the location of x_start and x_goal
circle(p_goal(1,1),p_goal(2,1),goal.r,'g');

% pts = obst(:,1:2);
% pts(end + 1, :) = [15 25];
% pts(end +1, :) = p_start';
% pts(end +1, :) = p_goal';
% pts(end +1, :) = p_sun';

% %add start, goal, sun nodes to pts.
pts = p_start';
pts(end +1, :) = p_goal';
pts(end +1, :) = p_sun';
%randomly sample points throughout the open space
qx = find(obst_ptsx);
qy = find(obst_ptsy);
while length(pts) < 100
    new = rand(2,1);
   yes = inpolygon(new(1), new(2), qx, qy);
   if ~yes
     pts(end+1,:) = new';
   end
end


logical_graph = zeros(size(pts,1), size(pts, 1));
for j=1:length(pts)
    for i =1:length(pts)
       line(i,:) = cat(2, pts(j,:), pts(i,:));
    end
    
    [adj, ~] = lineSegmentIntersect(line,obst);
    for i=1:min(size(line))
        logical_graph(i) = all(adj(i,:) == 0);
    end
    for i = 1:length(pts)
       G(i,j) = logical_graph(i).*norm(pts(j,:) - pts(i,:)); 
    end

end
[opt_path, cost] = dijkstra2(100, G, length(pts) - 1, length(pts));
