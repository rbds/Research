clear
% close all
figure
hold on

%create obstacles by code      
% course = [-15 -20 30 13];
course = [0 0 45 33];
numberOfObstacles = 5;
s = [];
% Draw Course    
obst = draw_obstacles(numberOfObstacles, course);

% Set Start and Goal locations
% p_start = [-10; 8];
% p_start = [-10;-15];%For skipping the sun
% p_goal =  [26; -14];
p_start = [5;28];
p_goal = [41; 6];
goal.r = .5;            %radius of goal

circle(p_start(1,1),p_start(2,1),goal.r,'g');               %draw the location of x_start and x_goal
circle(p_goal(1,1),p_goal(2,1),goal.r,'g');

pts = obst(:,1:2);
pts(end + 1, :) = [15 25];
pts(end +1, :) = p_start';
pts(end +1, :) = p_goal';


for j=1:length(pts)
    for i =1:length(pts)
       line(i,:) = cat(2, pts(j,:), pts(i,:));
    end
    
    [adj, ~] = lineSegmentIntersect(line,obst);
    for i=1:length(line)
        logical_graph(i) = all(adj(i,:) == 0);
    end
    for i = 1:length(pts)
       G(i,j) = logical_graph(i).*norm(pts(j,:) - pts(i,:)); 
    end

end

[opt_path, cost] = dijkstra_kyle(length(pts) - 1, length(pts), G);
for i = 1: length(opt_path)-1
   plot([pts(opt_path(i),1);pts(opt_path(i+1),1)],[pts(opt_path(i),2);pts(opt_path(i+1),2)], 'g', 'LineWidth', 1) 
end

% for i =1:length(pts)
%    line(i,:) = cat(2, pts(1,:), pts(i,:));
% end
% adj = lineSegmentIntersect(line,obst);
% logical_graph(1,:) = adj(:,1) == 0;
% for i = 1:length(pts)
%    G(:,i) = logical_graph(i).*norm(pts(1,:) - pts(i,:)); 
% end

