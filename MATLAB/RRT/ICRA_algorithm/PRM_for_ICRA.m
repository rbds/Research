% ICRA PRM

% close all
figure
hold on
%  clear


%create obstacles by code         
course = [0 0 45 33];
numberOfObstacles = 5;

% Draw Course    
[obst, obst_ptsx, obst_ptsy] = draw_obstacles(numberOfObstacles, course);
grid on
cost_map(1:7,1:9) = 0;
cost_map(2,2) = .4; cost_map(2,3) = 1; cost_map(2,4)=.3; cost_map(2,8)=.5;
cost_map(3,2) = .2; cost_map(3,3) = .8; cost_map(3,4)=.2; cost_map(3,6)=1; cost_map(3,7)=.2;
cost_map(4,6)=.2; cost_map(4,7)=.05;
cost_map(5,6)=.25; cost_map(5,7)=.9; cost_map(5,8)=.1;
cost_map(6,2)=.1; cost_map(6,3)=.1; cost_map(6,6)=.1; cost_map(6,7)=.4; 
cost_map(7,:)=.4;


% Set Start and Goal locations
% p_start = [5;28]';
% p_goal = [41; 6]';
% p_sun = [24;15]';
p_start = [2.5;2.5]';
p_goal = [32.5; 32.5]';
p_sun = [12.5;22.5]';
%         
% Parameters 
goal.r = .5;            %radius of goal
circle(p_start(1,1),p_start(1,2),goal.r,'g');               %draw the location of x_start and x_goal
circle(p_goal(1,1),p_goal(1,2),goal.r,'g');  
circle(p_sun(1,1),p_sun(1,2),goal.r,'y');

cost = [0,0];
grid_pts = [];
%choose points
for i =1:7
    for j = 1:9
%         cost(end+1,:) =  [i*5-2.5;j*5-2.5]*(1+cost_map(i, j));
        grid_pts(end+1,:) = [i*5-2.5;j*5-2.5];
    end
end
cost_map(1:7,1:9) = 0;
cost_map(11) = .4; cost_map(12) = 1; cost_map(13)=.3; cost_map(17)=.5;
cost_map(20) = .2; cost_map(21) = .8; cost_map(22)=.2; cost_map(24)=1; cost_map(25)=.2;
cost_map(33)=.2; cost_map(34)=.1;
cost_map(42)=.3; cost_map(43)=.9; cost_map(44)=.2;
cost_map(47)=.15; cost_map(48)=.15; cost_map(51)=.2; cost_map(52)=.4; 
cost_map(54:end)=.4;
% cost_map(11,1) = 1;
% flipped_cmap = flipud(cost_map);
flipped_cmap = cost_map;
for i = 1:size(grid_pts,1)
     v = [ grid_pts(i,2)-2.5 grid_pts(i,1)-2.5; grid_pts(i,2)-2.5 grid_pts(i,1)+2.5; grid_pts(i,2)+2.5 grid_pts(i,1)+2.5; grid_pts(i,2)+2.5  grid_pts(i,1)-2.5 ];
    patch('Faces', [1 2 3 4], 'Vertices', v, 'FaceColor', 'black', 'FaceAlpha', flipped_cmap(i))
end

% grid_pts = [grid_pts; p_sun'; p_start'; p_goal'];
adj_mat = zeros(length(grid_pts)+3);

for i=1:length(grid_pts)-1
%     for j = 1:length(grid_pts)-1          
        if i>1
            adj_mat(i,i-1) =  (1+cost_map(i)+ cost_map(i-1))*5;
        end
        if i<length(grid_pts)-1
            adj_mat(i,i+1) =  (1+cost_map(i)+ cost_map(i+1))*5;
        end
        if i>9
            adj_mat(i,i-9)=  (1+cost_map(i)+ cost_map(i-9))*5;
        end
        if i<length(grid_pts)-9
            adj_mat(i,i+9)=  (1+cost_map(i)+ cost_map(i+9))*5;
        end
        if i<length(grid_pts)-8
            adj_mat(i,i+8) = (1+cost_map(i)+ cost_map(i+8))*7.07;
        end
        if i<length(grid_pts)-10
            adj_mat(i,i+10)= (1+cost_map(i)+ cost_map(i+10))*7.07;
        end
        if i>8
            adj_mat(i,i-8) = (1+cost_map(i)+ cost_map(i-8))*7.07;
        end
        if i>10
            adj_mat(i,i-10)= (1+cost_map(i)+ cost_map(i-10))*7.07;
        end
end
% grid_pts = [grid_pts; p_sun; p_start; p_goal];
% for i = 1:length(grid_pts)-3
%     dist = norm(grid_pts(i,:) - p_sun);
%    if dist< 10
%        adj_mat(i,end-2) = (1+cost_map(i))*dist;
%    end
%        dist = norm(grid_pts(i,:) - p_start);
%    if dist< 10
%        adj_mat(i,end-1) = (1+cost_map(i))*dist;
%    end
%        dist = norm(grid_pts(i,:) - p_goal);
%    if dist< 10
%        adj_mat(i,end) = (1+cost_map(i))*dist;
%    end
% end



%check for collisions
for i=1:size(grid_pts,1)
    for j=1:size(grid_pts,1)
    line = [grid_pts(i,2) grid_pts(i,1) grid_pts(j,2) grid_pts(j,1)];
    [col, distance2obst] = lineSegmentIntersect(line, obst);
        if ~isempty(find(col ==1, 1)) % skip to next iteration if not valid edge
              adj_mat(i,j) = 0; 
        end
    end
end

% [path, totalCost] = dijkstra2(length(grid_pts),adj_mat', length(grid_pts)-1, length(grid_pts))
[path, totalCost] = dijkstra2(length(grid_pts),adj_mat', 1, 61)
 
 %plot path
 for i=1:length(path)-1
%     plot(grid_pts(path(i),2),  grid_pts(path(i),1), 'xr')
    plot([grid_pts(path(i),2); grid_pts(path(i+1),2)],[grid_pts(path(i),1); grid_pts(path(i+1),1)], 'LineWidth', 2)
 end
 
