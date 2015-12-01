%deterministic RRT based algorithm

close all
figure
hold on
clear

P = [];
T = [];

p_start = [2.5; 2.5];
T(1).p = p_start;
T(1).iPrev = 1;
T(1).cost = 0;
T(1).changed = 0;
T(1).place = 1;
P(:,1) = p_start;
%create obstacles by code         
course = [0 0 45 33];
num_cells = course(3)*course(4)/5;
numberOfObstacles = 5;

% Parameters 
param.res = 0.25;            %Resolution of intermediate points. Must be <0.25 for InCollision_Edge to run.
param.thresh = 12;           %Bias towards the goal starting at this distance
param.maxiters =  250;      %Cap on iterations to run RRT
param.RRTstarrad = 25;      %Maximum length of lines redrawn by RRT*
param.goalbias = .95;        %Probability of checking the goal as p_new
param.maxpathlength = 100;    %Maximum length of any path segment.

% Draw Course    
obst = draw_obstacles(numberOfObstacles, course);
grid on
cost_map(1:7,1:9) = 0;
cost_map(2,2) = .4; cost_map(2,3) = 1; cost_map(2,4)=.3; cost_map(2,8)=.5;
cost_map(3,2) = .2; cost_map(3,3) = .8; cost_map(3,4)=.2; cost_map(3,6)=1; cost_map(3,7)=.2;
cost_map(4,6)=.2; cost_map(4,7)=.05;
cost_map(5,6)=.25; cost_map(5,7)=.9; cost_map(5,8)=.1;
cost_map(6,2)=.1; cost_map(6,3)=.1; cost_map(6,6)=.1; cost_map(6,7)=.4; 
cost_map(7,:)=.4;


% cost_map=flipud(cost_map);
% cost_map = cost_map';

% Set Start and Goal locations
p_start = [5;28];
p_goal = [41; 6];
p_sun = [40; 26];

for iter_x= 1:9     %iterate through each point in the map
 for iter_y = 1:7
     if (iter_x==1 && iter_y==1 )   %skip first iteration
         continue
     end
     l = [];
    for i=1:(length(T))                           %iterates through each valid node in tree 
      %distance = norm(T(i).p -  pts(iter_x,iter_y));                  %finds length between vertex and new coordinate
      cost = T(i).cost + norm(T(i).p - [iter_x*5-2.5;iter_y*5-2.5])*(1+cost_map(iter_y, iter_x));
      if (i==1) || (cost < mindist)               %1st iteration or distance is less than minimum dist
          mindist = cost;
            l = i;
            imin = i;
%           l(end+1,:) = [i, mindist];                           %chooses single closest point. k_nearest search
      end 
    end 
%     [~, imin] = min(l(:,2));
    %check for obstacle collision
    line = [iter_x*5-2.5, iter_y*5-2.5, T(l).p(1), T(l).p(2)];
    [col, distance2obst] = lineSegmentIntersect(line, obst);
%     a = abs(distance2obst) == min(abs(distance2obst));
    
    if ~isempty(find(col ==1, 1)) % skip to next iteration if not valid edge
        continue 
    end
    T = AddNode(T, [iter_x*5-2.5;iter_y*5-2.5], imin, cost_map); % add p to T with parent l
    plot([T(end).p(1);T(imin).p(1,1)],[T(end).p(2);T(imin).p(2,1)],'m','LineWidth',2); %Plot new edge on tree
    T = RRT_star(obst, T, param, cost_map);
    %RRT star replotting function
    T = RRT_plot(T);

 end
end






