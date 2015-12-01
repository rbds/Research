function [ P] = pull(P, ~, p_start, p_goal, obst, param, i_goal)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

       plot([p_start(1); p_goal(1)],[p_start(2);p_goal(2)],'r')
       m=(p_goal(2)-p_start(2))/(p_goal(1)-p_start(1));
       b1 = p_start(2) - p_start(1)*m;
       %line from start to goal is y= mx+b1
       slope= -1/m; %perpendicular slope
       P= fliplr(P);    %go through P in reverse order. This ensures that path is feasible.
       for i=2:length(P)-1      %skip first and last entry in P, because they are already start and goal points
           b2= P(2,i)-P(1,i)*slope;
           x = (b2-b1)/(m-slope); %x coordinate of intersection
           y = m*x+b1;            %y coordinate of intersection
       
           x_pts = linspace(x, P(1,i), 10);  %change this 5 to a higher value to get more resolution
           y_pts = linspace(y, P(2,i), 10);
        plot(x_pts,y_pts, 'c*')
          for j= 1:length(x_pts) 
               p_new = [x_pts(i); y_pts(i)];
              line = [p_new(1) p_new(2) P(1, i-1) P(2, i-1)];
%               line(2,:) = [p_new(1) p_new(2) P(1, i+1) P(2, i+1)];
            [col, ~] = lineSegmentIntersect(line, obst);
             if ~isempty(find(col ==1, 1)) % skip to next iteration if not valid edge
                continue 
             end
             P(1:2, i) = p_new;
%            T = AddNode(T,p_new,imin); % add p to T with parent l
%            T = RRT_star(0, obst, T, param);
%             %RRT star replotting function
%            T = RRT_plot(T);
           break
          end
       end

P=fliplr(P);
%           Q = [];
%           i = i_goal;
%           while i>1
%              Q(1:2, end+1) = T(i).p;
%              Q(3, end) = i;
%              i = T(i).iPrev;
%           end
%           Q(1:2, end+1) = p_start;
%           Q(3, end) = 1;
%         P = fliplr(Q);


