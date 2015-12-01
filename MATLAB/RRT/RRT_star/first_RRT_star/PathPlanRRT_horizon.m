function [ P, T, iter, robot] = PathPlanRRT_horizon( robot,obst,param,p_start,p_goal, course)
P = [];
T = [];
%map = [];
%map(:, 1).p = [course(1) course(2)]';

p_new = p_start;
T(1).p = p_start;
T(1).iPrev = 1;
T(1).cost = 0;
T(1).changed = 0;
P(:,1) = p_start;
iter = 1;
robot.p_iter = 1;
robot.p = p_start;
h = draw_robot(robot);
locked = 1;
ma = 0; %mission accomplished flag
horizons = 0;
mind2g = norm(p_start - p_goal);
%  M(end+1) = getframe; %Create movie object

% axis([-20 40 -30 20])

% map = buildMap(robot, obst, map); %start building Map

% while iter < param.maxiters
while robot.p ~= p_goal        %While the robot position is not the goal.

 if mod(iter, 30) == 0        %Every 10th iteration move the robot
    dists = zeros(1,length(T));
        for i = 1:length(T)
            dists(i) = norm(T(i).p - p_goal);
        end
        m = min(dists);
        i_close = find(dists == m); %find the iteration that is closest to the goal.
        i = i_close;
       %construct current path:
       Q = [];
       while i~= robot.p_iter
        Q(:, end +1) = T(i).p;    
        i = T(i).iPrev;
       end 
      P = cat(2, P, fliplr(Q)); %Cumulative path
       figure
%        axis([-20 40 -30 20])
       hold on
       draw_obstacles( 1, 1, obst); %redraw obstacles in.
       for i = 1:length(P) - 1 %Draw Path so far.
          if (length(P) ~= 1)
          plot([P(1,i);P(1, i+1)],[P(2,i);P(2,i+1)], 'g', 'LineWidth', 3) 
          end
       end
       
       plot(p_start(1,1), p_start(2,1), 'go', 'LineWidth', 3)
       plot(p_goal(1,1), p_goal(2,1), 'go', 'LineWidth', 3)
%        %draw the sun.
%        p_sun = [5; -7];
%             th = 0:pi/50:2*pi;
%             xunit = 2 * cos(th) + p_sun(1);
%             yunit = 2 * sin(th) + p_sun(2);
%             orange = [230 218 57]./256;
%             plot(xunit, yunit,'LineWidth',3, 'Color', orange);
%             drawnow
%        circle(p_start(1,1),p_start(2,1), 0.5,'g');               %draw the location of x_start and x_goal
%        circle(p_goal(1,1),p_goal(2,1), 0.5,'g');
       
       
          horizons = horizons +1; %Keep track of how many times process repeated.
           iter = 1;
           T = T(i_close);      %delete tree
        T(1).iPrev = 1;   %reset some variables.
        T(1).cost = 0;
        T(1).changed = 0;
     robot.p = P(:,end);  %move robot to the end of the current path.

%      map = buildMap(robot, obst, map);
        %set(h, 'Visible', 'off')
        h = draw_robot(robot);
        %set(h, 'Visible', 'on')
        drawnow     
        if robot.p == p_goal
            break
        end
%         M(end +1) = getframe;
  end     %end of restarting the tree

    
   p = rand(2,1); 
         r_ = p(1)*param.maxpathlength;
         theta_ = p(2)*2*pi;
         p_new(1,1) = robot.p(1) + r_ * cos(theta_);
         p_new(2,1) = robot.p(2) + r_ * sin(theta_);

while p_new(1,1) <course(1) || p_new(1,1) > course(3)   %Make sure the new point is within the bounds of the course
         p = rand(2,1);
             r_ = p(1)*param.maxpathlength;
             theta_ = p(2)*2*pi;
             
         p_new(1,1) =robot.p(1) + r_ * cos(theta_);
end
while p_new(2,1) < course(2) || p_new(2,1) > course(4)
        p = rand(2,1); 
             r_ = p(1)*param.maxpathlength;
             theta_ = p(2)*2*pi;    
        p_new(2,1) =robot.p(2) + r_ * sin(theta_);
end

 %     goal biasing with a certain probability
    n = rand(1);
    if n> param.goalbias
        p_new = p_goal;
    end
    
    % do something if valid coordinate
    for i=1:(length(T))                            %iterates through each valid node 
      distance = norm(T(i).p -  p_new);                  %finds length between vertex and new coordinate
        if (i==1) || (distance < mindist)               %1st iteration or distance is less than minimum dist
            mindist = distance;
            imin = i;
            l = T(i).p;                           %chooses single closest point. Consider changing this to a k-nearest search
        end 
    end
    
    line = [p_new(1) p_new(2) l(1) l(2)];
    [col, distance2obst] = lineSegmentIntersect(line, obst);
    a = abs(distance2obst) == min(abs(distance2obst));
    c.d2o = abs(distance2obst(a)* norm([line(1);line(3)] - [line(2); line(4)])); %find the distance to the closest obstacle
    
%     for i=1:length(T) %find quality numerator (distance cost to goal)
%         c.d2g = norm(p_goal - p_new) + norm(p_new - l) + T(imin).d2g;
%     end
%     if c.d2g < mind2g   %if there is a new minimum distance to the goal
%         mind2g = c.d2g;
%     end
    
    if ~isempty(find(col ==1, 1)) % skip to next iteration if not valid edge
       % circle(p_new(1),p_new(2),0.1,'r');     %plot a red dot for a failed point
        iter = iter + 1;
        continue 
    end
    
%     %cost structure
%     c.d2o %distance to nearest obstacle
%     c.d2g %distance to goal
%     c.SOC %SOC = distance from last point * charge/distance
%     c.d2e %distance to energy source
%     %tuning parameters
%     alpha
%     beta
%     gamma
        
    
    T = AddNode(T,p_new,imin); % add p to T with parent l
    plot([T(end).p(1);T(imin).p(1,1)],[T(end).p(2);T(imin).p(2,1)],'m','LineWidth',2); %Plot new edge on tree

    drawnow
%       M(end + 1) = getframe;
    
      T = RRT_star(robot, obst, T, param, locked);
%     %RRT star replotting function
      T = RRT_plot(T);
      
      drawnow
%      M(end +1) = getframe;
    
  if ma <1   
    %check robot position against x_goal if goal hasn't already been found.
    distance = norm(p_new-p_goal);        %computes distance from vertex to goal
    if (distance < param.thresh)            %if branch distance is less than the "close" distance
        line = [p_new(1) p_new(2) p_goal(1) p_goal(2)]; %check for valid edge
        col = lineSegmentIntersect(line, obst);
        if ~isempty(find(col == 1, 1)) % skip to next iteration if not valid edge
            iter = iter + 1;
            continue 
        end
     % add qgoal to T with parent q and exit with success
        T = AddNode(T,p_goal,length(T));
         ma = 1;
        i_goal = length(T);
        T(end).changed = 0;
        plot([p_new(1,1);p_goal(1,1)],[p_new(2,1);p_goal(2,1)],'m','LineWidth',2);
        drawnow
%          M(end +1) = getframe;     
    end
  end
  
  iter = iter + 1;
 end
end



