function [ P, T] = PathPlan( robot,obst,param,p_start,p_goal, course)
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

locked = 1;
ma = 0; %mission accomplished flag
cost = 0;
%  M(end+1) = getframe; %Create movie object

while iter < param.maxiters
   p = rand(2,1); 
   p_new(1) = p(1)*(course(3) - course(1)) + course(1);
   p_new(2) = p(2)*(course(4) - course(2)) + course(2);

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
  
    if ~isempty(find(col ==1, 1)) % skip to next iteration if not valid edge
          iter = iter + 1;
        continue 
    end     
    
    T = AddNode(T,p_new,imin); % add p to T with parent l
%     plot([T(end).p(1);T(imin).p(1,1)],[T(end).p(2);T(imin).p(2,1)],'m','LineWidth',2); %Plot new edge on tree

    drawnow
%       M(end + 1) = getframe;
    
      T = RRT_star( obst, T, param);
    %RRT star replotting function
%       T = RRT_plot(T);
      
%       drawnow
%      M(end +1) = getframe;
    
  if ma>=1
      Q = [];
      i = i_goal;
      while i>1
         Q(:, end+1) = T(i).p;
         i = T(i).iPrev;
      end
      Q(:, end+1) = p_start;
    P = fliplr(Q);  
    
    
  end

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
%         plot([p_new(1);p_goal(1)],[p_new(2);p_goal(2)],'m','LineWidth',2);
%         drawnow
%          M(end +1) = getframe;    
    end
  end
  
  iter = iter + 1;
 end
end



