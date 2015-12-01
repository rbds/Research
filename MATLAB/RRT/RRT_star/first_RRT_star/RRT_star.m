function [T ] = RRT_star( obst, T, param)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
        T(end).changed = 0;
        T = pathcost(T);
    for i = 1: length(T)-1
        %T(i).changed = 0;
     
        if norm(T(end).p - T(i).p) < param.RRTstarrad
            marg_cost = norm(T(end).p - T(i).p);
            line = [T(i).p(1) T(i).p(2) T(end).p(1) T(end).p(2)];
            collision = lineSegmentIntersect(line, obst);
%             collision = InCollision_Edge(rob, obst, T(end).p, T(i).p, param.res);
            if ~isempty(find(collision ==1,1))
                continue
            end
            
            if T(i).cost > (T(end).cost + marg_cost)
                T(i).iPrev = length(T);
                T(i).changed = 1;
            end
            if T(end).cost > (T(i).cost + marg_cost)
                T(end).iPrev = i;
                T(end).changed = 1;
            end
        end
      
    end
    T = pathcost(T);
    S = zeros(1, length(T));
    
 for i = 1:length(T)
    S(i) = T(i).changed;
 end
% 
%     for i = 1:length(T)
%        if S(i)  == 1 && ~isempty(S(i))
%        if isempty(find(locked ==i, 1))   
%            
%          for k = 1:length(T) 
%           if norm(T(i).p - T(k).p) < param.RRTstarrad && norm(T(i).p - T(k).p) > 0.1
%             marg_cost = norm(T(k).p - T(i).p);
%             line = [T(end).p(1) T(end).p(2) T(i).p(1) T(i).p(2)];
%             collision = lineSegmentIntersect(line, obst);
% %             collision = InCollision_Edge(rob, obst, T(end).p, T(i).p, param.res);
%             if ~isempty(find(collision ==1,1))
%                 continue
%             end
%                 if T(i).cost > (T(k).cost + marg_cost)
%                     T(i).iPrev = k;
%                     T(i).changed = 1;
%                     S(i) = S(i) + 1;
%                 end
%                   if T(k).cost > (T(i).cost + marg_cost)
%                       T(k).iPrev = i;
%                       T(k).changed = 1;
%                       S(k) = 1;               
%                   end
%            end
%         end
%       end
% 
%        end
%      end   


end

