function [T ] = RRT_star( obst, T, param, cost_map)
%UNTITLED3 Summary of this function goes here
        T = pathcost(T, cost_map);
    for i = 1: length(T)-1
       if norm(T(end).p - T(i).p) < param.RRTstarrad
            marg_cost = norm(T(end).p - T(i).p)*(1 + cost_map(T(end).place));
            line = [T(i).p(1) T(i).p(2) T(end).p(1) T(end).p(2)];
            collision = lineSegmentIntersect(line, obst);
            if ~isempty(find(collision ==1,1))
                continue
            end
            
            if T(i).cost > (T(end).cost + marg_cost)*(1 + cost_map(T(end).place))
                T(i).iPrev = length(T);
                T(i).changed = 1;
            end
            if T(end).cost > (T(i).cost + marg_cost)*(1 + cost_map(T(end).place))
                T(end).iPrev = i;
                T(end).changed = 1;
            end
        end
      
    end
    T = pathcost(T, cost_map);
%     S = zeros(1, length(T));
    
end

