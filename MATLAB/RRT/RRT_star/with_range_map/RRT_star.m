function [T ] = RRT_star( ~, obst, T, param )
       locked =1;
        T(end).changed = 0;
        T = pathcost(T, param);
    for i = 1: length(T)-1
        if norm(T(end).p - T(i).p) < param.RRTstarrad
            marg_cost = norm(T(end).p - T(i).p);
            line = [T(i).p(1) T(i).p(2) T(end).p(1) T(end).p(2)];
            collision = lineSegmentIntersect(line, obst);
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
    T = pathcost(T, param);
%     S = zeros(1, length(T));
%     
%  for i = 1:length(T)
%     S(i) = T(i).changed;
%  end

%     for i = 1:length(T)
       
%      end   


end

