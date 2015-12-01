function [ col, imin ] = collision( T, p_new, obst )

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
%     a = abs(distance2obst) == min(abs(distance2obst));
end

