function [T ] = RRT_plot( T)
%UNTITLED4 Summary of this function goes here
%   Detailed explanation goes here

for i = 1: length(T)
    if T(i).changed == 1
        p1 = T(i).p;
        p2 = T(T(i).iPrev).p;
        plot([p1(1); p2(1)],[p1(2); p2(2)],'b','LineWidth',2);
        T(i).changed = 0; 
    end
end



end

