function [ T, E ] = pathcost( T, param, p_sun )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
E = T(1);
for i=1:length(T)
    T(i).cost = T(T(i).iPrev).cost + norm(T(i).p - T(T(i).iPrev).p);
    T(i).SOC  = T(T(i).iPrev).SOC - param.drain*norm(T(i).p - T(T(i).iPrev).p);
    if norm(T(i).p - p_sun) <.05
        T(i).SOC =1;
    end
    if T(i).SOC >.05
       if i>1
        E(end+1) = T(i);
        E(end).iPrev = i; %E.iPrev is the corresponding node in T.
       end
    end
end


% path_cost = 0;
% for i=1:length(P) - 1
%     
%     path_cost = path_cost + norm(P(i) - P(i+1));
% 
% end