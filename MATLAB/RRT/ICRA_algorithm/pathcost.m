function [ T ] = pathcost( T, cost_map )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

for i=1:length(T)
    T(i).cost = T(T(i).iPrev).cost + norm(T(i).p - T(T(i).iPrev).p)*(1+cost_map(T(i).place));
end

% path_cost = 0;
% for i=1:length(P) - 1
%     
%     path_cost = path_cost + norm(P(i) - P(i+1));
% 
% end