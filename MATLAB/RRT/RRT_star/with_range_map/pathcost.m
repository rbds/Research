function [ T ] = pathcost( T, param )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

for i=1:length(T)
    T(i).cost = T(T(i).iPrev).cost + norm(T(i).p - T(T(i).iPrev).p);
    drained = param.drain*(T(T(i).iPrev).cost + norm(T(i).p - T(T(i).iPrev).p));
    T(i).SOC=1 - drained;
end

% path_cost = 0;
% for i=1:length(P) - 1
%     
%     path_cost = path_cost + norm(P(i) - P(i+1));
% 
% end