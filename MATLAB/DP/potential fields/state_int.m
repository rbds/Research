function [ robot ] = state_int( robot, F, dt )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

robot.p = robot.p + dt*F';
end

