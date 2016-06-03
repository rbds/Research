function [ obst ] = add_obstacles( )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% close all


r = 0.25; %radius of trees
points = [4, 6; 
          4 6.75;
          4, 7.25;
          4.25, 7.75;
          3.5, 6.5;
          3.5, 7.25;
          3.75, 7.75;
          4.5 6.25;
          4.5 6.75;
          4.75 7.25;
          4 6.5;
          5 7;
          5.25 7.25;
          7.5 13.5; 
          7.5 12.75;
          7.5 14.5;
          6.5 13;
          6.5 14;
          7 13;
          7 14;
          12.5 2.5;
          13 2.5;
          13 3;
          13.5 2.5;
          13.5 1.75;
          13.5 3;
          13.25 3.75;
          14 1.25;
          14 2;
          12 12.5;
          12.5 13; 
          12.5 12;
          13.25 12.25;
          13 13;
          13 13.5;
          13.75 13;
          13.25 13.75;
          14 11.5; 
          14.25 13;
          14.25 12.25;
          15 12;
          14.75 13]; %centers of trees
    
obst = zeros(length(points),3);       
for ii = 1:length(points)
    obst(ii,:) = [points(ii,:), r];
end




end

