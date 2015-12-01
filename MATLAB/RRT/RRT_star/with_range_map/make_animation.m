function [ ] = make_animation( Movie)
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here
clear mex
movie2avi(Movie, 'RRT_demo_sun_sensor', 'FPS',10 , 'Compression', 'none') 

end

