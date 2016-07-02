function [ obst ] = draw_obstacles( num_obst, centers, course )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
% close all



     % % Uncomment here for random obstacles, and  comment out below.
obst = [];
nsides = 4; %number of sides in each obstacle
lsides = 1; %length of each side.
points = centers;%starting points
% 
% x = points(1, :) .*(course(3) - course(1)) + course(1);
% y = points(2, :) .*(course(4) - course(2)) + course(2);

x = points(1, :) ;
y = points(2, :) ;       
       
for v = 1:num_obst
    x0 = x(v);
    y0 = y(v);
    for w = 1: nsides
        phi = (180 - 180*(nsides - 2)/nsides)*(w-1);
        xn = x0 + lsides*cosd(phi);
        yn = y0 + lsides*sind(phi);
        obst(end +1, :) = [x0 y0 xn yn];
        x0 = xn;
        y0 = yn;
    end  
end

obst(end +1, :) = [course(1) course(2) course(3) course(2)];
obst(end +1, :) = [course(3) course(2) course(3) course(4)];
obst(end +1, :) = [course(3) course(4) course(1) course(4)];
obst(end +1, :) = [course(1) course(4) course(1) course(2)];


 hold on %uncomment for testing purposes.
    for z = 1:length(obst)
    plot([obst(z,1); obst(z,3)],[obst(z,2);obst(z,4)], 'k', 'LineWidth', 3)
    end
    
end

