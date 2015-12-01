function [ collide ] = ellipse( h, k, theta, a, b, constraint, obst )
%Draws an ellipse around the vertex, and checks if any obstacles are within
%that ellipse. if so, collide=1.
collide = 0;   
    for i=1:length(obst)
        x = obst(i,1);
        y = obst(i,2);
        val = ((x-h)*cos(theta)+(y-k)*sin(theta))^2/a^2 + ((x-h)*sin(theta)-(y-k)*cos(theta))^2/b^2;
        if val < 1-constraint 
           collide = 1;
        end
    end
    
end

