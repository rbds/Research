function [ object_coords, error ] = create_obstacles( obst_bottomleft, obst_sides)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
error = 0;
object_coords = [];

    for k = 1: length(obst_sides)
    %If obst_side2 = 0, object is interpreted as a horizontal line
     if obst_sides(2, k) == 0
        object_coords(end +1) = obst_bottomleft(1, k);
        object_coords(end +1) = obst_bottomleft(2, k);
            len = obst_sides(1,k)*3/2;
         for i = 1:len   %draw in the horizontal line
            object_coords(end +1) = obst_bottomleft(1,k) + (i*2/3);
            object_coords(end +1) = obst_bottomleft(2,k);
         end
     elseif obst_sides(1, k) ==0
         object_coords(end +1) = obst_bottomleft(1, k);
         object_coords(end +1) = obst_bottomleft(2, k);
            len = obst_sides(2,k)*3/2;
         for i = 1:len   %draw in the horizontal line
            object_coords(end +1) = obst_bottomleft(1,k);
            object_coords(end +1) = obst_bottomleft(2,k) + (i*2/3);
         end
     else
% 
%      %Otherwise, interpret as a square
            object_coords(end +1) = obst_bottomleft(1, k);
            object_coords(end +1) = obst_bottomleft(2, k);
            object_coords(end +1) = obst_bottomleft(1, k);
            object_coords(end +1) = obst_bottomleft(2, k) + obst_sides(2,k);
            len = obst_sides(1,k)*3/2;
         for i = 1:len   %draw in the horizontal line
            object_coords(end +1) = obst_bottomleft(1,k) + (i*2/3);
            object_coords(end +1) = obst_bottomleft(2,k);
            object_coords(end +1) = obst_bottomleft(1,k) + (i*2/3);
            object_coords(end +1) = obst_bottomleft(2,k) + obst_sides(2,k);
         end
         
         len = obst_sides(2,k)*3/2;
         for i = 1:len
            object_coords(end +1) = obst_bottomleft(1,k);
            object_coords(end +1) = obst_bottomleft(2,k) + (i*2/3);
            object_coords(end +1) = obst_bottomleft(1,k) + obst_sides(1,k);
            object_coords(end +1) = obst_bottomleft(2,k)+ (i*2/3) ;
         
         end

     end
    end

           object_coords(end +1) = 999;
  
    
end

