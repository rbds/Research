function col = InCollision_Node(rob,obst, p)

col = 0;
numobst = length(obst(1).ball);
    for j=1:numobst % check for robot-obstacle collision
        % calculate distance between ith robot ball center and jth obstacle
        % ball center
        %dist = sqrt((rob.ball{i}.p(1)-obst.ball{j}.p(1))^2+(rob.ball{i}.p(2)-obst.ball{j}.p(2))^2+(rob.ball{i}.p(3)-obst.ball{j}.p(3))^2);
        dist = norm(p-obst(1).ball{j}.p);
        if dist < (rob.r + obst(1).ball{1}.r)
            col = 1;
%             obst.ball{j}.p    %Display Obstical Coord
%             circle(obst.ball{j}.p(1,1),obst.ball{j}.p(2,1),(1/3),'b');
%              pause(0.1)
            return;
        end 
    end 
end