function [xloc, yloc] = sensorSweep(line,obst, robot, rad)
xloc = [];
yloc = [];

X =line(:,3);
Y = line(:,4);
n = length(obst);

for ii=1:n
    d = norm(robot.p - obst(ii,1:2));
    if d < obst(ii,3)+rad
       xloc(end+1) = obst(ii,1);
       yloc(end+1) = obst(ii,2);
    end
    
end

end

