function [ map ] = buildMap( robot, obst, map )

%Sensor Sweep
num_readings = 10;
gapsize = 7;
theta = linspace(0, 6.2, num_readings);
x1(1:num_readings) = robot.p(1);
y1(1:num_readings) = robot.p(2);
rad = 75;
x2 = rad*cos(theta) + x1;       %rad is an arbitrarily large number. This may need to be changed to reflect course size
y2 = rad*sin(theta) + y1;
lines(:, 1) = x1;
lines(:,2) = y1;
lines(:, 3) = x2;
lines(:,4) = y2;
mindist = 100;

locked = [];
%  plot([x1;x2],[y1;y2])
[~, xlocations, ylocations] = sensorSweep(lines,obst);

%Find sensor line intersections, add them to the map.
for i = 1:size(xlocations,1)
    x = xlocations(i,:);        %xlocations is the N1xN2 matrix where i,j is the x coordinate of the intersections between X1(i) and X2(j)
    y = ylocations(i,:);        %x,y are the coordinates of the intersection between sensor line and obstacles
    z = cat(1,x,y);             %Z is the concat coordinates of the intersections of sensor line with each obstacles
    for k = 1:length(z)
        if xlocations(i,k) == 0
            d(k) = 100000000; %random big number
        else
        d(k) = norm(z(:,k) - robot.p);  %distance from robot.p to each obstacle intersection.
        end
    end
     closest = find(d== min(d));        %find closest obstacle for each sensor line.

       map(1, end +1)= x(closest); %add closest intersection to the map.
       map(2, end) = y(closest);
end

for i = 1:length(map)
plot(map(1,i), map(2,i), 'r*', 'LineWidth', 3);    %plot the coordinates of the full map.
end
% 
% 
% %Find nearest point on map
% % if size(map, 2) > num_readings +1;
% %  for i=1:size(map,2)
% for i=1:size(xlocations,1)  %for each new obstacle point
%     inew = size(map,2) - size(xlocations,1) + i; 
% %         inew = i;
%     for j = 2:size(map,2)   %iterate through all map points.
%         distance = norm(map(inew).p - map(j).p); 
%         if j==2 || distance < mindist  %Find closest obstacle point.
%             if distance ~= 0 %&& distance> .0001% If it is not checking against itself.
%                if isempty(find(locked == j, 1)) %check if it is already locked.
% 
%                 mindist = distance;   
%                 jmin = j; 
%                end
%             end
%         end
%     end
%         locked(end +1) = jmin;
%         map(inew).next = jmin;
%         plot([map(inew).p(1); map(map(inew).next).p(1)],[map(inew).p(2); map(map(inew).next).p(2)], 'g', 'LineWidth', 2);
% 
% 
%       if ~isempty(map(jmin).prev)
%         map(map(jmin).prev).next = inew;
%         map(inew).prev = map(jmin).prev;
%       end   
%         map(jmin).prev = inew;
% %       plot([map(inew).p(1); map(map(inew).next).p(1)], [map(inew).p(2); map(map(inew).next).p(2)], 'LineWidth', 3)
% end
% % gapcolor = [39 164 81]./256;
% % for i = 1:size(map,2)
% %     if norm(map(i).p - map(map(i).next).p) < gapsize
% %         plot([map(i).p(1); map(map(i).next).p(1)],[map(i).p(2); map(map(i).next).p(2)], 'r', 'LineWidth', 2); %plot obstacle borders
% %     else
% %         plot([map(i).p(1); map(map(i).next).p(1)],[map(i).p(2); map(map(i).next).p(2)], 'Color',gapcolor, 'LineWidth', 2);  %plot gaps
% %     end
% % end
% % end
% % 
% % %if this is the first time through, just connect points clockwise.
% %     if size(map,2) == (num_readings +1)
% %         for i = 2:size(xlocations,1)
% %             map(i).next = i+1;
% %             map(i+1).prev = i;
% %             plot([map(i).p(1); map(map(i).next).p(1)],[map(i).p(2); map(map(i).next).p(2)])
% %         %     plot([map(i).p(1); map(map(i).next).p(1)],[map(i).p(2); map(map(i).next).p(2)], 'LineWidth', 3)
% %         end
% %         map(end).next = 2;
% %         map(2).prev = size(map,2);
% %         plot([map(end).p(1); map(map(end).next).p(1)],[map(end).p(2); map(map(end).next).p(2)])
% %     end
% % end
% %     
% % %Built the forwards pointers.
% % 
% 
% 

