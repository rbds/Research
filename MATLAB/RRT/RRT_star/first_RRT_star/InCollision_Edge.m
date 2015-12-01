function col = InCollision_Edge(rob,obst,p1, p2,res) %makes sure robot doesn't decide to go into obstacle

col = 0;


    line = [p_new(1) l(1) p_new(2) l(2)];
    collision = lineSegmentIntersect(line, obst); 
    if ~isempty(find(collision ==1, 1)) % skip to next iteration if not valid edge
       col = 1;
    end
 end   
% d = norm(p1 - p2);         %Dist between 2 points
% m = ceil(d/res);           %distance/resolution
% t = linspace(0,1,m);       
%     
% for i=1:(m)
%         p = (1-t(i))*p1 + t(i)*p2; %calculate configuration
%         for k = 1:length(obst)
%             if norm(p - obst(k).center) < obst(k).rad
%             col = InCollision_Node(rob,obst, p); 
%             end
%             if col == 1
%                % plot(p(1), p(2), 'xr')
%                 return;
%             end
%         end
% end

