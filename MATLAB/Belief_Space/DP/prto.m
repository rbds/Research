function [ front, inds ] = prto( a )
%prto Returns the Pareto front for EAPP.
%   First column of front should be cost, second column should be P_tr. The
%   front consists of the non-dominated entries with low cost and high
%   Probability of Traverse.

a(:,4) = [1:size(a,1)]';
front = a(1,:);

for i=2:size(a,1)
    flag = 0;
    spot = find(front(:,1) > a(i,1), 1);
    if (spot > 1) spot = spot - 1; end
    z = spot +1; 
    to_remove = [];
    while z <= size(front,1) %check to see if any further entries should be removed
        if a(i,2) > front(z,2)
           to_remove = [to_remove, z];  
           flag = 1; %set flag to indicate that the new node fits into the front
        end
        z = z+1;
    end
    
    if flag
       if (a(i,2) > front(spot,2)) && (a(i,1) < front(spot,1))
          to_remove = [to_remove, spot]; 
%           flag = 1;
       end
    end
    
    if a(i,2) > front(spot,2)
       flag = 1; %set flag to indicate that the new node fits into the front
    end
    
    front(to_remove, :) = [];
    
    if isempty(front) %check if front is now empty
       front = a(i,:);        
    else
        
        if isempty(spot) %check if it fits at the end
            if a(i,2) > front(end,2)
                front = [front; a(i,:)];
            end
        elseif spot ==1
         if a(i,1) < front(spot,1) %check if it fits at the top of the front
          if a(i,2) < front(spot,2)   
            front = [a(i,:); front]; 
          elseif a(i,2) > front(spot,2)
            front(spot,:) = a(i,:); 
          end
          flag = 0;
         end 
        end
    
        if flag
           if spot< size(front,1)
            front = cat(1, front(1:spot,:), a(i,:), front(spot+1:end, :)); 
           else
            front = cat(1, front(1:spot,:), a(i,:));
           end
        end
    end
    

end

inds = front(:,4);
front = front(:,1:3);


end

