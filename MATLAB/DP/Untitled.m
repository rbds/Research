% clf
clear


a = 10*abs(randn(100,2));
% a = [1 10; 2 8; 2 7; 3 6; 4 8; 6 3; 1 11; 2 2; 6 1];

front1 = a(1,:); %front sorted by cost
front2 = a(1,:); %front sorted by P_tr
for i=2:length(a)
    [c, a] = pop(a);
    m1 = find(front1(:,1) > c(1), 1 ); %find first entry in front with cost greater than c
    m2 = find(front2(:,2) < c(2), 1 ); %find first entry with P_tr lower than c
    flag = 0;
    to_remove = [];
    if (~isempty(m1))
        for k=m1:size(front1,1)     %for all entries with a worse cost
            if front1(k,2) < c(2)   %if P_tr is also worse
               to_remove = [to_remove, k];
            end
            flag = 1;            %set flag to add current node into front
        end
        front1(to_remove,:) = [];    %remove entry from front
    end
    to_remove = [];
    if (~isempty(m2))
        for k=m2:size(front2,1)     %for all entries with a worse P_tr
            if front2(k,1) > c(1)   %if cost is also worse
               to_remove = [to_remove, k];
            end
            flag = 1;
        end
        front2(to_remove,:) = [];    %remove from front 
    end
    if flag     %if c was better than anything on the front, it is now on the front itself.
        front1 = [front1; c];
        front2 = [front2; c];
    end
    front1 = sortrows(front1); %re-sort (should be changed to inset in correct spot (m1, m2) - push function
    front2 = sortrows(front2, 2);
end

front1
front2