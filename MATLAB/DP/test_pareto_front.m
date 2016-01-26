clear all

% direction = 'outer';
direction = 'inner';

a = abs(randn(100,2));
% a = [1 10; 2 8; 2 7; 3 6; 4 8; 6 3; 1 11; 2 2; 6 1];

front = a(1,:);

for i=2:length(a)
    flag = 0;
    spot = find(front(:,1) > a(i,1), 1);
    z = spot;
    to_remove = [];
    while z <= size(front,1)
        if a(i,2) > front(z,2)
           to_remove = [to_remove, z];  
        end
        z = z+1;
    end
    front(to_remove, :) = [];
    if isempty(front)
       front = a(i,:); 
    end
    
    if size(front,1) > spot
        front = cat(1, front(1:spot,:), a(i,:), front(spot+1:end, :));
        flag = 1;
    else 
        front = cat(1, front(1:spot,:), a(i,:));
    end
    
    if flag < 1
        spot = find(front(:,2) < a(i,2), 1);
        z = spot;
        to_remove = [];
        while z <= size(front,2)
            if a(i,1) < front(z,1)
               to_remove = [to_remove, z];  
            end
            z = z+1;
        end
        front(to_remove, :) = [];
        if isempty(front)
           front = a(i,:); 
        end

        if size(front,1) > spot
            front = cat(1, front(1:spot,:), a(i,:), front(spot+1:end, :));
        else 
            front = cat(1, front(1:spot,:), a(i,:));
        end
    end
end

front




























% for i=2:length(a)
%     switch direction
%         case 'inner'
%            % inner front
%             list1 = find(front(:,1) >= a(i,1));
%             list2 = find(front(:,2) >= a(i,2));
%             to_remove = [];
%             ind = [];
% 
%             for j=1:length(list1)
%                 ind = find(list1(j) == list2);
%                if ~isempty(ind)
%                 to_remove(end+1) = ind;
%                end
%             end
%             front(list2(to_remove),:) = [];
% 
%             if (length(list1) + length(list2) >0)
%                 t1 = front(:,1) <= a(i,1);
%                 t2 = front(:,2) <= a(i,2);
%                 if (isempty(find(t1==t2)))
%                     front(end+1,:) = a(i,:);
%                 end
%             end
%         case 'outer'
%             %outer front
%             list1 = find(front(:,1) <= a(i,1));
%             list2 = find(front(:,2) <= a(i,2));
%             to_remove = [];
%             ind = [];
% 
%             for j=1:length(list1)
%                 ind = find(list1(j) == list2);
%                if ~isempty(ind)
%                 to_remove(end+1) = ind;
%                end
%             end
%             front(list2(to_remove),:) = [];
% 
%             if (length(list1) + length(list2) >0)
%                 t1 = front(:,1) >= a(i,1);
%                 t2 = front(:,2) >= a(i,2);
%                 if (isempty(find(t1==t2)))
%                     front(end+1,:) = a(i,:);
%                 end
%             end
%     end
% end
% 
% plot(a(:,1), a(:,2), 'x')
% hold on
% [~, order] = sort(front(:,1));
% front = front(order,:)
% plot(front(:,1), front(:,2), 'r')
% axis equal