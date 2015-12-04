clf
clear all

% direction = 'outer';
direction = 'inner';

a = 10*abs(randn(100,2));
% a = [1 10; 2 8; 2 7; 3 6; 4 8; 6 3; 1 11; 2 2; 6 1];

front = a(1,:);

for i=2:length(a)
    switch direction
        case 'inner'
           % inner front
            list1 = find(front(:,1) >= a(i,1));
            list2 = find(front(:,2) >= a(i,2));
            to_remove = [];
            ind = [];

            for j=1:length(list1)
                ind = find(list1(j) == list2);
               if ~isempty(ind)
                to_remove(end+1) = ind;
               end
            end
            front(list2(to_remove),:) = [];

            if (length(list1) + length(list2) >0)
                t1 = front(:,1) <= a(i,1);
                t2 = front(:,2) <= a(i,2);
                if (isempty(find(t1==t2)))
                    front(end+1,:) = a(i,:);
                end
            end
        case 'outer'
            %outer front
            list1 = find(front(:,1) <= a(i,1));
            list2 = find(front(:,2) <= a(i,2));
            to_remove = [];
            ind = [];

            for j=1:length(list1)
                ind = find(list1(j) == list2);
               if ~isempty(ind)
                to_remove(end+1) = ind;
               end
            end
            front(list2(to_remove),:) = [];

            if (length(list1) + length(list2) >0)
                t1 = front(:,1) >= a(i,1);
                t2 = front(:,2) >= a(i,2);
                if (isempty(find(t1==t2)))
                    front(end+1,:) = a(i,:);
                end
            end
    end
end

plot(a(:,1), a(:,2), 'o')
hold on
[~, order] = sort(front(:,1));
front = front(order,:)
plot(front(:,1), front(:,2), 'r')
axis equal