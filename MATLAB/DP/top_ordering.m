V = 4;
rows_to_do = zeros(V,1);

% [row, col] = find(adj);

% p = abs(pascal(V,1)');
% p_pr = p';
% 
% for i=2:V^2
%    rows_to_do(i) = V^2 - p(i+1) - p(i;
% end
% 
% rows_to_do'

row = 0;
col = 0;
for i=1:V
    for j =i:-1:1
      row(end+1) = j-1;  
    end
end
for i=1:V
    for j =1:i
      col(end+1) = j-1;  
    end
end

for ind=1:V
   rows_to_do(ind) =  V^2 - row(ind) - V*col(ind);
end
rows_to_do