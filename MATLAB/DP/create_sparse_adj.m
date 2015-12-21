clear
close all

P_tr_thresh = 0.4;

%build adjacency matrix
%square grid, connected right and down
s =2;
V = s^2;
i_vals = [];
j_vals = [];
% k_vals = [];

for i = 1:V    %letter
   if (mod(i,s) >0) %if it isn't on the right edge of grid 
    j_vals(end+1) = i; %add node to right
    i_vals(end+1) = i+1;
   end

   if (mod(i, s) ~=1) %if node isn't on the left edge of grid
    j_vals(end+1) = i; %add node to left
    i_vals(end+1) = i-1;
   end
   
   if (i> s) %if vertex isn't on top of grid
    j_vals(end+1) = i ;   %add next node up
    i_vals(end+1) = i - s;
       if (mod(i,s) >0) %if it isn't on the right edge of grid 
        j_vals(end+1) = i; %add node to diagonal right
        i_vals(end+1) = i+1 - s;
       end

       if (mod(i, s) ~=1) %if node isn't on the left edge of grid
        j_vals(end+1) = i; %add node to diagonal left
        i_vals(end+1) = i-1 -s;
       end
   end
   
   if (i<=(V- s))   %if vertex isn't on bottom of grid
    j_vals(end+1) = i ;  %add next node down
    i_vals(end+1) = i+s ;
       if (mod(i,s) >0) %if it isn't on the right edge of grid 
        j_vals(end+1) = i; %add node to diagonal right
        i_vals(end+1) = i+1+ s;
       end

       if (mod(i, s) ~=1) %if node isn't on the left edge of grid
        j_vals(end+1) = i; %add node to diagonal left
        i_vals(end+1) = i-1+s;
       end
    
   end

      
end

minimum = 0.9;
maximum = 1.0;
% vals = ones(length(i_vals),1);
vals = 10*abs(randn(length(i_vals), 1));  %generate random costs
% adj = sparse(i_vals, j_vals, vals, V^2, V^2);
a= sparse(i_vals, j_vals, vals);
adj = sparse(V^2, V^2);
for i=1:V-1
   adj((i-1)*V+1:(i-1)*V+V, i*V+1:i*V+V) = a;
end


%adj = [z a  z z; z z a z; z z z a; z z z z]; %

% adj = adj*10*abs(randn(size(adj)));
spy(adj)