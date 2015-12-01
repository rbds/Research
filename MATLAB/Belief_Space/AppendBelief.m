function [ V, success ] = AppendBelief( V, n_a, n )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
success = [];
for i=1:size(V,2)
   n_b = V{n(1), i};

   if isfield(n_b, 'cost')
   sigma_a = sqrt(n_a.sigma(1)^2+n_a.sigma(2)^2);
   sigma_b = sqrt(n_b.sigma(1)^2+n_b.sigma(2)^2) + .0001;
   cost_a = n_a.cost;
   cost_b = n_b.cost;
       dominates = sigma_a < sigma_b & cost_a < cost_b; 
       if dominates
          V{n(1), i} = []; 
          success = 1;
       end
       
       dominated = sigma_b < sigma_a & cost_b < cost_a;
       if dominated
           if isempty(success)
                success = 0; 
           end
       end
   end
end
if isempty(success)
   success =1; 
end

end

