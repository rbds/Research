function [ end_node, success ] = propogate(start_node, e_nearest, obs, constraint )
%Takes an edge and a starting belief node, returns a belief node at the
%ending vertex. 


 [~, state] = movement_simulation(e_nearest, start_node);
   p = state(:, 1:2);
   p_hat = state(:, 3:4);

%      [~, state] = movement_simulation(e);
%      p = state(:, 1:2);
%      p_hat = state(:, 3:4);
%      end_node.x = state(end, :);
for i= 1:size(pts,2)
        h = edge(1,i);
        k = edge(2,i);
        theta = 0;
        a = v.sigma(1)+abs(.1*randn(1));  %uncertainty in ellipse, plus some new uncertainty
        b = v.sigma(2)+abs(.1*randn(1));     
        collision(i) = ellipse(h,k,theta,a,b, constraint, obs); %#ok<AGROW>
end



    if isempty(collision(collision==1))
        success = 1;
    else
        success = 0;
    end


end