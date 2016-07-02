function [ dist, loc ] = d_to_e( pos, sources )
%d_to_e find distance to nearest energy source.
%   Detailed explanation goes here

% dist = norm(repmat(robot.p(1:2)', size(sources,1), 1),  - sources);
dists = zeros(length(sources),1);
for kk = 1:length(sources)
   dists(kk) = norm(pos' - sources(kk,:)); 
end


[dist, m] = min(dists);
loc = sources(m,:);

end

