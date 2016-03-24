function [ c,p  ] = plot_paths( d, best_path, cost, Ptr )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here

x = 1:length(best_path)';
c = zeros(size(best_path));
p = zeros(size(best_path));
for i=x
    c(i) = cost - d{best_path(i)}(1);
    p(i) = d{best_path(i)}(2);
end

[ax, h1, h2] = plotyy(x, c,  x, p)
title('Path cost and Probability of Traverse')
xlabel('Path step number')
axes(ax(1)); ylabel('Cost (J)');
axes(ax(2)); ylabel('P_{tr}');

end

