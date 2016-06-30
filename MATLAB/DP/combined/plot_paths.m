function [ c,p  ] = plot_paths( d, best_path, costs, P_tr, coords )
%UNTITLED2 Summary of this function goes here
%   Detailed explanation goes here



x = 1:length(best_path)';
% c = zeros(size(best_path));
% p = zeros(size(best_path));
% for i=x
%     c(i) = cost - d{best_path(i)}(1);
%     if i> 1
%         p(i) = p(i-1)*P_tr(best_path(i));
%     else
%        p(i) = 1 ;
%     end
% end
c = cumsum(costs(best_path));
p = cumprod(P_tr(best_path));
% [ax, h1, h2] = plotyy(x, c,  x, p);

% figure
subplot(1,2,1)
hold on
plot(x, c, 'LineWidth', 3)
title('Estimated Path Cost')
xlabel('Path step number')
ylabel('Path cost (J)')

subplot(1,2,2)
hold on
plot(x, p, 'LineWidth', 3)
xlabel('Path step number')
title('Probability of Traverse')
axis([0 max(x) 0 1])
% axes(ax(1));  ylabel('Cost (J)');
% axes(ax(2)); axis([0 25 0 1]); ylabel('P_{tr}');


end

