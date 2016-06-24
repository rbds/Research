function [ output_args ] =plot_maps(V, coords, costs, P_tr)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

subplot(1,2,1)

% figure
for i = 1:V
    v = [ coords(i,1)-0.5 coords(i,2)-0.5; coords(i,1)-0.5 coords(i,2)+0.5; coords(i,1)+0.5 coords(i,2)+0.5; coords(i,1)+0.5  coords(i,2)-0.5 ];
%     patch('Faces', [1 2 3 4], 'Vertices', v, 'FaceColor', [0 0 1], 'FaceAlpha', costs(i).^.5)
    patch('Faces', [1 2 3 4], 'Vertices', v, 'FaceColor', [1-costs(i).^.5 1-costs(i)^.5 1-costs(i)^.5])
end

axis off
axis equal
axis([0 16 0 16])
title('Cost Map')

shades = linspace(0, 1, 64);
newmap = [shades' shades' shades'];
colormap((newmap));   %activate it
% colorbar('location', 'southoutside', 'YDir', 'reverse')
h = colorbar('location', 'southoutside');
set(h, 'YDir', 'reverse')
subplot(1,2,2)

% figure
for i = 1:V
    v = [ coords(i,1)-0.5 coords(i,2)-0.5; coords(i,1)-0.5 coords(i,2)+0.5; coords(i,1)+0.5 coords(i,2)+0.5; coords(i,1)+0.5  coords(i,2)-0.5 ];
%     patch('Faces', [1 2 3 4], 'Vertices', v, 'FaceColor', [0 0 1], 'FaceAlpha', costs(i).^.5)
    patch('Faces', [1 2 3 4], 'Vertices', v, 'FaceColor', [1-P_tr(i).^2 1-P_tr(i).^2 1-P_tr(i).^2])
end

axis off
axis equal
axis([0 16 0 16])
title('P_{tr} Map')

shades = linspace(0, 1, 64);
newmap = [shades' shades' shades'];
colormap(flipud(newmap));   %activate it
h = colorbar('location', 'southoutside');
set(h, 'YDir', 'reverse')

end
