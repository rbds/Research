%best paths for following road
figure(1)
bp_75 = [1    17    32    47    62    77    92   108   124   140   156   157   158   159   160   161   162   163   179   195   210   225]; 
bp_50 = [1    17    33    49    65    80    96   111   127   143   159   160   161   162   163   179   195   210   225];
bp_25 = [1    17    33    48    63    79    95   110   126   127   143   159   160   161   162   163   179   195   210   225];
bp_0 =  [ 1    17    33    48    63    78    93   109   125   126   127   143   159   160   161   162   163   179   195   210   225];

%best paths for avoiding road 
% figure(1)
% bp_75 = [1 17 32 47 62 77 92 108 124 140 156 157 158 159 160 161 162 163 179 195 210 225]; 
% bp_50 = [1 17 33 49 65 80 96 111 127 143 159 160 161 162 163 179 195 210 225];
% bp_25 = [1 17 32 47 62 77 92 108 124 110 111 127 143 159 160 161 162 163 179 195 210 225];
% bp_0 = [1 17 33 48 63 78 93 109 125 126 127 143 159 160 161 162 163 179 195 210 225];

color90 = [.75 .45 .35];

for i=1:length(bp_0)-1  %plot path
%     plot(coords(bp_90(i),1), coords(bp_90(i),2), '*', 'Color', color90)
   h0 =  plot([coords(bp_0(i),1), coords(bp_0(i+1),1)],[coords(bp_0(i),2), coords(bp_0(i+1),2)], '-', 'LineWidth', 4, 'Color', color90);    
end

for i=1:length(bp_25)-1  %plot path
%     plot(coords(bp_25(i),1), coords(bp_25(i),2), 'g*')
  h25 = plot([coords(bp_25(i),1), coords(bp_25(i+1),1)],[coords(bp_25(i),2), coords(bp_25(i+1),2)], 'g-', 'LineWidth', 4);    
end

for i=1:length(bp_50)-1  %plot path
%     plot(coords(bp_50(i),1), coords(bp_50(i),2), 'k*')
 h50 =    plot([coords(bp_50(i),1), coords(bp_50(i+1),1)],[coords(bp_50(i),2), coords(bp_50(i+1),2)], 'k-', 'LineWidth', 4);    
end

for i=1:length(bp_75)-1  %plot path
%     plot(coords(bp_75(i),1), coords(bp_75(i),2), 'b*')
  h75 =   plot([coords(bp_75(i),1), coords(bp_75(i+1),1)],[coords(bp_75(i),2), coords(bp_75(i+1),2)], 'b-', 'LineWidth', 4);    
end



H = [h0; h25; h50;  h75];
legend(H, '\epsilon = 0', '\epsilon = 0.25', '\epsilon = 0.50', '\epsilon = 0.75', 'Location', 'NorthWest')
figure
hold on
[c, p] = plot_paths( d, bp_0, cost, P_tr, coords );
[c, p] = plot_paths( d, bp_25, cost, P_tr, coords );
[c, p] = plot_paths( d, bp_50, cost, P_tr, coords );
[c, p] = plot_paths( d, bp_75, cost, P_tr, coords );
