bp_90 = [1 17 32 47 62 77 92 107 123 139 155 171 172 158 159 160 176 192 208 224 225];
bp_75 = [1 17 32 47 62 77 92 107 123 139 140 156 157 173 158 144 160 176 192 208 224 225]; 
bp_50 = [1 17 32 47 62 77 92 108 124 110 111 127 143 144 160 176 192 208 224 225];
bp_25 = [1 17 32 47 63 79 95 111 127 143 144 160 176 192 208 224 225];
bp_0 =  [1 17 33 49 64 80 96 112 128 144 160 176 192 208 224 225];

color90 = [.75 .45 .35];

for i=1:length(bp_90)-1  %plot path
%     plot(coords(bp_90(i),1), coords(bp_90(i),2), '*', 'Color', color90)
    plot([coords(bp_90(i),1), coords(bp_90(i+1),1)],[coords(bp_90(i),2), coords(bp_90(i+1),2)], '-', 'LineWidth', 4, 'Color', color90)    
end

for i=1:length(bp_75)-1  %plot path
%     plot(coords(bp_75(i),1), coords(bp_75(i),2), 'b*')
    plot([coords(bp_75(i),1), coords(bp_75(i+1),1)],[coords(bp_75(i),2), coords(bp_75(i+1),2)], 'b-', 'LineWidth', 4)    
end

for i=1:length(bp_50)-1  %plot path
%     plot(coords(bp_50(i),1), coords(bp_50(i),2), 'k*')
    plot([coords(bp_50(i),1), coords(bp_50(i+1),1)],[coords(bp_50(i),2), coords(bp_50(i+1),2)], 'k-', 'LineWidth', 4)    
end

for i=1:length(bp_25)-1  %plot path
%     plot(coords(bp_25(i),1), coords(bp_25(i),2), 'g*')
    plot([coords(bp_25(i),1), coords(bp_25(i+1),1)],[coords(bp_25(i),2), coords(bp_25(i+1),2)], 'g-', 'LineWidth', 4)    
end

legend('\epsilon = 0.9', '\epsilon = 0.75', '\epsilon = 0.50', '\epsilon = 0.25', 'Location', 'NorthWest')