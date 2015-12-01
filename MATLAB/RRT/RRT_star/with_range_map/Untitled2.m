figure 
hold on
for i = 1:length(P) - 1 %Draw Path
      if (length(P) ~= 1)
      plot([P(1,i);P(1, i+1)],[P(2,i);P(2,i+1)], 'g', 'LineWidth', 3) 
      end
end
P=pull(P, T, p_start, p_goal, obst, param, i_goal);
obst = draw_obstacles(numberOfObstacles, course);
for i = 1:length(P) - 1 %Draw Path
      if (length(P) ~= 1)
      plot([P(1,i);P(1, i+1)],[P(2,i);P(2,i+1)], 'r', 'LineWidth', 3) 
      end
end