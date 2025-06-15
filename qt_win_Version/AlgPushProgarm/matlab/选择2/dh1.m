%% 动画演示
figure;
axis equal;
xlim([-500, leader_traj(end,1)+500]);
ylim([-500, 500]);
xlabel('X'); ylabel('Y'); grid on;
title('无人艇编队 + DWA 避障动画');
hold on;

colors = lines(N);
traj_lines = gobjects(N,1);
boat_markers = gobjects(N,1);
for i = 1:N
    traj_lines(i) = plot(nan, nan, '-', 'Color', colors(i,:), 'LineWidth', 1.2);
    boat_markers(i) = plot(nan, nan, 'o', 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');
end
leader_line = plot(nan, nan, 'k--', 'LineWidth', 1.5);
leader_marker = plot(nan, nan, 'ks', 'MarkerFaceColor', 'y');

% 绘制障碍物
for obs = obstacles
    if obs.type == "circle"
        viscircles([obs.x, obs.y], obs.r, 'Color', 'r');
    else
        rectangle('Position',[obs.x-obs.w/2, obs.y-obs.h/2, obs.w, obs.h], ...
                  'EdgeColor','r','LineWidth',1.5);
    end
end

for k = 1:1:steps
if k==1
 pause(10);
end

    for i = 1:N
        x = squeeze(history(i,1:k,1));
        y = squeeze(history(i,1:k,2));
        set(traj_lines(i), 'XData', x, 'YData', y);
        set(boat_markers(i), 'XData', x(end), 'YData', y(end));
    end
    set(leader_line, 'XData', leader_traj(1:k,1), 'YData', leader_traj(1:k,2));
    set(leader_marker, 'XData', leader_traj(k,1), 'YData', leader_traj(k,2));
    pause(0.01);
    drawnow;
end
