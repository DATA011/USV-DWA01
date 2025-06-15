function visualize_simulation(history, leader_traj, obstacles, dt, r_safe)
[N, steps, ~] = size(history);
figure; hold on; axis equal;
colors = lines(N);

for i = 1:length(obstacles)
    obs = obstacles(i);
    if obs.type == "circle"
        viscircles([obs.x, obs.y], obs.r, 'Color','r');
    else
        rectangle('Position', [obs.x-obs.w/2, obs.y-obs.h/2, obs.w, obs.h], ...
                  'EdgeColor', 'r');
    end
end

for t = 1:10:steps
    cla;
    for i = 1:N
        plot(squeeze(history(i,1:t,1)), squeeze(history(i,1:t,2)), '-', ...
            'Color', colors(i,:), 'LineWidth', 1.2);
        plot(history(i,t,1), history(i,t,2), 'o', 'Color', colors(i,:), ...
            'MarkerFaceColor', colors(i,:));
    end
    plot(leader_traj(1:t,1), leader_traj(1:t,2), 'k--', 'LineWidth', 1.5);
    title(sprintf('Time: %.1f sec', (t-1)*dt));
    xlim([0, max(leader_traj(:,1))+50]); ylim([-300, 300]);
    drawnow;
end
end
