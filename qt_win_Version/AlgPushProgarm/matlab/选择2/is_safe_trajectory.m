function safe = is_safe_trajectory(traj, obstacles, other_boats, r_safe)
    safe = true;
    for i = 1:size(traj,1)
        pos = traj(i,1:2);

        % 与障碍物碰撞检测
        for j = 1:length(obstacles)
            obs = obstacles(j);
            if obs.type == "circle"
                if norm(pos - [obs.x, obs.y]) < (obs.r + r_safe)
                    safe = false; return;
                end
            elseif obs.type == "rect"
                if abs(pos(1) - obs.x) < (obs.w/2 + r_safe) && ...
                   abs(pos(2) - obs.y) < (obs.h/2 + r_safe)
                    safe = false; return;
                end
            end
        end

        % 与其他船之间
        for j = 1:size(other_boats,1)
            if norm(pos - other_boats(j,:)) < r_safe
                safe = false; return;
            end
        end
    end
end
