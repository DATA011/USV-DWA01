function [v_opt, w_opt] = dwa_controller_priority( ...
    state, goal, v_cur, w_cur, obstacles, ...
    v_min, v_max, dv_max, predict_time, predict_dt, dt, ...
    r_safe, other_boats, boats, index, priority)

theta = state(3);
best_score = -inf;
v_opt = v_cur;
w_opt = 0;

w_samples = linspace(-pi/3, pi/3, 7);
v_samples = linspace(max(v_min, v_cur - dv_max), ...
                     min(v_max, v_cur + dv_max), 5);

for v = v_samples
    for w = w_samples
        traj = predict_trajectory(state, v, w, predict_time, predict_dt);

        goal_cost = -norm(traj(end,1:2) - goal);

        col_safe = true;
        for j = 1:size(other_boats,1)
            dists = vecnorm(traj(:,1:2) - other_boats(j,:), 2, 2);
            if any(dists < r_safe)
                pr_i = find(priority == index);
                pr_j = find(priority == j);
                if pr_i > pr_j
                    goal_cost = goal_cost - 1e3;  % 低优先级就罚分
                end
                col_safe = false;
                break;
            end
        end


        if col_safe
            for obs = obstacles
                if obs.type == "circle"
                    dists = vecnorm(traj(:,1:2) - [obs.x, obs.y], 2, 2);
                    if any(dists < obs.r + 5)
                        col_safe = false; break;
                    end
                elseif obs.type == "rect"
                    if any( ...
                        traj(:,1) > obs.x - obs.w/2 & traj(:,1) < obs.x + obs.w/2 & ...
                        traj(:,2) > obs.y - obs.h/2 & traj(:,2) < obs.y + obs.h/2)
                        col_safe = false; break;
                    end
                end
            end
        end

        if col_safe && goal_cost > best_score
            best_score = goal_cost;
            v_opt = v;
            w_opt = w;
        end
    end
end
end
