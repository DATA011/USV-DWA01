function [v_opt, w_opt] = dwa_controller(state, goal, v_cur, w_cur, obstacles, ...
        v_min, v_max, dv_max, predict_time, dt, r_safe, other_boats, boat_id, N)

    x = state(1); y = state(2); theta = state(3);
    v_low = max(v_min, v_cur - dv_max);
    v_high = min(v_max, v_cur + dv_max);
    w_max = pi/3;

    v_samples = linspace(v_low, v_high, 5);
    w_samples = linspace(-w_max, w_max, 9);

    best_score = -inf;
    v_opt = v_cur;
    w_opt = 0;

    for vi = 1:length(v_samples)
        for wi = 1:length(w_samples)
            v = v_samples(vi); w = w_samples(wi);
            traj = simulate_trajectory(x, y, theta, v, w, predict_time, dt);

            % 如果与静态障碍或其他船冲突
            if check_collision(traj, obstacles, r_safe)
                continue;
            end

            % 船间避障：让行策略（后编号优先避）
            danger = false;
            for j = 1:size(other_boats,1)
                dist_min = min(vecnorm(traj(:,1:2) - other_boats(j,:), 2, 2));
                if dist_min < r_safe
                    if boat_id > j  % 当前艇编号大，让行
                        v = v * 0.6; % 减速避让
                        w = w + 0.1 * sign(randn()); % 稍微偏转
                        danger = true;
                    else
                        continue;  % 高优先级艇继续尝试
                    end
                end
            end

            final_dist = norm(traj(end,1:2) - goal);
            score = -final_dist;
            if danger
                score = score - 100; % 略微惩罚，让行优先
            end

            if score > best_score
                best_score = score;
                v_opt = v;
                w_opt = w;
            end
        end
    end
end
