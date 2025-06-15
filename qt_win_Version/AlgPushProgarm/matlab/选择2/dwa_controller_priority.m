function [v_opt, w_opt] = dwa_controller_priority( ...
    state, goal, v_cur, w_cur, obstacles, ...
    v_min, v_max, dv_max, predict_time, dt, ...
    r_safe, other_boats, index, priority)

    % 参数初始化
    max_steps = floor(predict_time / dt);
    theta = state(3);
    best_score = -inf;
    v_opt = v_cur;
    w_opt = 0;

    % ==== 1. 检查是否存在碰撞风险 ====
    has_collision = check_collision(state, v_cur, w_cur, ...
                        obstacles, other_boats, predict_time, dt, r_safe);

    if ~has_collision
        % ==== 2. 使用 PID 控制（无碰撞风险时） ====
        [v_opt, w_opt] = pid_controller(state, goal, v_cur, v_min, v_max);
        return;
    end

    % ==== 3. 使用 DWA 控制器 ====
    for v = max(v_min, v_cur - dv_max):0.5:min(v_max, v_cur + dv_max)
        for w = -pi/3:pi/18:pi/3
            traj = predict_trajectory(state, v, w, max_steps, dt);
            final_pos = traj(end,1:2);

            % 到目标距离（越小越好）
            dist_to_goal = norm(final_pos - goal);
            heading_score = -dist_to_goal;
            % 检查碰撞
            if ~is_safe_trajectory(traj, obstacles, other_boats, r_safe)
                continue;
            end

            % 船间优先级让行
            score_penalty = 0;
            for j = 1:size(other_boats,1)
                if find(priority==index) > find(priority==j)
                    % 低优先级需让行，距离目标更远则降分
                    if norm(traj(end,1:2) - other_boats(j,:)) < r_safe * 1.5  %1.5
                        score_penalty = score_penalty - 100;
                    end
                end
            end

            total_score = heading_score + score_penalty;

            if total_score > best_score
                best_score = total_score;
                v_opt = v;
                w_opt = w;
            end
        end
    end
end
