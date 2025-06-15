function [v, w] = pid_controller(state, goal, v_cur, v_min, v_max)
    % 简化 PID：只针对方向角控制
    Kp_ang = 2.0;
    Kp_v = 0.5;

    pos = state(1:2);
    theta = state(3);

    % 计算目标方向和角误差
    direction = goal - pos;
    desired_theta = atan2(direction(2), direction(1));
    error_theta = atan2(sin(desired_theta - theta), cos(desired_theta - theta));

    % 线速度用目标距离决定（越远越快）
    dist_to_goal = norm(direction);
    v = Kp_v * dist_to_goal;
    v = max(min(v, v_max), v_min);  % 约束速度

    % 角速度 PID
    w = Kp_ang * error_theta;
end
