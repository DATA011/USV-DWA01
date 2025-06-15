% main.m
clc;close all;clear all;

%% 参数设置
N = 6;
T = 400;
dt = 0.1;
steps = T / dt;
r_safe = 75;
v_min = 3;
v_max = 15;
dv_max = 0.5;
predict_time = 25.0;
predict_dt = 0.5;

formation_offsets = [-50 0; -180 -225; -180 225; -180 -75; -180 75; -310 0];
boats = [0,0,0;75,130,0;75,-130,0;225,130,0;225,-130,0;300,0,0];
velocities = 5 * ones(N,1);
w_max = pi/3;

obstacles = [ ...
    % struct('type',"circle",'x',150,'y',20, 'r',8), ...
    % struct('type',"rect",  'x',180,'y',0,  'w',20,'h',10), ...
    % struct('type',"circle",'x',260,'y',0,  'r',6), ...
    % struct('type',"rect",  'x',300,'y',-10,'w',15,'h',20) ...
];

history = zeros(N, steps, 2);
leader_traj = zeros(steps, 2);

%% 仿真主循环
for k = 1:steps
    t = (k-1)*dt
    leader_pos = [8*t + 300, 0]; % 领导者沿x轴匀速前进（速度改为8）
    leader_traj(k,:) = leader_pos;

    % 动态优先级计算：距离越近优先级越高（值越小）
    distances_to_leader = vecnorm(boats(:,1:2) - leader_pos, 2, 2);
    [~, priority] = sort(distances_to_leader);

    for pr = 1:N
        i = priority(pr);
        goal = leader_pos + formation_offsets(i,:);
        state = boats(i,:);
        v_cur = velocities(i);
        w_cur = 0;

        other_boats = boats(:,1:2);
        other_boats(i,:) = [];

        % DWA 控制器（无惩罚项）
        [v_des, w] = dwa_controller_priority( ...
            state, goal, v_cur, w_cur, obstacles, ...
            v_min, v_max, dv_max, predict_time, predict_dt, dt, ...
            r_safe, other_boats, boats, i, priority);

        dv = max(-dv_max, min(dv_max, v_des - v_cur));
        v_new = v_cur + dv;
        velocities(i) = v_new;

        theta = state(3);
        boats(i,1) = boats(i,1) + v_new * cos(theta) * dt;
        boats(i,2) = boats(i,2) + v_new * sin(theta) * dt;
        boats(i,3) = boats(i,3) + w * dt;

        history(i,k,:) = boats(i,1:2);
    end
end
r_safe = 50;
% %% 可视化动画
% visualize_simulation(history, leader_traj, obstacles, dt, r_safe);
% 
% %% 船间距离图
% plot_interboat_distances(history, dt, r_safe);
