% main.m
clc;close all;clear all;
%% 主函数：无人艇编队 + 避障 + 优先级避碰

% 参数设置
N = 6;              % 无人艇数量
T = 400;            % 仿真总时间
dt = 0.1;           % 仿真时间步长
steps = T / dt;     % 仿真步数

v_min = 3;
v_max = 15;
dv_max = 0.3;       % 最大速度变化率
w_max = pi/3;       % 最大角速度
r_safe = 60;        % 船间最小安全距离60
predict_time = 30;  % DWA预测窗口时间30
predict_dt = 0.5;
boom = 0;
obsboom = 0;
% 编队结构（相对于领导者的偏移）

%test1
%[4210 0; 4080 -225; 4080 225; 4080 -75; 4080 75; 3950 0]
%[3950 0; 3820 -225; 3820 225; 3820 -75; 3820 75; 3690 0]
formation_offsets = [210 0; 80 -225; 80 225; 80 -75; 80 75; -50 0];
%est20
%[3950 0; 3950 150; 4100 150; 4100 0; 3800 -150; 3800 300; 4250 300; 4250 -150]
% formation_offsets = [-50 0; -50 150; 100 150; 100 0; -200 -150; -200 300; 250 300; 250 -150];

%test33
%formation_offsets = [189.073 212.707; 269.571   86.227; 286.164   -62.9114; 236.569   -204.407; 59.7281   288.861; 91.4009   99.6734;138.151   -42.9888;67.3478   -175.358;-90.2562   294.259; -50   150;-50   0;-73.728   -225.756;-223.281   225.01;-303.338   96.234;-301.196   -54.3105;-215.726   -177.719];
%test27
%formation_offsets = [-50   150;  -50   300; -50   -450; -50   -150; -50   0; -50   -300;-50   450;-50   600];

% 初始状态：[x y theta]
%test1
boats = [0,0,0;75,130,0;75,-130,0;225,130,0;225,-130,0;300,0,0];
%test20
% boats = [0,0,0;150,0,0;-150,0,0;0,150,0;-150,-150,0;150,-150,0;-300,-150,0;300,-150,0];

%test33
% boats = [0,0,0;0,150,0;0,300,0;0,450,0;150,0,0;150,150,0;150,300,0;150,450,0;300,0,0;300,150,0;300,300,0;300,450,0;450,0,0;450,150,0;450,300,0;450,450,0];
%test27
%boats = [0,0,0;0,150,0;0,-150,0;0,300,0;0,-300,0;0,450,0;0,-450,0;0,600,0];

velocities = 5 * ones(N, 1);  % 初始速度为 5 m/s

% 障碍物设置
obstacles = [ ...
    %test33
%     struct('type',"circle",'x',1300,'y',0,   'r',300,'w',NaN,'h',NaN), ...
%     struct('type',"circle",'x',1900,'y',0,   'r',300,'w',NaN,'h',NaN), ...
%     struct('type',"circle",'x',2600,'y',0,   'r',300,'w',NaN,'h',NaN), ...
    
    %struct('type',"circle",'x',2000,'y',0,   'r',200,'w',NaN,'h',NaN), ...
    % struct('type',"rect",  'x',250,'y',60,  'r',NaN,'w',50,'h',20), ...
    % struct('type',"rect",  'x',300,'y',-60, 'r',NaN,'w',30,'h',30), ...
    % struct('type',"circle",'x',400,'y',0,   'r',20,'w',NaN,'h',NaN) ...
];

% 轨迹记录
history = zeros(N, steps, 2);
leader_traj = zeros(steps, 2);

% 主仿真循环
for k = 1:steps
    t = (k-1)*dt
    leader_pos = [8*t + 800, 0];  % 领导者以8 m/s前进
    leader_traj(k,:) = leader_pos;

    % 优先级更新：按与领导者的距离排序，近的优先
%     dists_to_leader = vecnorm((boats(:,1:2) - leader_pos), 2, 2);
%     [~, priority] = sort(dists_to_leader);

for K = 1:N
    if boats(1,1) > leader_pos(1,1)
        dists_to_leader = -vecnorm((boats(:,1:2) - leader_pos), 2, 2);
    else
        dists_to_leader = vecnorm((boats(:,1:2) - leader_pos), 2, 2);
    end
end
[~, priority] = sort(dists_to_leader);

%     dists_to_leader = boats(:,1);
%     [~, priority] = sort(dists_to_leader, 'descend');  % 'descend' 表示从大到小排序

    for i = 1:N
        goal = leader_pos + formation_offsets(i,:);
        state = boats(i,:);
        v_cur = velocities(i);
        w_cur = 0;

        % 构造动态障碍（其他船）
        other_boats = boats(:,1:2);
        other_boats(i,:) = [];

        % 控制器调用
        [v_des, w] = dwa_controller_priority( ...
            state, goal, v_cur, w_cur, obstacles, ...
            v_min, v_max, dv_max, predict_time, predict_dt, ...
            r_safe, other_boats, i, priority);

        % 平滑速度变化
        dv = max(-dv_max, min(dv_max, v_des - v_cur));
        v_new = v_cur + dv;
        % v_new = v_des;
        velocities(i) = v_new;

        % 更新状态
        theta = state(3);
        boats(i,1) = boats(i,1) + v_new * cos(theta) * dt;
        boats(i,2) = boats(i,2) + v_new * sin(theta) * dt;
        boats(i,3) = boats(i,3) + w * dt;

        history(i,k,:) = boats(i,1:2);
    end
    for i = 1:N
        for j = 1:N
            if i ~= j
                kj = sqrt((boats(i,1)-boats(j,1))^2+(boats(i,2)-boats(j,2))^2);
                if(kj<=50)
                boom = 1;
                break
                end
            end
        end
    end
    for i = 1:N
        for j = 1:length(obstacles)
            obs = obstacles(j);
            kv = sqrt((boats(i,1)-obs.x)^2+(boats(i,2)-obs.y)^2);
            if(kv<=50+obs.r)
                obsboom = 1;
                break
            end
        end
    end
end
%%
% 结果绘图
figure;
hold on;
colors = lines(N);
for i = 1:N
    plot(squeeze(history(i,:,1)), squeeze(history(i,:,2)), '-', 'Color', colors(i,:), 'DisplayName', sprintf('Boat %d', i));
end
plot(leader_traj(:,1), leader_traj(:,2), 'k--', 'DisplayName', 'Leader');
legend;
title('无人艇编队轨迹');
xlabel('x'); ylabel('y'); axis equal;

% 障碍物绘图
for i = 1:length(obstacles)
    obs = obstacles(i);
    if obs.type == "circle"
        viscircles([obs.x, obs.y], obs.r, 'EdgeColor','r');
    elseif obs.type == "rect"
        rectangle('Position', [obs.x - obs.w/2, obs.y - obs.h/2, obs.w, obs.h], ...
                  'EdgeColor','r', 'LineWidth',1.5);
    end
end
