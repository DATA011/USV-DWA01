% === 配置参数 ===
clear;
csv_file = 'C:\Users\15205\Desktop\demo_qt_win_Version1121_last\qt_win_Version\AlgPushProgarm\build\Desktop_Qt_5_15_2_MinGW_64_bit-Debug\multi_usv_log.csv'; % CSV 文件路径
json_file = 'C:\Users\15205\Desktop\demo_qt_win_Version1121_last\qt_win_Version\AlgPushProgarm\build\Desktop_Qt_5_15_2_MinGW_64_bit-Debug\config\TaskInfo.json'; % JSON 队形配置路径

arrow_len = 10;         % 航向箭头长度
pause_time = 0.01;      % 每帧间隔

% === 读取 CSV 数据 ===
data = readtable(csv_file);
all_ids = unique(data.id);
N = length(all_ids);
steps = unique(data.Step);

% === 预处理轨迹 ===
history = cell(N, 1);
yaw_data = cell(N, 1);
for i = 1:N
    id = all_ids(i);
    rows = data.id == id;
    history{i} = [data.x(rows), data.y(rows)];
    yaw_data{i} = data.Yaw(rows);
end

% === 读取 JSON 队形配置 ===
json_text = fileread(json_file);
json_data = jsondecode(json_text);
if iscell(json_data.agents)
    agent2 = json_data.agents{2};  % 用花括号取元素
else
    agent2 = json_data.agents(2);  % 结构体数组正常访问
end

center = agent2.center;
radius = agent2.radius;
formation = agent2.formation;

% === 计算任务队形点位置 ===
formation_x = [];
formation_y = [];
for i = 1:length(formation)
    dist = formation(i).distance;
    ang = deg2rad(90 - formation(i).angle);
    fx = center(1) + dist * cos(ang);
    fy = center(2) + dist * sin(ang);
    formation_x(end+1) = fx;
    formation_y(end+1) = fy;
end

% === 初始化画布 ===
figure;
axis equal;
x_all_range = [data.x; formation_x(:); center(1)];
y_all_range = [data.y; formation_y(:); center(2)];
xlim([min(x_all_range)-50, max(x_all_range)+50]);
ylim([min(y_all_range)-50, max(y_all_range)+50]);
xlabel('X'); ylabel('Y'); grid on;
title('多USV轨迹 + 航向动画 + 任务队形');
hold on;

% 绘制目标圆轨迹（虚线圆）
theta = linspace(0, 2*pi, 100);
circle_x = center(1) + radius*cos(theta);
circle_y = center(2) + radius*sin(theta);
plot(circle_x, circle_y, 'r--');

% 绘制任务位置点和编号
for i = 1:length(formation)
    fx = formation_x(i);
    fy = formation_y(i);
    usv_id = formation(i).usvId;
    if usv_id == 0
        usv_id = i;
    end
    plot(fx, fy, 'rx', 'MarkerSize', 10, 'LineWidth', 2); % 红色 X
    text(fx + 10, fy, sprintf('USV %d', usv_id), 'Color', 'r', 'FontSize', 9);
end

% 绘制中心点
plot(center(1), center(2), 'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 5);
text(center(1) + 10, center(2), '中心', 'FontSize', 9);

% === 初始化轨迹对象 ===
colors = lines(N);
traj_lines = gobjects(N,1);
boat_markers = gobjects(N,1);
heading_arrows = gobjects(N,1);

for i = 1:N
    traj_lines(i) = plot(nan, nan, '-', 'Color', colors(i,:), 'LineWidth', 1.2);
    boat_markers(i) = plot(nan, nan, 'o', 'MarkerFaceColor', colors(i,:), 'MarkerEdgeColor', 'k');
    heading_arrows(i) = quiver(nan, nan, nan, nan, 0, 'Color', colors(i,:), 'LineWidth', 1.0, 'MaxHeadSize', 1);
    id_labels(i) = text(nan, nan, sprintf('%d', all_ids(i)), ...
                        'FontSize', 8, 'Color', colors(i,:), 'HorizontalAlignment', 'center', ...
                        'VerticalAlignment', 'bottom', 'FontWeight', 'bold');
end

% === 动画主循环 ===
for k = 1:length(steps)
    for i = 1:N-1
        x_all = history{i}(:,1);
        y_all = history{i}(:,2);
        yaw_all = yaw_data{i};

        if k <= length(x_all)
            % 轨迹更新
            set(traj_lines(i), 'XData', x_all(1:k), 'YData', y_all(1:k));
            set(boat_markers(i), 'XData', x_all(k), 'YData', y_all(k));

            % 航向箭头
            theta = deg2rad(yaw_all(k));
            dx = arrow_len * cos(theta);
            dy = arrow_len * sin(theta);
            set(heading_arrows(i), 'XData', x_all(k), 'YData', y_all(k), 'UData', dx, 'VData', dy);

            % USV ID 标签
            set(id_labels(i), 'Position', [x_all(k), y_all(k) + 5, 0]);  % 稍微往上偏移显示
        end
    end

    x_all = history{N}(:,1);
    y_all = history{N}(:,2);
    yaw_all = yaw_data{N};

    if k <= length(x_all)
        % 轨迹更新
        set(traj_lines(N), 'XData', x_all(1:k), 'YData', y_all(1:k),'LineStyle', '--');
        set(boat_markers(N), 'XData', x_all(k), 'YData', y_all(k));

        % 航向箭头
        theta = deg2rad(yaw_all(k));
        dx = arrow_len * cos(theta);
        dy = arrow_len * sin(theta);
        set(heading_arrows(N), 'XData', x_all(k), 'YData', y_all(k), 'UData', dx, 'VData', dy);

        % USV ID 标签
        set(id_labels(N), 'Position', [x_all(k), y_all(k) + 5, 0], 'String', 'vir');
    end

    drawnow;
    pause(pause_time);
end
