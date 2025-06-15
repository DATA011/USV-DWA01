% ========= 绘制船间距离随时间变化图 ==========
figure;
hold on;
colors = lines(N);

% 计算每一对船之间的距离
pair_idx = 1;
for i = 1:N
    for j = i+1:N
        dist = zeros(steps,1);
        for k = 1:steps
            pos_i = squeeze(history(i,k,:));
            pos_j = squeeze(history(j,k,:));
            dist(k) = norm(pos_i - pos_j);
        end
        plot(1:steps, dist, 'DisplayName', sprintf('Boat %d-%d', i, j));
        pair_idx = pair_idx + 1;
    end
end

% 添加安全距离线
yline(r_safe, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Safe distance');

xlabel('Time step');
ylabel('Distance Between Boats (m)');
title('Inter-Boat Distances Over Time');
legend show;
grid on;
