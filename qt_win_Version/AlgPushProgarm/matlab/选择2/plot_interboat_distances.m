function plot_interboat_distances(history, dt, r_safe)
[N, steps, ~] = size(history);
t = 0:dt:(steps-1)*dt;

figure; hold on;
for i = 1:N
    for j = i+1:N
        d = vecnorm(squeeze(history(i,:,1:2) - history(j,:,1:2)), 2, 2);
        plot(t, d, 'DisplayName', sprintf('%d-%d', i,j));
    end
end
yline(r_safe, 'r--', '安全距离');
legend;
xlabel('时间 (s)');
ylabel('船间距离');
title('无人艇间最小距离变化');
end
