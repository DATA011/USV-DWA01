function traj = simulate_trajectory(x, y, theta, v, w, predict_time, dt)
    steps = floor(predict_time / dt);
    traj = zeros(steps, 3);
    for i = 1:steps
        x = x + v * cos(theta) * dt;
        y = y + v * sin(theta) * dt;
        theta = theta + w * dt;
        traj(i,:) = [x, y, theta];
    end
end
