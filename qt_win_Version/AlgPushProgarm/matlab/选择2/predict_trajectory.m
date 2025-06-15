function traj = predict_trajectory(state, v, w, steps, dt)
    x = state(1); y = state(2); theta = state(3);
    traj = zeros(steps, 3);
    for k = 1:steps
        x = x + v * cos(theta) * dt;
        y = y + v * sin(theta) * dt;
        %w
        theta = theta + w * dt;
        traj(k,:) = [x, y, theta];
    end
end
