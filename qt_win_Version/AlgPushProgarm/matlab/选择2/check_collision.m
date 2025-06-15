function has_collision = check_collision(state, v, w, obstacles, other_boats, predict_time, dt, r_safe)
    traj = predict_trajectory(state, v, w, floor(predict_time / dt), dt);
    has_collision = ~is_safe_trajectory(traj, obstacles, other_boats, r_safe);
end
