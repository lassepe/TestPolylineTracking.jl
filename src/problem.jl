problem = let
    lane = Chain(Point(0.0, 0.0), Point(1.0, 0.0), Point(2.0, 1.0), Point(3.0, 1.0))

    # dynamics
    n_timesteps = 100
    T = 10
    Δt = T / n_timesteps

    dynamics = let

        A = [
            1 0 Δt 0
            0 1 0 Δt
            0 0 1 0
            0 0 0 1
        ]

        B = [
            0 0
            0 0
            Δt 0
            0 Δt
        ]

        (; A, B)
    end

    position_indices = SA[1, 2]
    total_path_length = sum(Meshes.measure, Meshes.segments(lane))
    step_distance = total_path_length / (n_timesteps - 1)
    v_nominal = step_distance / Δt
    x0 = [0, 0.2, v_nominal, 0]

    direction = SA[1, 0]

    obstacles =
        [(; position = SA[0.5, 0.0], radius = 0.2), (; position = SA[2.0, 1.0], radius = 0.2)]

    (; x0, lane, dynamics, direction, step_distance, n_timesteps, position_indices, obstacles)
end
