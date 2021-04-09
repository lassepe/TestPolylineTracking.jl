problem = let
    x0 = [0, 0, 0.01, 0]

    lane = Chain(Point(0.0, 0.0), Point(1.0, 0.0), Point(2.0, 1.0), Point(3.0, 1.0))

    dynamics = let
        # dynamics
        Δt = 0.1

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

    direction = SA[1, 0]

    target_progress = 0.1

    (; x0, lane, dynamics, direction, target_progress)
end
