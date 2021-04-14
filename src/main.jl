import JuMP
import Ipopt

using Meshes: Meshes, Chain, Point, coordinates, vertices, mindistance, closest_point
using Distances: Euclidean, SqEuclidean
using VegaLite: @vlplot
using LinearAlgebra: norm, normalize, ⋅
using StaticArrays: SA
using JuMPObjectiveUtils: NonlinearJuMPObjective, QuadraticJuMPObjective, add_objective!

include("problem.jl")
include("tracking_utils.jl")

function dynamic_tracking_cost(x, u; problem)
    sum(2:(problem.n_timesteps)) do t
        p = Point(x[1, t], x[2, t])
        p_previous = Point(x[1, t - 1], x[2, t - 1])
        wp = next_waypoint(problem.lane, p_previous, problem.direction, problem.step_distance)
        mindistance(SqEuclidean(), wp, p)
    end + sum(u .^ 2)
end

function static_tracking_cost(x, u; problem)
    p0 = Point(problem.x0[problem.position_indices]...)
    nominal_waypoints = mapreduce(hcat, 0:(problem.n_timesteps - 1)) do t
        coordinates(next_waypoint(problem.lane, p0, problem.direction, t * problem.step_distance))
    end
    sum((x[problem.position_indices, :] .- nominal_waypoints) .^ 2) + 0.1 * sum(u .^ 2)
end

function track_jump(objective, problem)
    n_states, n_controls = size(problem.dynamics.B)

    opt_model = JuMP.Model(Ipopt.Optimizer)
    x = JuMP.@variable(opt_model, [1:n_states, 1:(problem.n_timesteps)])
    u = JuMP.@variable(opt_model, [1:n_controls, 1:(problem.n_timesteps)])

    # initial condition
    JuMP.@constraint(opt_model, x[:, 1] .== problem.x0)

    for obstacle in problem.obstacles
        JuMP.@constraint(
            opt_model,
            [t = 1:(problem.n_timesteps)],
            sum((x[problem.position_indices, t] .- obstacle.position) .^ 2) >= obstacle.radius^2
        )
    end

    # dynamics
    JuMP.@constraint(
        opt_model,
        [t = 1:(problem.n_timesteps - 1)],
        x[:, t + 1] .== problem.dynamics.A * x[:, t] + problem.dynamics.B * u[:, t]
    )

    add_objective!(opt_model, objective, x, u; problem)

    JuMP.optimize!(opt_model)

    (; x = JuMP.value.(x), u = JuMP.value.(u))
end

solution_jump = track_jump(QuadraticJuMPObjective(static_tracking_cost), problem)

function named_coordinates(vector)
    (; x = vector[1], y = vector[2])
end

function trajectory_viz(x)
    data = [(; named_coordinates(xt)..., t) for (t, xt) in enumerate(eachcol(x))]
    visualizer = @vlplot("point", x = "x:q", y = "y:q", color = "t:q", order = "t:q")
    data |> visualizer
end

function obstacle_viz(obstacle)
    data = [(; named_coordinates(obstacle.position)..., obstacle.radius)]
    println(data)
    visualizer =
        @vlplot("point", "x:q", "y:q", size = {type = "quantitative", expr = "datum.radius * 1000"})
    data |> visualizer
end

function lane_viz(lane)
    data = [(; named_coordinates(coordinates(p))..., i) for (i, p) in enumerate(vertices(lane))]
    visualizer = @vlplot(:line, "x:q", "y:q")
    data |> visualizer
end

@vlplot(height = 300, width = 900) +
lane_viz(problem.lane) +
trajectory_viz(solution_jump.x) +
sum(obstacle_viz, problem.obstacles; init = @vlplot())
