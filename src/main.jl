import JuMP
import Ipopt

using Meshes: Meshes, Chain, Point, coordinates, vertices, mindistance, closest_point
using Distances: Euclidean, SqEuclidean
using VegaLite: @vlplot
using LinearAlgebra: norm, normalize, ⋅
using StaticArrays: SA

include("problem.jl")
include("tracking_utils.jl")

function track_jump(problem)

    opt_model = JuMP.Model(Ipopt.Optimizer)
    x = JuMP.@variable(opt_model, [1:4, 1:100])
    u = JuMP.@variable(opt_model, [1:2, 1:100])

    # initial condition
    JuMP.@constraint(opt_model, x[:, 1] .== problem.x0)

    # dynamics
    JuMP.@constraint(
        opt_model,
        [t = 1:99],
        x[:, t + 1] .== problem.dynamics.A * x[:, t] + problem.dynamics.B * u[:, t]
    )

    # tracking objective
    JuMP.register(
        opt_model,
        :tracking_error,
        2,
        (px, py) -> tracking_error(px, py; problem.lane, problem.direction, problem.target_progress),
        autodiff = true,
    )
    JuMP.@NLobjective(
        opt_model,
        JuMP.MOI.MIN_SENSE,
        sum(tracking_error(x[1, t], x[2, t])^2 for i in 1:4, t in 1:100) +
        sum(u[i, t]^2 for i in 1:2, t in 1:100)
    )

    JuMP.optimize!(opt_model)

    (; x = JuMP.value.(x), u = JuMP.value.(u))
end

solution_jump = track_jump(problem)

function trajectory_viz(x)
    data = [(; px = xt[1], py = xt[2], t) for (t, xt) in enumerate(eachcol(x))]
    visualizer = @vlplot("point", x = "px:q", y = "py:q", color = "t:q", order = "t:q")
    data |> visualizer
end

function lane_viz(lane)
    data = [
        (; x = coordinates(p)[1], y = coordinates(p)[2], i) for (i, p) in enumerate(vertices(lane))
    ]
    visualizer = @vlplot(:line, "x:q", "y:q")
    data |> visualizer
end

@vlplot(height = 300, width = 900) + lane_viz(problem.lane) + trajectory_viz(solution_jump.x)