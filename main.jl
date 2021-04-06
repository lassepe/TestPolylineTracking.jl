import JuMP
import Ipopt

using Meshes: Meshes, Chain, Point, coordinates, vertices, mindistance, closest_point
using Distances: Euclidean, SqEuclidean
using VegaLite: @vlplot
using LinearAlgebra: norm, normalize, ⋅
using StaticArrays: SA

lane = Chain(Point(0.0, 0.0), Point(1.0, 0.0), Point(2.0, 1.0), Point(3.0, 1.0))

function cross_track_error(px, py)
    mindistance(Euclidean(), Point(px, py), lane)
end

function tracking_error(px, py)
    p = Point(px, py)
    wp = next_waypoint(lane, p, SA[1., 0.], 0.1)
    mindistance(Euclidean(), wp, p)
end

function next_waypoint(lane, current_position, direction, preferred_progress)
    metric = Euclidean()
    segments = collect(Meshes.segments(lane))
    closest = reduce(
        enumerate(segments);
        init = (; xte = Inf, lane_point = first(vertices(lane)), segment = first(segments), i = 0),
    ) do best, (i, segment)
        lane_point = closest_point(metric, segment, current_position)
        xte = mindistance(metric, lane_point, current_position)
        xte < best.xte ? (; xte, lane_point, segment, i) : best
    end

    total_progress = 0.0
    waypoint = closest.lane_point

    ordered_tail_vertices = let
        p1, p2 = vertices(closest.segment)
        if direction ⋅ (p2 - p1) > 0
            vertices(lane)[(closest.i + 1):end]
        else
            vertices(lane)[(closest.i):-1:begin]
        end
    end

    for v in ordered_tail_vertices
        if total_progress >= preferred_progress
            break
        end

        step_vector = v - waypoint
        max_step_size = norm(step_vector)

        if total_progress + max_step_size <= preferred_progress
            waypoint = v
            total_progress = total_progress + max_step_size
        else
            waypoint = Point(
                coordinates(waypoint) +
                (preferred_progress - total_progress) * normalize(step_vector),
            )
            total_progress = preferred_progress
        end
    end

    waypoint
end

x0 = [0, 0, 0.2, 0]
xg = [3, 1, 0, 0]

opt_model = JuMP.Model(Ipopt.Optimizer)
x = JuMP.@variable(opt_model, [1:4, 1:100])
u = JuMP.@variable(opt_model, [1:2, 1:100])

# initial condition
JuMP.@constraint(opt_model, x[:, 1] .== x0)

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

JuMP.@constraint(opt_model, [t = 1:99], x[:, t + 1] .== A * x[:, t] + B * u[:, t])

JuMP.register(opt_model, :tracking_error, 2, tracking_error, autodiff = true)

# tracking objective
JuMP.@NLobjective(
    opt_model,
    JuMP.MOI.MIN_SENSE,
    sum(tracking_error(x[1, t], x[2, t])^2 for i in 1:4, t in 1:100) +
    sum(u[i, t]^2 for i in 1:2, t in 1:100)
)

JuMP.optimize!(opt_model)

trajectory_viz = let
    trajectory_data =
        [(; px = xt[1], py = xt[2], t) for (t, xt) in enumerate(eachcol(JuMP.value.(x)))]
    trajectory_visualizer =
        @vlplot("point", x = "px:q", y = "py:q", color = "t:q", order = "t:q")
    trajectory_data |> trajectory_visualizer
end

lane_viz = let
    [(; x = coordinates(p)[1], y = coordinates(p)[2], i) for (i, p) in enumerate(vertices(lane))] |> @vlplot(:line, "x:q", "y:q")
end

@vlplot(height = 300, width = 900) + lane_viz + trajectory_viz
