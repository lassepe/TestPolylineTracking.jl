function tracking_error(px, py, px_last, py_last; lane, direction, step_distance)
    p = Point(px, py)
    p_last = Point(px_last, py_last)
    wp = next_waypoint(lane, p_last, direction, step_distance)
    mindistance(Euclidean(), wp, p)
end

function next_waypoint(lane, current_position, direction, step_distance)
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

    ordered_vertices = let
        p1, p2 = vertices(closest.segment)
        tail = if direction ⋅ (p2 - p1) > 0
            vertices(lane)[(closest.i + 1):end]
        else
            vertices(lane)[(closest.i):-1:begin]
        end

        [closest.lane_point; tail]
    end

    move_along_chain(ordered_vertices, step_distance)
end

function move_along_chain(points_on_chain, step_distance)
    moved_distance = 0.0
    next_point = first(points_on_chain)

    for v in points_on_chain[2:end]
        if moved_distance >= step_distance
            break
        end

        step_vector = v - next_point
        max_step_distance = norm(step_vector)

        if moved_distance + max_step_distance <= step_distance
            next_point = v
            moved_distance += max_step_distance
        else
            next_point = Point(
                coordinates(next_point) +
                (step_distance - moved_distance) * normalize(step_vector),
            )
            moved_distance = step_distance
        end
    end

    next_point
end
