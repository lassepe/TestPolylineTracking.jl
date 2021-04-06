function tracking_error(px, py; lane, direction, target_progress)
    p = Point(px, py)
    wp = next_waypoint(lane, p, direction, target_progress)
    mindistance(Euclidean(), wp, p)
end

function next_waypoint(lane, current_position, direction, target_progress)
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
        if total_progress >= target_progress
            break
        end

        step_vector = v - waypoint
        max_step_size = norm(step_vector)

        if total_progress + max_step_size <= target_progress
            waypoint = v
            total_progress = total_progress + max_step_size
        else
            waypoint = Point(
                coordinates(waypoint) + (target_progress - total_progress) * normalize(step_vector),
            )
            total_progress = target_progress
        end
    end

    waypoint
end
