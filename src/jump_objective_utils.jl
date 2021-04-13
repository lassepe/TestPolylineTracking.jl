function add_objective! end

struct NonlinearJuMPObjective{F}
    cost::F
end

function add_objective!(opt_model, objective::NonlinearJuMPObjective, args...; kwargs...)
    flattened_args = Iterators.flatten(args)
    flattened_length = sum(length, args)

    function shape_like(splatted_values, groups)
        n_collected = 0
        map(groups) do group
            value_group = reshape(
                collect(splatted_values[(n_collected + 1):(n_collected + length(group))]),
                size(group),
            )
            n_collected += length(group)
            value_group
        end
    end

    JuMP.register(
        opt_model,
        :cost,
        flattened_length,
        (v...) -> objective.cost(shape_like(v, args)...; kwargs...);
        autodiff = true,
    )

    JuMP.@NLobjective(opt_model, JuMP.MOI.MIN_SENSE, cost(flattened_args...))
end

struct QuadraticJuMPObjective{F}
    cost::F
end

function add_objective!(opt_model, objective::QuadraticJuMPObjective, args...; kwargs...)
    JuMP.@objective(opt_model, JuMP.MOI.MIN_SENSE, objective.cost(args...; kwargs...))
end
