"""
    add_objective!(opt_model::JuMP.Model, objective, decision_variables...; parameters...)

Adds the cost function specfied by the `objective` to the `opt_model` for some generic arguments
`decision_variables...` and `parameters...` that are passed on to the objective.

Differentiation must happen only w.r.t. `decision_variables...`. `parameters...` can be optionally
used to provide additional parameters.
"""
function add_objective! end

"""
    NonlinearJuMPObjective

Wraps a `cost` that can be called as `cost(decision_variables...; parameters...)` to make it
accesible to a `JuMP.Model` via `add_objective!` using the `JuMP.@NLobjective` mechanism.

See also: `add_objective!` and `QuadraticJuMPObjective`.
"""
struct NonlinearJuMPObjective{F}
    cost::F
end

function add_objective!(
    opt_model,
    objective::NonlinearJuMPObjective,
    decision_variables...;
    parameters...,
)
    flattened_decision_variables = Iterators.flatten(decision_variables)
    flattened_length = sum(length, decision_variables)

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
        (v...) -> objective.cost(shape_like(v, decision_variables)...; parameters...);
        autodiff = true,
    )

    JuMP.@NLobjective(opt_model, JuMP.MOI.MIN_SENSE, cost(flattened_decision_variables...))
end

"""
    QuadraticJuMPObjective

Like `NonlinearJuMPObjective` but uses `JuMP.@objective`.
"""
struct QuadraticJuMPObjective{F}
    cost::F
end

function add_objective!(
    opt_model,
    objective::QuadraticJuMPObjective,
    decision_variables...;
    parameters...,
)
    JuMP.@objective(
        opt_model,
        JuMP.MOI.MIN_SENSE,
        objective.cost(decision_variables...; parameters...)
    )
end
