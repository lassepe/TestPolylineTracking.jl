function add_objective! end

struct NonlinearJuMPObjective{F}
    cost::F
end

function add_objective!(opt_model, objective::NonlinearJuMPObjective, x, u; kwargs...)
    n_states, n_timesteps = size(x)
    n_controls = size(u, 1)

    JuMP.register(
        opt_model,
        :cost,
        (n_states + n_controls) * n_timesteps,
        (v...) -> objective.cost(
            reshape(collect(v[begin:(n_states * n_timesteps)]), n_states, n_timesteps),
            reshape(collect(v[(n_states * n_timesteps + 1):end]), n_controls, n_timesteps);
            kwargs...,
        );
        autodiff = true,
    )
    JuMP.@NLobjective(opt_model, JuMP.MOI.MIN_SENSE, cost(x..., u...))
end

struct QuadraticJuMPObjective{F}
    cost::F
end

function add_objective!(opt_model, objective::QuadraticJuMPObjective, x, u; kwargs...)
    JuMP.@objective(opt_model, JuMP.MOI.MIN_SENSE, objective.cost(x, u; kwargs...))
end


