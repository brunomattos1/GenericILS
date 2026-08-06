function registerRoutePool!(solver::Solver, solution::Solution)
    algo = solver.algorithm
    for r = 1:length(solution.routes)
        rt = solution.routes[r]
        if length(rt.visits) <= 2
            continue
        end
        if (rt.feasibleF == length(rt.visits) - 1) && (rt.forwardLabels[end].std1State.stdWarp <= 1e-6) && (rt.forwardLabels[end].std2State.stdWarp <= 1e-6)
            route_id = get(algo.route_lookup, rt.visits, 0)

            if route_id > 0
                if solution.cost < algo.cost_storage[route_id]
                    algo.cost_storage[route_id] = solution.cost
                end
            else
                persistent_route_copy = copy(rt.visits)
                push!(algo.route_storage, persistent_route_copy)
                push!(algo.cost_storage, solution.cost)
                new_id = length(algo.route_storage)
                algo.route_lookup[persistent_route_copy] = new_id
            end
        end
    end
end

function α(i::Int, route::Vector{Int})
    return i in route
end

function c(solver::Solver, r::Int)
    cost = 0.
    for i = 1:length(solver.pool[r])-1
        cost += solver.data.costMatrix[solver.pool[r][i]+1, solver.pool[r][i+1]+1]
    end
    return cost
end

function c(solver::Solver, r::Vector{Int})
    cost = 0.
    for i = 1:length(r)-1
        cost += solver.data.costMatrix[r[i]+1, r[i+1]+1]
    end
    if isCostResource()
        cost += computeRouteLabelCost(solver, r)
    end
    return cost
end

function setPartitioning(solver::Solver, cutOff::Float64)
    sp = Model(solver.MIPSolver)
    set_silent(sp)
    set_optimizer_attribute(sp, "CPXPARAM_MIP_Tolerances_UpperCutoff", cutOff + 0.1)

    set_time_limit_sec(sp, solver.algorithm.timeLimitSP)
    routes = solver.algorithm.route_storage

    @variable(sp, λ[r = 1:length(routes)], Bin)

    @objective(sp, Min, sum(c(solver, routes[r])*λ[r] for r = 1:length(routes)))

    @constraint(sp, [i = 1:length(solver.data.vertices)], sum(α(i, routes[r])λ[r] for r = 1:length(routes)) == 1)
    @constraint(sp, sum(λ[r] for r = 1:length(routes)) <= solver.data.maxNbRoutes)

    nonEmptyRoutes = filter(rt -> length(rt.visits) > 2, solver.bestFeasSol.routes)
    warmStartIds = [get(solver.algorithm.route_lookup, rt.visits, 0) for rt in nonEmptyRoutes]
    if !isempty(warmStartIds) && all(id -> id > 0, warmStartIds)
        for id in warmStartIds
            set_start_value(λ[id], 1.0)
        end
    end

    optimize!(sp)

    if termination_status(sp) == OPTIMAL
        if objective_value(sp) < solver.bestFeasSol.cost - 1e-6
            solver.bestFeasSol.routes = [new_route(solver, routes[r]) for r = 1:length(routes) if value(λ[r]) >= 0.9]
            solver.bestFeasSol.cost = objective_value(sp)
            solver.bestFeasSol.dist = 0

            computeLabels(solver, solver.bestFeasSol)
            registerBestFeasible!(solver, solver.bestFeasSol)
        end
    end
    if termination_status(sp) == INFEASIBLE
        return solver.bestFeasSol
    end
    if termination_status(sp) == TIME_LIMIT
        if result_count(sp) >= 1
            if objective_value(sp) < solver.bestFeasSol.cost - 1e-6
                solver.bestFeasSol.routes = [new_route(solver, routes[r]) for r = 1:length(routes) if value(λ[r]) >= 0.9]
                solver.bestFeasSol.cost = objective_value(sp)
                solver.bestFeasSol.dist = 0
                computeLabels(solver, solver.bestFeasSol)
                registerBestFeasible!(solver, solver.bestFeasSol)
            end
        end
        return solver.bestFeasSol
    end
end
