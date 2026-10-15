function registerRoutePool!(solver::Solver, solution::Solution)
    algo = solver.algorithm
    for r = 1:length(solution.routes)
        rt = solution.routes[r]
        if length(rt.visits) <= 2
            continue
        end
        if (rt.feasibleF == length(rt.visits) - 1) && (rt.forwardLabels[end].std1State.stdWarp <= 1e-6) && (rt.forwardLabels[end].std2State.stdWarp <= 1e-6)
            routeId = get(algo.routeLookup, rt.visits, 0)

            if routeId > 0
                if solution.cost < algo.costStorage[routeId]
                    algo.costStorage[routeId] = solution.cost
                end
            else
                persistentRouteCopy = copy(rt.visits)
                push!(algo.routeStorage, persistentRouteCopy)
                push!(algo.costStorage, solution.cost)
                newId = length(algo.routeStorage)
                algo.routeLookup[persistentRouteCopy] = newId
            end
        end
    end
end

function visitsCustomer(i::Int, route::Vector{Int})
    return i in route
end

function routeCost(solver::Solver, r::Vector{Int})
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
    # The cutoff is only a speed-up hint (CPLEX-specific); other solvers (e.g. HiGHS) skip it.
    try
        set_optimizer_attribute(sp, "CPXPARAM_MIP_Tolerances_UpperCutoff", cutOff + 0.1)
    catch e
        e isa JuMP.MOI.UnsupportedAttribute || rethrow()
    end

    set_time_limit_sec(sp, solver.algorithm.timeLimitSP)
    routes = solver.algorithm.routeStorage

    @variable(sp, λ[r = 1:length(routes)], Bin)

    @objective(sp, Min, sum(routeCost(solver, routes[r])*λ[r] for r = 1:length(routes)))

    @constraint(sp, [i = 1:length(solver.data.vertices)], sum(visitsCustomer(i, routes[r]) * λ[r] for r = 1:length(routes)) == 1)
    @constraint(sp, sum(λ[r] for r = 1:length(routes)) <= solver.data.maxNbRoutes)

    nonEmptyRoutes = filter(rt -> length(rt.visits) > 2, solver.bestFeasSol.routes)
    warmStartIds = [get(solver.algorithm.routeLookup, rt.visits, 0) for rt in nonEmptyRoutes]
    if !isempty(warmStartIds) && all(id -> id > 0, warmStartIds)
        for id in warmStartIds
            set_start_value(λ[id], 1.0)
        end
    end

    optimize!(sp)

    if termination_status(sp) == OPTIMAL
        if objective_value(sp) < solver.bestFeasSol.cost - 1e-6
            solver.bestFeasSol.routes = [newRoute(solver, routes[r]) for r = 1:length(routes) if value(λ[r]) >= 0.9]
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
                solver.bestFeasSol.routes = [newRoute(solver, routes[r]) for r = 1:length(routes) if value(λ[r]) >= 0.9]
                solver.bestFeasSol.cost = objective_value(sp)
                solver.bestFeasSol.dist = 0
                computeLabels(solver, solver.bestFeasSol)
                registerBestFeasible!(solver, solver.bestFeasSol)
            end
        end
        return solver.bestFeasSol
    end
end
