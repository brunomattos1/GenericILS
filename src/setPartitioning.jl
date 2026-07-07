# function hash(route::Vector{Int})
#     edges = Vector{Tuple{Int, Int}}()
#     for i = 1:length(route) - 1
#         push!(edges, (route[i], route[i+1]))
#     end
#     return hash(edges)
# end

function hash(route::Vector{Int})
    h = UInt(0)
    @inbounds for i in 1:length(route)-1
        h = hash((route[i], route[i+1]), h)
    end
    return h
end

function push!(solver::Solver, solution::Solution)
    if isSymmetric()
        for r = 1:length(solution.routes)
            rt = solution.routes[r]
            if length(rt.visits) <= 2
                continue
            end
            if (rt.feasibleF == length(rt.visits) - 1) || (rt.feasibleB == length(rt.visits) - 1)
                routeHash = hash(rt.visits)
                if !haskey(solver.pool, rt.visits)
                    push!(solver.hashes, routeHash)
                    solver.pool[copy(rt.visits)] = solution.cost
                else
                    solver.pool[copy(rt.visits)] = min(solver.pool[rt.visits], solution.cost)
                end
            end
        end
    else
        for r = 1:length(solution.routes)
            rt = solution.routes[r]
            if length(rt.visits) <= 2
                continue
            end
            if (rt.feasibleF == length(rt.visits) - 1) && (rt.forwardLabels[end].std1State.stdWarp <= 1e-6) && (rt.forwardLabels[end].std2State.stdWarp <= 1e-6)
                route_id = get(solver.route_lookup, rt.visits, 0)

                if route_id > 0
                    if solution.cost < solver.cost_storage[route_id]
                        solver.cost_storage[route_id] = solution.cost
                    end
                else
                    persistent_route_copy = copy(rt.visits)
                    push!(solver.route_storage, persistent_route_copy)
                    push!(solver.cost_storage, solution.cost)
                    new_id = length(solver.route_storage)
                    solver.route_lookup[persistent_route_copy] = new_id
                end
            end
            # if (solution.feasiblesB[r] == length(solution.routes[r]) - 1) && (solution.backwardLabels[r][end].std_res.stdWarp <= 1e-6)
            #     route = solution.routes[r]
            #     route_id = get(solver.route_lookup, route, 0)

            #     if route_id > 0
            #         if solution.cost < solver.cost_storage[route_id]
            #             solver.cost_storage[route_id] = solution.cost
            #         end
            #     else
            #         persistent_route_copy = copy(route)
            #         push!(solver.route_storage, persistent_route_copy)
            #         push!(solver.cost_storage, solution.cost)
            #         new_id = length(solver.route_storage)
            #         solver.route_lookup[persistent_route_copy] = new_id
            #     end
            # end
            # if (solution.feasiblesB[r] == length(solution.routes[r]) - 1) && (solution.backwardLabels[r][end].std_res.stdWarp <= 1e-6)# || (solution.feasiblesB[r] == length(solution.routes[r]) - 2)
            #     revRoute = reverse(copy(solution.routes[r]))
            #     routeHash = hash(revRoute)
            #     if !haskey(solver.pool, revRoute)#(!(routeHash in solver.hashes))
            #         push!(solver.hashes, routeHash)
            #         solver.pool[revRoute] = solution.cost
            #     else
            #         solver.pool[revRoute] = min(solver.pool[revRoute], solution.cost)
            #     end
            # end
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
    set_optimizer_attribute(sp, "CPXPARAM_MIP_Tolerances_UpperCutoff", cutOff + 0.1)
    set_time_limit_sec(sp, solver.timeLimitSP)
    routes = solver.route_storage#collect(keys(solver.pool))
    
    @variable(sp, λ[r = 1:length(routes)], Bin)

    @objective(sp, Min, sum(c(solver, routes[r])*λ[r] for r = 1:length(routes)))

    @constraint(sp, [i = 1:length(solver.data.vertices)], sum(α(i, routes[r])λ[r] for r = 1:length(routes)) == 1)
    @constraint(sp, sum(λ[r] for r = 1:length(routes)) <= solver.data.maxNbRoutes)
    optimize!(sp)

    if termination_status(sp) == OPTIMAL
        println("-"^144)
        println("Set Partitioning optimally solved!")
        println("-"^144)

        solver.bestFeasSol.routes = [new_route(solver, routes[r]) for r = 1:length(routes) if value(λ[r]) >= 0.9]
        solver.bestFeasSol.cost = objective_value(sp)
        solver.bestFeasSol.dist = 0#objective_value(sp)

        computeLabels(solver, solver.bestFeasSol)
    end
    if termination_status(sp) == INFEASIBLE
        println("-"^144)
        println("Set Partitioning is infeasible!")
        println("-"^144)
        return solver.bestFeasSol
    end
    if termination_status(sp) == TIME_LIMIT
        println("-"^144)
        println("Set Partitioning reached time limit!")
        println("-"^144)
        if result_count(sp) >= 1
            if objective_value(sp) < solver.bestFeasSol.cost - 1e-6
                solver.bestFeasSol.routes = [new_route(solver, routes[r]) for r = 1:length(routes) if value(λ[r]) >= 0.9]
                solver.bestFeasSol.cost = objective_value(sp)
                solver.bestFeasSol.dist = 0#objective_value(sp)
                computeLabels(solver, solver.bestFeasSol)
            end
        end
        return solver.bestFeasSol
    end
end