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
            if length(solution.routes[r]) <= 2
                continue
            end
            if (solution.feasiblesF[r] == length(solution.routes[r]) - 1) || (solution.feasiblesB[r] == length(solution.routes[r]) - 1)
                routeHash = hash(solution.routes[r])
                if !haskey(solver.pool, solution.routes[r])#(!(routeHash in solver.hashes))
                    push!(solver.hashes, routeHash)
                    solver.pool[copy(solution.routes[r])] = solution.cost
                else
                    solver.pool[copy(solution.routes[r])] = min(solver.pool[solution.routes[r]], solution.cost)
                end
            end
        end
    else
        for r = 1:length(solution.routes)
            if length(solution.routes[r]) <= 2
                continue
            end
            # check if the route is feasible in the forward sense
            # if (solution.feasiblesF[r] == length(solution.routes[r]) - 1) && (solution.forwardLabels[r][end].std_res.stdWarp <= 1e-6)# || (solution.feasiblesB[r] == length(solution.routes[r]) - 2)
            #     routeHash = hash(solution.routes[r])
            #     if !haskey(solver.pool, solution.routes[r])#(!(routeHash in solver.hashes))
            #         push!(solver.hashes, routeHash)
            #         # push!(solver.pool, copy(solution.routes[r]))
            #         solver.pool[copy(solution.routes[r])] = solution.cost
            #     else
            #         solver.pool[copy(solution.routes[r])] = min(solver.pool[solution.routes[r]], solution.cost)
            #     end
            # end
            if (solution.feasiblesF[r] == length(solution.routes[r]) - 1) && (solution.forwardLabels[r][end].std_res.stdWarp <= 1e-6)
                route_id = get(solver.route_lookup, solution.routes[r], 0)

                if route_id > 0
                    if solution.cost < solver.cost_storage[route_id]
                        solver.cost_storage[route_id] = solution.cost
                    end
                else
                    persistent_route_copy = copy(solution.routes[r])
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
    return cost
end

function setPartitioning(solver::Solver, cutOff::Float64)
    sp = Model(CPLEX.Optimizer)
    set_optimizer_attribute(sp, "CPXPARAM_MIP_Tolerances_UpperCutoff", cutOff + 0.1)
    routes = solver.route_storage#collect(keys(solver.pool))
    @variable(sp, λ[r = 1:length(routes)], Bin)

    @objective(sp, Min, sum(c(solver, routes[r])*λ[r] for r = 1:length(routes)))

    @constraint(sp, [i = 1:length(solver.data.vertices)], sum(α(i, routes[r])λ[r] for r = 1:length(routes)) == 1)
    @constraint(sp, sum(λ[r] for r = 1:length(routes)) <= solver.data.maxNbRoutes)
    optimize!(sp)
    @show objective_value(sp)
    if termination_status(sp) == OPTIMAL
        # return Solution([routes[r] for r = 1:length(routes) if value(λ[r]) >= 0.9], objective_value(sp), [0 for i = 1:solver.data.maxNbRoutes], 0)
        solver.currSol.routes = [routes[r] for r = 1:length(routes) if value(λ[r]) >= 0.9]
        solver.currSol.cost = objective_value(sp)
        solver.currSol.dist = objective_value(sp)

        computeLabels(solver, solver.currSol)
    else
        if objective_value(sp) < solver.bestSol.cost - 0.001
            solver.currSol.routes = [routes[r] for r = 1:length(routes) if value(λ[r]) >= 0.9]
            solver.currSol.cost = objective_value(sp)
            computeLabels(solver)
        end
    end
end