"""
Neighborhood that removes a visited optional vertex from its route. Whether the visit is worth keeping is up to
the resource (a profit, for instance): the move is applied only if it reduces the objective. It depends on one
route only, so it uses the `lastEval` cache as the other neighborhoods do.
"""
struct RemoveOptional end

"""Distance after removing `route[i]` from `route`, from the current distance."""
function removeOptionalCost(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, i::Int)
    newCost = currCost
    newCost -= costMatrix[route[i-1]+1, route[i]+1]
    newCost -= costMatrix[route[i]+1,   route[i+1]+1]
    newCost += costMatrix[route[i-1]+1, route[i+1]+1]
    return newCost
end

"""Predicted penalized objective of removing the vertex at position `i` of route `r`, `dist` being the distance after it."""
function evalRemoveOptional(solver::Solver, sol::Solution, r::Int, i::Int, dist::Float64)
    infeas, labelCost, warpStd1, warpStd2 = infeasArcsRemovalK(solver, sol, r, i, 1)
    return objectiveValue(solver, sol, r, dist, infeas, labelCost, warpStd1, warpStd2)
end

"""
Evaluate the removal of every visited optional vertex and apply, per route, the best one if it improves the
solution. Returns `true` when `sol` improved.
"""
function search!(neigh::RemoveOptional, solver::Solver, sol::Solution)
    data = solver.data
    data.hasOptional || return false
    flag = false
    copySolution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    neighborhoodId = neighIndex(typeof(solver), RemoveOptional)
    resize!(solver.buffer, length(sol.routes))
    copyto!(solver.buffer, 1:length(sol.routes))
    shuffle!(solver.seed, solver.buffer)
    routesIdx = solver.buffer

    for r in routesIdx
        rt = sol.routes[r]
        sol.lastEval[neighborhoodId, r, r] > rt.lastModif && continue
        bestMove = BestOptional(dist = sol.dist, cost = sol.cost)
        route = rt.visits
        canPrune = canPruneByDist(solver)
        fixedPenalty = canPrune ? pruningFixedPenalty(solver, sol, r) : 0.0
        for i = 2:(length(route) - 1)
            data.isOptional[route[i]] || continue
            dist = removeOptionalCost(sol.dist, data.costMatrix, route, i)
            if canPrune && dist + fixedPenalty >= bestMove.cost - 1e-6
                continue
            end
            cost = evalRemoveOptional(solver, sol, r, i, dist)
            if cost < bestMove.cost - 1e-6
                bestMove = BestOptional(cost, dist, r, i, route[i])
            end
        end
        sol.timeStamp += 1
        sol.lastEval[neighborhoodId, r, r] = sol.timeStamp

        if bestMove.idx > 0
            prevCost = oldSol.cost
            apply!(neigh, solver, sol, bestMove)
            if sol.cost < prevCost - 1e-6
                flag = true
                copySolution!(oldSol, sol)
            else
                copySolution!(sol, oldSol)
            end
        end
    end
    return flag
end

"""Remove the vertex and refresh labels and bookkeeping of the route."""
function apply!(::RemoveOptional, solver::Solver, sol::Solution, move::BestOptional)
    discountRoute!(sol, move.route)
    sol.dist = move.dist
    deleteat!(sol.routes[move.route].visits, move.idx)
    updateRoute!(solver, sol, move.route)
    sol.cost = objectiveValue(solver, sol)
    sol.routes[move.route].lastModif = sol.timeStamp
    return nothing
end

"""Route of `sol` where group `g` has a visit, or `0` if it has none."""
function routeOfGroup(data::ProblemData, sol::Solution, g::Int)
    for (r, rt) in enumerate(sol.routes)
        for v in rt.visits
            v != 0 && data.groupOf[v] == g && return r
        end
    end
    return 0
end

"""
Perturbation of the optional visits: flip a random optional vertex, removing it if it is visited and inserting it
at a random position of a random route otherwise (the route of its group, when the group has to stay in one
route). Returns `false`, doing nothing, if there are no optional vertices.
"""
function randomToggleOptional!(solver::Solver, solution::Solution)
    data = solver.data
    data.hasOptional || return false
    u = data.optionalIds[rand(solver.seed, 1:length(data.optionalIds))]
    routes = solution.routes
    for r in eachindex(routes)
        visits = routes[r].visits
        for i = 2:length(visits)-1
            if visits[i] == u
                dist = removeOptionalCost(solution.dist, data.costMatrix, visits, i)
                apply!(RemoveOptional(), solver, solution, BestOptional(solution.cost, dist, r, i, u))
                return true
            end
        end
    end
    r = rand(solver.seed, 1:length(routes))
    g = data.groupOf[u]
    if data.locked[g]
        groupRoute = routeOfGroup(data, solution, g)
        groupRoute != 0 && (r = groupRoute)
    end
    visits = routes[r].visits
    j = rand(solver.seed, 2:length(visits))
    dist = bestInsertionCost(solution.dist, data.costMatrix, visits, u, j)
    apply!(AddOptional(), solver, solution, BestOptional(solution.cost, dist, r, j, u))
    return true
end
