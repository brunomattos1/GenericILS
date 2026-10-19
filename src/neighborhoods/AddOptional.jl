"""
Neighborhood that visits an optional vertex the solution leaves out, inserting it in the best position of a route.
The vertex of a group that has to stay in one route can only go to the route of its group. Whether it is worth
visiting is up to the resource (a profit, for instance): the move is applied only if it reduces the objective.

The candidates depend on which optional vertices are visited by the whole solution, not on a pair of routes, so
this neighborhood does not use the `lastEval` cache and evaluates every route each time.
"""
struct AddOptional end

"""
Best move found by `AddOptional` or `RemoveOptional`: objective `cost` and distance after the move, the `route`
and the position `idx` where `customer` enters (`AddOptional`) or from where it leaves (`RemoveOptional`).
"""
struct BestOptional
    cost::Float64
    dist::Float64
    route::Int
    idx::Int
    customer::Int
end
BestOptional(; cost::Float64 = Inf, dist::Float64 = Inf, route::Int = 0, idx::Int = 0, customer::Int = 0) =
    BestOptional(cost, dist, route, idx, customer)

"""
Refresh the bookkeeping of route `r` after its visits changed: labels, infeasibility, warps and label cost, and the
totals of `sol`, from which the old values of the route must already have been taken out.
"""
function updateRoute!(solver::Solver, sol::Solution, r::Int)
    computeLabels(solver, sol, r)
    rt = sol.routes[r]
    rt.infeas = length(rt.visits) - max(rt.feasibleF, rt.feasibleB) - 1
    rt.warpStd1 = rt.forwardLabels[end].std1State.stdWarp
    rt.warpStd2 = rt.forwardLabels[end].std2State.stdWarp
    rt.labelCost = min(rt.forwardLabels[end].cost, rt.backwardLabels[end].cost)
    sol.totalInfeas += rt.infeas
    sol.totalWarpStd1 += rt.warpStd1
    sol.totalWarpStd2 += rt.warpStd2
    sol.totalLabelCost += rt.labelCost
    return nothing
end

"""Take the bookkeeping of route `r` out of the totals of `sol`, before the route changes."""
function discountRoute!(sol::Solution, r::Int)
    rt = sol.routes[r]
    sol.totalInfeas -= rt.infeas
    sol.totalWarpStd1 -= rt.warpStd1
    sol.totalWarpStd2 -= rt.warpStd2
    sol.totalLabelCost -= rt.labelCost
    return nothing
end

"""Predicted penalized objective of inserting `customer` before position `j` of route `r`, `dist` being the distance after it."""
function evalAddOptional(solver::Solver, sol::Solution, r::Int, j::Int, customer::Int, dist::Float64)
    block = solver.bufferRoute
    resize!(block, 1)
    block[1] = customer
    infeas, labelCost, warpStd1, warpStd2 = infeasArcsInsertionK(solver, sol, r, block, j)
    return objectiveValue(solver, sol, r, dist, infeas, labelCost, warpStd1, warpStd2)
end

"""
Evaluate the insertion of every optional vertex that is not visited in every position of every route and apply, per
route, the best one if it improves the solution. Returns `true` when `sol` improved.
"""
function search!(neigh::AddOptional, solver::Solver, sol::Solution)
    data = solver.data
    data.hasOptional || return false
    visited = visitedVertices(data, sol)
    any(u -> !visited[u], data.optionalIds) || return false          # all of them are visited already
    flag = false
    copySolution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    groupRoute = data.hasLockedGroups ? routeOfGroups(data, sol) : Int[]
    resize!(solver.buffer, length(sol.routes))
    copyto!(solver.buffer, 1:length(sol.routes))
    shuffle!(solver.seed, solver.buffer)
    routesIdx = solver.buffer

    for r in routesIdx
        bestMove = BestOptional(dist = sol.dist, cost = sol.cost)
        route = sol.routes[r].visits
        len = length(route)
        canPrune = canPruneByDist(solver)
        fixedPenalty = canPrune ? pruningFixedPenalty(solver, sol, r) : 0.0
        for u in data.optionalIds
            visited[u] && continue
            g = data.groupOf[u]
            data.locked[g] && groupRoute[g] != 0 && groupRoute[g] != r && continue      # its group is in another route
            for j = 2:len
                dist = bestInsertionCost(sol.dist, data.costMatrix, route, u, j)
                if canPrune && dist + fixedPenalty >= bestMove.cost - 1e-6
                    continue
                end
                cost = evalAddOptional(solver, sol, r, j, u, dist)
                if cost < bestMove.cost - 1e-6
                    bestMove = BestOptional(cost, dist, r, j, u)
                end
            end
        end

        if bestMove.idx > 0
            prevCost = oldSol.cost
            apply!(neigh, solver, sol, bestMove)
            if sol.cost < prevCost - 1e-6
                flag = true
                visited[bestMove.customer] = true
                g = data.groupOf[bestMove.customer]
                data.locked[g] && (groupRoute[g] = r)
                copySolution!(oldSol, sol)
            else
                copySolution!(sol, oldSol)
            end
        end
    end
    return flag
end

"""For every group that has to stay in one route, the route where its visits are (0 if it has none)."""
function routeOfGroups(data::ProblemData, sol::Solution)
    groupRoute = zeros(Int, length(data.groups))
    for (r, rt) in enumerate(sol.routes)
        for v in rt.visits
            v != 0 && data.locked[data.groupOf[v]] && (groupRoute[data.groupOf[v]] = r)
        end
    end
    return groupRoute
end

"""Insert the vertex and refresh labels and bookkeeping of the route."""
function apply!(::AddOptional, solver::Solver, sol::Solution, move::BestOptional)
    discountRoute!(sol, move.route)
    sol.dist = move.dist
    insert!(sol.routes[move.route].visits, move.idx, move.customer)
    updateRoute!(solver, sol, move.route)
    sol.cost = objectiveValue(solver, sol)
    sol.routes[move.route].lastModif = sol.timeStamp
    return nothing
end
