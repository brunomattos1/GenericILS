"""
Neighborhood that replaces a visited member of a group by another member of the same group that is not
visited. The new member goes to any position of any route, unless the group must stay in one route
(`sameRoute` with `numVisits > 1`): then it takes the place of the member it replaces.
"""
struct IntraGroupExchange end

"""
Best exchange found by `IntraGroupExchange`: objective `cost`, distance, the routes and positions involved and
the `customer` that enters the solution. The member at `firstIdx` of `firstRoute` leaves; `customer` is
inserted before `secondIdx` of `secondRoute`, or takes its place when both routes are the same.
"""
struct BestExchange
    cost::Float64
    dist::Float64
    firstRoute::Int
    secondRoute::Int
    firstIdx::Int
    secondIdx::Int
    customer::Int
end
BestExchange(;
    cost::Float64 = Inf,
    dist::Float64 = Inf,
    firstRoute::Int = 0,
    secondRoute::Int = 0,
    firstIdx::Int = 0,
    secondIdx::Int = 0,
    customer::Int = 0
) = BestExchange(cost, dist, firstRoute, secondRoute, firstIdx, secondIdx, customer)

"""Distance after removing `route1[i]` and inserting `customer` before position `j` of `route2` (another route)."""
function intraGroupExchangeCost(currCost::Float64, costMatrix::Matrix{Float64}, route1::Vector{Int}, route2::Vector{Int}, i::Int, j::Int, customer::Int)
    newCost = currCost
    newCost -= costMatrix[route1[i-1]+1, route1[i]+1]
    newCost -= costMatrix[route1[i]+1,   route1[i+1]+1]
    newCost += costMatrix[route1[i-1]+1, route1[i+1]+1]
    newCost -= costMatrix[route2[j-1]+1, route2[j]+1]
    newCost += costMatrix[route2[j-1]+1, customer+1]
    newCost += costMatrix[customer+1,    route2[j]+1]
    return newCost
end

"""Distance after replacing `route[i]` by `customer` in the same position."""
function intraGroupReplaceCost(currCost::Float64, costMatrix::Matrix{Float64}, route::Vector{Int}, i::Int, customer::Int)
    newCost = currCost
    newCost -= costMatrix[route[i-1]+1, route[i]+1]
    newCost -= costMatrix[route[i]+1,   route[i+1]+1]
    newCost += costMatrix[route[i-1]+1, customer+1]
    newCost += costMatrix[customer+1,   route[i+1]+1]
    return newCost
end

"""
Predicted penalized objective of removing the member at `i` of route `r1` and inserting `customer` before
position `j` of route `r2`; `r1 == r2` replaces the member in place (`j` is ignored). `dist` is the distance
after the move.
"""
function evalIntraGroupExchange(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int, customer::Int, dist::Float64)
    block = solver.bufferRoute
    resize!(block, 1)
    block[1] = customer
    if r1 == r2
        infeas, labelCost, warpStd1, warpStd2 = infeasArcsReplaceBlockK(solver, sol, r1, i, 1, block)
        return objectiveValue(solver, sol, r1, dist, infeas, labelCost, warpStd1, warpStd2)
    end
    violInfo, warpR1s1, warpR2s1, warpR1s2, warpR2s2 = computeViolInterShiftK(solver, sol, r1, r2, i, j, 1, block)
    return objectiveValue(solver, sol,
        Cost(dist, r1, r2, violInfo, (warpR1s1, warpR2s1), (warpR1s2, warpR2s2)))
end

"""Flags of the vertices (by id) that are visited by `sol`."""
function visitedVertices(data::ProblemData, sol::Solution)
    visited = falses(length(data.groupOf))
    for rt in sol.routes
        for v in rt.visits
            v != 0 && (visited[v] = true)
        end
    end
    return visited
end

"""
Evaluate every exchange of a visited vertex with an unvisited member of its group and apply, per route, the
best one if it improves the solution. Returns `true` when `sol` improved.
"""
function search!(neigh::IntraGroupExchange, solver::Solver, sol::Solution)
    flag = false
    data = solver.data
    copySolution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    neighborhoodId = neighIndex(typeof(solver), IntraGroupExchange)
    visited = visitedVertices(data, sol)
    resize!(solver.buffer, length(sol.routes))
    copyto!(solver.buffer, 1:length(sol.routes))
    shuffle!(solver.seed, solver.buffer)
    routesIdx = solver.buffer

    for r1 in routesIdx
        bestMove = BestExchange(dist = sol.dist, cost = sol.cost)
        route1 = sol.routes[r1].visits
        len1   = length(route1)
        len1 <= 2 && continue
        for r2 in routesIdx
            sol.lastEval[neighborhoodId, r1, r2] > max(sol.routes[r1].lastModif, sol.routes[r2].lastModif) && continue
            route2 = sol.routes[r2].visits
            len2   = length(route2)
            canPrune = canPruneByDist(solver)
            fixedPenalty = canPrune ? (r1 == r2 ? pruningFixedPenalty(solver, sol, r1) : pruningFixedPenalty(solver, sol, r1, r2)) : 0.0
            for i = 2:(len1 - 1)
                g = data.groupOf[route1[i]]
                r1 != r2 && mustStayInRoute(data.groups[g]) && continue
                for u in data.members[g]
                    visited[u] && continue
                    if r1 == r2
                        dist = intraGroupReplaceCost(sol.dist, data.costMatrix, route1, i, u)
                        if canPrune && dist + fixedPenalty >= bestMove.cost - 1e-6
                            continue
                        end
                        cost = evalIntraGroupExchange(solver, sol, r1, r2, i, i, u, dist)
                        if cost < bestMove.cost - 1e-6
                            bestMove = BestExchange(cost, dist, r1, r2, i, i, u)
                        end
                    else
                        for j = 2:len2
                            dist = intraGroupExchangeCost(sol.dist, data.costMatrix, route1, route2, i, j, u)
                            if canPrune && dist + fixedPenalty >= bestMove.cost - 1e-6
                                continue
                            end
                            cost = evalIntraGroupExchange(solver, sol, r1, r2, i, j, u, dist)
                            if cost < bestMove.cost - 1e-6
                                bestMove = BestExchange(cost, dist, r1, r2, i, j, u)
                            end
                        end
                    end
                end
            end
            sol.timeStamp += 1
            sol.lastEval[neighborhoodId, r1, r2] = sol.timeStamp
        end

        if bestMove.firstIdx > 0
            prevCost = oldSol.cost
            leaving = route1[bestMove.firstIdx]
            apply!(neigh, solver, sol, bestMove)
            if sol.cost < prevCost - 1e-6
                flag = true
                visited[leaving] = false
                visited[bestMove.customer] = true
                copySolution!(oldSol, sol)
            else
                copySolution!(sol, oldSol)
            end
        end
    end
    return flag
end

"""
Exchange the members and refresh labels and bookkeeping of the routes. The exchange changes which members
of the group are visited, which the cached evaluations of every route with a member of the group depend
on, so those routes are marked as modified too.
"""
function apply!(::IntraGroupExchange, solver::Solver, sol::Solution, bestMove::BestExchange)
    r1, r2 = bestMove.firstRoute, bestMove.secondRoute
    i,  j  = bestMove.firstIdx,   bestMove.secondIdx
    u      = bestMove.customer
    g      = solver.data.groupOf[u]
    affected = r1 == r2 ? (r1,) : (r1, r2)

    sol.dist = bestMove.dist
    for r in affected
        rt = sol.routes[r]
        sol.totalInfeas    -= rt.infeas
        sol.totalWarpStd1  -= rt.warpStd1
        sol.totalWarpStd2  -= rt.warpStd2
        sol.totalLabelCost -= rt.labelCost
    end

    if r1 == r2
        sol.routes[r1].visits[i] = u
        computeLabels(solver, sol, r1)
    else
        deleteat!(sol.routes[r1].visits, i)
        insert!(sol.routes[r2].visits, j, u)
        computeLabels(solver, sol, r1, r2)
    end

    for r in affected
        rt = sol.routes[r]
        rt.infeas = length(rt.visits) - max(rt.feasibleF, rt.feasibleB) - 1
        rt.warpStd1 = rt.forwardLabels[end].std1State.stdWarp
        rt.warpStd2 = rt.forwardLabels[end].std2State.stdWarp
        rt.labelCost = min(rt.forwardLabels[end].cost, rt.backwardLabels[end].cost)
        sol.totalInfeas    += rt.infeas
        sol.totalWarpStd1  += rt.warpStd1
        sol.totalWarpStd2  += rt.warpStd2
        sol.totalLabelCost += rt.labelCost
    end

    sol.cost = objectiveValue(solver, sol)
    for rt in sol.routes
        for v in rt.visits
            if v != 0 && solver.data.groupOf[v] == g
                rt.lastModif = sol.timeStamp
                break
            end
        end
    end
    for r in affected
        sol.routes[r].lastModif = sol.timeStamp
    end
end
