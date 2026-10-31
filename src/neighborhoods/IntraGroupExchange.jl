"""
Neighborhood that replaces a visited member of a group by another member of the same group that is not
visited. Only regular members are exchanged: the optional ones are not in the quota of the group and have their
own neighborhoods (`AddOptional`, `RemoveOptional`). The new member goes to any position of any route, unless
the group must stay in one route (`sameRoute` with several visits): then it takes the place of the member it
replaces.
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
Predicted effect on route `r` of removing its member at `i`. It does not depend on the member that enters nor on
the route it goes to, so the search computes it once per position.
"""
struct Removal
    infeas::Int
    labelCost::Float64
    warpStd1::Float64
    warpStd2::Float64
end
Removal(solver::Solver, sol::Solution, r::Int, i::Int) = Removal(infeasArcsRemovalK(solver, sol, r, i, 1)...)

"""
Predicted penalized objective of removing the member at `i` of route `r1` (`removal` is its predicted effect)
and inserting `customer` before position `j` of another route `r2`. `dist` is the distance after the move.
"""
function evalIntraGroupExchange(solver::Solver, sol::Solution, r1::Int, r2::Int, j::Int, customer::Int, dist::Float64, removal::Removal)
    block = solver.bufferRoute
    resize!(block, 1)
    block[1] = customer
    infeasR2, labelCostR2, warpR2s1, warpR2s2 = infeasArcsInsertionK(solver, sol, r2, block, j)
    violInfo = ViolationInfo(removal.infeas, infeasR2, removal.labelCost, labelCostR2)
    return objectiveValue(solver, sol,
        Cost(dist, r1, r2, violInfo, (removal.warpStd1, warpR2s1), (removal.warpStd2, warpR2s2)))
end

"""Predicted penalized objective of replacing the member at `i` of route `r` by `customer`, in place."""
function evalIntraGroupReplace(solver::Solver, sol::Solution, r::Int, i::Int, customer::Int, dist::Float64)
    block = solver.bufferRoute
    resize!(block, 1)
    block[1] = customer
    infeas, labelCost, warpStd1, warpStd2 = infeasArcsReplaceBlockK(solver, sol, r, i, 1, block)
    return objectiveValue(solver, sol, r, dist, infeas, labelCost, warpStd1, warpStd2)
end

"""
Predicted penalized objective of the exchange of the member at `i` of route `r1` for `customer`, inserted before
position `j` of route `r2` (or in place of the member when `r1 == r2`, then `j` is ignored).
"""
function evalIntraGroupExchange(solver::Solver, sol::Solution, r1::Int, r2::Int, i::Int, j::Int, customer::Int, dist::Float64)
    r1 == r2 && return evalIntraGroupReplace(solver, sol, r1, i, customer, dist)
    return evalIntraGroupExchange(solver, sol, r1, r2, j, customer, dist, Removal(solver, sol, r1, i))
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
Evaluate every exchange of a visited regular vertex with an unvisited regular member of its group and apply, per route, the
best one if it improves the solution. Returns `true` when `sol` improved.
"""
function search!(neigh::IntraGroupExchange, solver::Solver, sol::Solution)
    data = solver.data
    data.hasExchangeableGroups || return false          # every regular vertex is visited: nothing to exchange
    visited = visitedVertices(data, sol)
    flag = false
    copySolution!(solver.bufferSol, sol)
    oldSol = solver.bufferSol
    neighborhoodId = neighIndex(typeof(solver), IntraGroupExchange)
    removals = Removal[]
    removalKnown = Bool[]
    resize!(solver.buffer, length(sol.routes))
    copyto!(solver.buffer, 1:length(sol.routes))
    shuffle!(solver.seed, solver.buffer)
    routesIdx = solver.buffer

    for r1 in routesIdx
        bestMove = BestExchange(dist = sol.dist, cost = sol.cost)
        route1 = sol.routes[r1].visits
        len1   = length(route1)
        len1 <= 2 && continue
        resize!(removals, len1)
        resize!(removalKnown, len1)
        fill!(removalKnown, false)
        for r2 in routesIdx
            sol.lastEval[neighborhoodId, r1, r2] > max(sol.routes[r1].lastModif, sol.routes[r2].lastModif) && continue
            route2 = sol.routes[r2].visits
            len2   = length(route2)
            canPrune = canPruneByDist(solver)
            fixedPenalty = canPrune ? (r1 == r2 ? pruningFixedPenalty(solver, sol, r1) : pruningFixedPenalty(solver, sol, r1, r2)) : 0.0
            for i = 2:(len1 - 1)
                data.isOptional[route1[i]] && continue          # optional vertices are not in the quota
                g = data.groupOf[route1[i]]
                r1 != r2 && data.locked[g] && continue
                for u in data.regularMembers[g]
                    visited[u] && continue
                    if r1 == r2
                        dist = intraGroupReplaceCost(sol.dist, data.costMatrix, route1, i, u)
                        if canPrune && dist + fixedPenalty >= bestMove.cost - 1e-6
                            continue
                        end
                        cost = evalIntraGroupReplace(solver, sol, r1, i, u, dist)
                        if cost < bestMove.cost - 1e-6
                            bestMove = BestExchange(cost, dist, r1, r2, i, i, u)
                        end
                    else
                        for j = 2:len2
                            dist = intraGroupExchangeCost(sol.dist, data.costMatrix, route1, route2, i, j, u)
                            if canPrune && dist + fixedPenalty >= bestMove.cost - 1e-6
                                continue
                            end
                            if !removalKnown[i]
                                removals[i] = Removal(solver, sol, r1, i)
                                removalKnown[i] = true
                            end
                            cost = evalIntraGroupExchange(solver, sol, r1, r2, j, u, dist, removals[i])
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
